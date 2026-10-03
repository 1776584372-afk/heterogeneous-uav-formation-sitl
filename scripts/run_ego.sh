#!/usr/bin/env bash
# ============================================================
# run_ego.sh —— EGO-planner 中心轨迹 + NMPC 全员跟随 + ArduPilot 桥 一键启动
# (v9.8hou §8.2 无领机架构: EGO 规划【虚拟中心】, 全部 N 架 NMPC 跟随, 槽位全对称)
#
# 链路:
#   [授权 armed+GUIDED] → ego_env_bridge 发布均值中心 odom(门控, 掉机留洞)
#     → bamboo ego_planner_node(/drone_0 命名空间, 规划【编队中心】, 无障碍空图)
#     → traj_server → /formation_center/position_cmd
#     → egompc_formation(全部 N 架 NMPC: 中心沿轨迹推进 + Rot(ψ)·槽位_i 刚体旋转,
#       拐弯撕裂的抑制机制) → /drone_{0..N-1}/position_cmd
#     → poscmd_bridge → /uav{i}/mavros/setpoint_position/local → 飞控
#
# ⚠️ 前置与约束:
#   1. 先起 ArduPilot 堆栈【不带编队节点】: ./scripts/start_all.sh  (不加 --with-formation!)
#      drone_node 与 poscmd_bridge 会抢同一 setpoint 话题, 本脚本检测到 drone_node 直接退出。
#   2. 需 ~/bamboo_square_ws/install(EGO-planner ROS2 移植, 已预编译)。
#   3. 地面站授权后 EGO 自动规划并起飞; 收回控制=ground_station revoke。
#
# 用法:
#   ./scripts/run_ego.sh                       # 3 架, preset 模式: 5 点多弯道航线 z=2(默认)
#   ./scripts/run_ego.sh --mission             # ★ 地面站上传航线模式: GCS 画线→上传→auth
#   ./scripts/run_ego.sh --n-drones 2 --z 5
#   ./scripts/run_ego.sh --formation column --spacing 5
#   ./scripts/run_ego.sh --route 5,0,10 35,0,10 45,30,10
#   ./scripts/run_ego.sh --speed 1.5           # EGO 规划巡航速度 m/s
#   ./scripts/run_ego.sh --stop                # 停止 EGO 链路
#   ★ 2026-09-01: 每次启动前自动清旧 EGO 链路(干净启动, 防 4 套并存互发设定点);
#     --stop 与 Ctrl+C 同样全清(两段式 TERM→KILL)。
set -u

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAMBOO="$HOME/bamboo_square_ws"
LOG_DIR="$HOME/multi_uav_logs"
mkdir -p "$LOG_DIR"

N_DRONES=3
Z=2.0     # ★ 默认起飞高度 m(2026-09-04 对齐 v13: 原 10 易卡门控)。NAV_TAKEOFF 默认=2m;
          #   mission 模式航点带 z 时由桥/egompc 对齐到航点高度
SPEED=1.5
HOME_SPACING=10.0
FORMATION=line
SPACING=10.0
TURN_RADIUS=0   # ★ ICC 转弯半径 m(0=自动取 半宽+5; 调小可减少拐角内切, 但内侧机半径变小)
V_MAX=30.0      # ★ 单机速度硬上界 m/s(设计上限, 可再调; 飞控 WPNAV_SPEED 才是真瓶颈)
A_N_MAX=5.0     # ★ 最大法向加速度 m/s²(转弯不失速权衡, 与飞控 WP_ACC=5 对齐; 调大转弯更紧)
TURN_MARGIN=5.0 # ★ 转弯半径几何裕量 m(沿用"半宽+5"的 5)
ROUTE=""    # 空=默认 5 点多弯道(x,y 中心线, z 用 --z)
MISSION=0   # 1=地面站上传航线模式(flight_type=1, 桥拉全 FC 航线一致性校验后逐点 /goal)
MISSION_MAP_SIZE=400.0  # ★ mission 模式地图边长 m(默认 400, 覆盖 ±200m 航线; §8.8 选项 b)

# ★ EGO 链路全量清理(2026-09-02, 干净启动/--stop/Ctrl+C 三处共用):
#   旧 egompc/poscmd_bridge 不清会与新链路同时往 /uav{i} 发设定点
#   (实测多套并存互发矛盾目标 → 飞机集体"死机")。TERM→1.5s→KILL 两段式。
#   ★ 不杀旧 run_ego bash 本身: 其子组件被清后 wait 自然返回退出 ——
#     若杀它, 旧实例的 cleanup 陷阱会在临死前反杀新实例(实测级联互杀,
#     新脚本刚起步就被旧实例的 clean_ego 干掉)。
clean_ego() {
    local pat
    local pats=('[e]go_planner_node' '[p]lan_manage traj_server'
                '[p]lan_manage/traj_server' '[e]gompc_formation'
                '[e]go_env_bridge' '[p]oscmd_bridge' '[l]eader_traj')
    for pat in "${pats[@]}"; do pkill -f "$pat" 2>/dev/null; done
    sleep 1.5
    for pat in "${pats[@]}"; do pkill -9 -f "$pat" 2>/dev/null; done
}

while [ $# -gt 0 ]; do
  case "$1" in
    --n-drones) N_DRONES="$2"; shift 2 ;;
    --z) Z="$2"; shift 2 ;;
    --speed) SPEED="$2"; shift 2 ;;
    --home-spacing) HOME_SPACING="$2"; shift 2 ;;
    --formation) FORMATION="$2"; shift 2 ;;
    --turn-radius) TURN_RADIUS="$2"; shift 2 ;;
    --v-max) V_MAX="$2"; shift 2 ;;
    --a-n-max) A_N_MAX="$2"; shift 2 ;;
    --turn-margin) TURN_MARGIN="$2"; shift 2 ;;
    --mission-map-size) MISSION_MAP_SIZE="$2"; shift 2 ;;
    --spacing) SPACING="$2"; shift 2 ;;
    --mission) MISSION=1; shift ;;
    --route) shift; ROUTE=""; while [ $# -gt 0 ] && [[ "$1" != --* ]]; do ROUTE+="$1 "; shift; done ;;
    --stop) clean_ego; echo "EGO 链路已停止"; exit 0 ;;
    *) echo "[!] 未知参数: $1"; exit 2 ;;
  esac
done

if [ ! -d "$BAMBOO/install" ]; then
    echo "[!] 缺少 $BAMBOO/install —— 先编译 bamboo_square_ws(EGO-planner 参考)"
    exit 1
fi
if pgrep -f 'drone_node.py' > /dev/null; then
    echo "[!] 检测到 drone_node 在跑 —— 与 poscmd_bridge 抢 setpoint 话题!"
    echo "    EGO 模式须不带编队节点: ./scripts/start_all.sh  (无 --with-formation)"
    exit 1
fi

# ★ 干净启动(2026-09-01): 每次启动先清旧 EGO 链路 —— 残留的 egompc/poscmd_bridge
#   会与新链路同时往 /uav{i} 发设定点(实测 4 套并存 → 飞机集体"死机")。
#   drone_node 守卫在上, 先守卫后清理(有 drone_node 时原样退出, 不动任何进程)。
echo "==== 清理旧 EGO 链路(干净启动) ===="
clean_ego
echo "  清理完成"

# 地图尺寸: preset 模式按航线包络; mission 模式(上传航线)无法预知包络 →
# 固定 200x200m + 分辨率 0.25(内存 ~0.5GB/规划器; 分辨率粗一点不影响无障碍规划)
if [ "$MISSION" = "1" ]; then
    FLIGHT_TYPE=1
    if [ -z "$ROUTE" ]; then
        MINX=-100; MINY=-100; MAXX=100; MAXY=100
        MAP_SX=$MISSION_MAP_SIZE; MAP_SY=$MISSION_MAP_SIZE
    fi
    MAP_RES=0.25
else
    FLIGHT_TYPE=2
    MAP_RES=0.1
fi
if [ "$MISSION" != "1" ] || [ -n "$ROUTE" ]; then
    # 默认航线(preset 模式用; mission 模式带 --route 时仅用于地图包络)
    if [ -z "$ROUTE" ]; then
        ROUTE="5,0 35,0 45,30 25,50 5,30"
    fi
    read -r MINX MINY MAXX MAXY <<< "$(python3 - <<PYEOF
pts = [tuple(map(float, p.split(','))) for p in "$ROUTE".split()]
xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
m = 12.0
print(min(xs)-m, min(ys)-m, max(xs)+m, max(ys)+m)
PYEOF
)"
    MAP_SX=$(python3 -c "print(max(30.0, round($MAXX - $MINX, 1)))")
    MAP_SY=$(python3 -c "print(max(30.0, round($MAXY - $MINY, 1)))")
fi
MAP_SZ=$(python3 -c "print(max(3.0, $Z + 2.0))")
# ★ 归一为浮点: 规划器 grid_map 参数是 double, 整数字面量会触发
#   InvalidParameterTypeException 启动即崩(实测)
MAP_SX=$(python3 -c "print('%.1f' % float('$MAP_SX'))")
MAP_SY=$(python3 -c "print('%.1f' % float('$MAP_SY'))")
MAP_SZ=$(python3 -c "print('%.1f' % float('$MAP_SZ'))")
MODE_TXT="preset 航线(--route)"; [ "$MISSION" = "1" ] && MODE_TXT="地面站上传航线(--mission)"
echo "==== EGO 链路启动: N=$N_DRONES, z=${Z}m, speed=${SPEED}m/s, 模式=$MODE_TXT, 地图 ${MAP_SX}x${MAP_SY}x${MAP_SZ}@${MAP_RES} ===="

# ★ ROS setup.bash 引用未定义变量(AMENT_TRACE_SETUP_FILES 等), 必须在非严格
#   模式下 source(实测: set -u 下直接报"未绑定的变量"退出), source 完再恢复。
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_mavros_ws/install/setup.bash"
source "$BAMBOO/install/setup.bash"
# ★ v9.8hou 补丁版 plan_manage 覆盖(§8.2: MANUAL 模式 trigger 订阅修复 ——
#   原实现只在 PRESET 分支订阅 /traj_start_trigger, 逐点 /goal 模式飞完一段
#   后 have_trigger_ 永久 false, 卡死 WAIT_TARGET 只能飞一个目标)
source "$HOME/bamboo_user/install/setup.bash"
set -u
export PYTHONPATH="$WS:${PYTHONPATH:-}"

PIDS=()
cleanup() {
    echo ""
    echo "停止 EGO 链路..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null; done
    clean_ego
    echo "已停止"
}
trap cleanup INT TERM

# ---------- ① EGO 领机规划器(drone_0, 预设航点) ----------
PLANNER_ARGS=(
    -r __ns:=/drone_0
    -p fsm.flight_type:=$FLIGHT_TYPE
    -p fsm.realworld_experiment:=false
    -p fsm.thresh_replan_time:=1.0
    -p fsm.thresh_no_replan_meter:=1.0
    -p fsm.planning_horizon:=7.5
    -p fsm.emergency_time:=1.0
    -p fsm.fail_safe:=true
    -p grid_map.resolution:=$MAP_RES
    -p grid_map.map_size_x:=$MAP_SX
    -p grid_map.map_size_y:=$MAP_SY
    -p grid_map.map_size_z:=$MAP_SZ
    -p grid_map.local_update_range_x:=5.5
    -p grid_map.local_update_range_y:=5.5
    -p grid_map.local_update_range_z:=4.5
    -p grid_map.obstacles_inflation:=0.0
    -p grid_map.ground_height:=-0.01
    -p grid_map.frame_id:=world
    -p grid_map.pose_type:=1
    -p manager.max_vel:=$SPEED
    -p manager.max_acc:=6.0
    -p manager.polyTraj_piece_length:=1.5
    -p manager.feasibility_tolerance:=0.05
    -p manager.planning_horizon:=7.5
    -p manager.drone_id:=0
    -p optimization.constraint_points_perPiece:=5
    -p optimization.weight_obstacle:=10000.0
    -p optimization.weight_swarm:=0.0
    -p optimization.weight_feasibility:=10000.0
    -p optimization.weight_time:=10.0
    -p optimization.weight_formation:=0.0
    -p optimization.obstacle_clearance:=0.1
    -p optimization.swarm_clearance:=0.5
    -p optimization.max_vel:=$SPEED
    -p optimization.max_acc:=6.0
    -p formation.num:=1
    -p formation.start:='[0.0, 0.0, 0.0]'
    -r odom_world:=/formation_center/odom
    -r grid_map/odom:=/formation_center/odom
    -r grid_map/cloud:=/drone_0/pcl_render_node/cloud
    -r planning/broadcast_traj_send:=/broadcast_traj
    -r planning/broadcast_traj_recv:=/broadcast_traj
    -r mandatory_stop:=/mandatory_stop_to_planner
)
# 航点参数(fsm.waypointN_x/y/z) —— 仅 preset 模式需要
WP_N=0
if [ "$MISSION" != "1" ]; then
    for p in $ROUTE; do
        IFS=',' read -r wx wy <<< "$p"
        PLANNER_ARGS+=(-p "fsm.waypoint${WP_N}_x:=$wx" -p "fsm.waypoint${WP_N}_y:=$wy" -p "fsm.waypoint${WP_N}_z:=$Z")
        WP_N=$((WP_N+1))
    done
    PLANNER_ARGS+=(-p "fsm.waypoint_num:=$WP_N")
fi

if [ "$MISSION" = "1" ]; then
    echo "==== ① ego_planner_node (/drone_0, flight_type=1 手动目标: /goal 逐点) ===="
else
    echo "==== ① ego_planner_node (/drone_0, flight_type=2, $WP_N 预设航点) ===="
fi
ros2 run plan_manage ego_planner_node --ros-args "${PLANNER_ARGS[@]}" \
    > "$LOG_DIR/ego_planner.log" 2>&1 &
PIDS+=($!)

echo "==== ② traj_server (/drone_0 → 中心轨迹 /formation_center/position_cmd) ===="
ros2 run plan_manage traj_server --ros-args -r __ns:=/drone_0 \
    -p traj_server.time_forward:=1.0 -r heartbeat:=planning/heartbeat \
    -r position_cmd:=/formation_center/position_cmd \
    > "$LOG_DIR/ego_traj_server.log" 2>&1 &
PIDS+=($!)

sleep 2
echo "==== ③ ego_env_bridge (均值中心 odom 门控 + 空点云) ===="
BRIDGE_ARGS=(--uav-index 0 --n-drones "$N_DRONES" --formation "$FORMATION" \
             --spacing "$SPACING" --home-spacing "$HOME_SPACING" --z "$Z")
if [ "$MISSION" = "1" ]; then
    BRIDGE_ARGS+=(--mission-goals)
fi
PYTHONUNBUFFERED=1 python3 "$WS/uav_formation/ego_env_bridge.py" "${BRIDGE_ARGS[@]}" \
    > "$LOG_DIR/ego_env_bridge.log" 2>&1 &
PIDS+=($!)

sleep 1
echo "==== ④ egompc_formation ($N_DRONES 架全 NMPC, $FORMATION 槽位 ${SPACING}m, ICC 转弯) ===="
# ★ ICC 转弯平滑化: egompc 自带解析 ICC 参考轨迹(直线+圆弧+五次过渡);
#   preset 模式命令行直传 --route, mission 模式经桥 /formation/route 下发。
EGOMPC_ARGS=(--n-drones "$N_DRONES" --formation "$FORMATION" --spacing "$SPACING" \
             --home-spacing "$HOME_SPACING" \
             --fly-z "$Z" --speed "$SPEED" --turn-radius "$TURN_RADIUS" \
             --v-max "$V_MAX" --a-n-max "$A_N_MAX" --turn-margin "$TURN_MARGIN")
if [ "$MISSION" != "1" ]; then
    EGOMPC_ARGS+=(--route "$ROUTE")
fi
PYTHONUNBUFFERED=1 python3 "$WS/uav_formation/egompc_formation.py" \
    "${EGOMPC_ARGS[@]}" \
    > "$LOG_DIR/ego_egompc.log" 2>&1 &
PIDS+=($!)

sleep 1
GATE_ALT=$(python3 -c "print(round(0.9 * $Z, 1))")
echo "==== ⑤ poscmd_bridge (N=$N_DRONES, home 间距 ${HOME_SPACING}m, 起飞门控 ${GATE_ALT}m) ===="
PYTHONUNBUFFERED=1 python3 "$WS/uav_formation/poscmd_bridge.py" \
    --n-drones "$N_DRONES" --home-spacing "$HOME_SPACING" --gate-alt "$GATE_ALT" \
    > "$LOG_DIR/ego_poscmd.log" 2>&1 &
PIDS+=($!)

echo ""
echo "=============================================="
echo "EGO 链路已启动(5 组件, 日志: $LOG_DIR/ego_*.log)"
if [ "$MISSION" = "1" ]; then
    echo "下一步(地面站上传航线模式):"
    echo "  ① 地面站上传航线(桥 14550 → 各飞控; 与普通编队模式完全一致)"
    echo "  ② 地面站授权: python3 scripts/ground_station.py --port 14550 auth"
    echo "  ③ 授权后: 桥拉取 FC1 航线 → 逐点 /goal → EGO 规划 → 全队起飞跟随"
    echo "     到位自动切下一点; 进度看 $LOG_DIR/ego_env_bridge.log"
else
    echo "下一步(preset 航线模式):"
    echo "  ① 地面站授权: python3 scripts/ground_station.py --port 14550 auth"
    echo "     (或 --all $N_DRONES auth; 解锁+GUIDED)"
    echo "  ② 授权后 ego_env_bridge 开门控 → EGO 自动规划预设航线 → 全队起飞跟随"
fi
echo "  收回: ground_station.py revoke LOITER; 停链路: $0 --stop"
echo "=============================================="
wait
