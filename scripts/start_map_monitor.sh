#!/usr/bin/env bash
# ============================================================
# start_map_monitor.sh —— MAVProxy 地图监控（serial2 通道, 只读）
#
# 背景（2026-08-28 桥定稿后）:
#   serial0(5760+10i) 已由 fc_udp_bridge.py 独占(地面站命令入口 14550/14600),
#   MAVProxy 不能再连 serial0 —— 会抢 SITL"第一个客户端", 弄挂地面站链路。
#   本脚本让 MAVProxy 连 serial2 调试口(5763+10i): 3 机同图(地图) + 控制台,
#   只读监控, 不干扰桥/地面站/mission 上传。
#   ⚠️ 命令入口仍是 14550(桥); 本监控实例只用于"看", 别从这里发指令。
#
# 用法:
#   ./scripts/start_map_monitor.sh             # 前台: 地图+控制台窗口(Ctrl+C 停)
#   ./scripts/start_map_monitor.sh --log       # 后台: 日志 ~/multi_uav_logs/map_monitor.log
#   ./scripts/start_map_monitor.sh --drones 5  # 换机数(端口自动推导 5763+10*i)
#   ./scripts/start_map_monitor.sh --stop      # 停掉监控实例(mavproxy.py)
#
# 想随 start_all.sh 一起起: 在启动后加一句  ./scripts/start_map_monitor.sh --log
# ============================================================
set -u

N_DRONES=3
BASE_PORT2=5763                    # 第 0 架 serial2 端口; 第 i 架 = BASE_PORT2 + 10*i
LOG_DIR="$HOME/multi_uav_logs"
mkdir -p "$LOG_DIR"
MODE="foreground"

while [ $# -gt 0 ]; do
    case "$1" in
        --log)          MODE="log"; shift ;;
        --stop)         MODE="stop"; shift ;;
        --drones)       N_DRONES="$2"; shift 2 ;;
        --drones=*)     N_DRONES="${1#*=}"; shift ;;
        *) echo "[!] 未知参数: $1 (支持 --log / --stop / --drones N)"; exit 2 ;;
    esac
done
case "$N_DRONES" in
    ''|*[!0-9]*) echo "[!] --drones 需要是数字"; exit 2 ;;
esac

# 停监控(只停 mavproxy.py —— 本脚本只管监控实例, 不动 SITL/桥/MAVROS/节点)
if [ "$MODE" = "stop" ]; then
    pkill -f '[m]avproxy.py' 2>/dev/null
    sleep 1
    if pgrep -f '[m]avproxy.py' >/dev/null; then
        echo "[!] 仍有 mavproxy 存活, 请确认: ps aux | grep mavproxy"
        exit 1
    fi
    echo "监控已停止。"
    exit 0
fi

command -v mavproxy.py >/dev/null 2>&1 || { echo "[!] 找不到 mavproxy.py (装了吗?)"; exit 1; }

# 防重复: 启动前清掉残留监控实例(只清 mavproxy.py, 不动 SITL/桥/MAVROS/节点)
pkill -f '[m]avproxy.py' 2>/dev/null; sleep 0.5

# serial2 端口预检(5763+10*i 未监听说明 SITL 没起)
miss=0
for ((i=0; i<N_DRONES; i++)); do
    p=$((BASE_PORT2 + 10*i))
    if ! ss -tln 2>/dev/null | grep -q ":$p "; then
        echo "[!] serial2 端口 $p 未监听 —— SITL 起过吗? (./scripts/start_all.sh)"
        miss=1
    fi
done
[ "$miss" = "1" ] && exit 1

# ★ v9.8hou §8.8: 改从桥的本地 UDP 镜像(14551)读遥测 —— 旧方案直连
#   serial2 TCP, MAVProxy(GUI) 读慢时 TCP 背压会阻塞飞控主循环, 仿真时钟
#   坍缩到 3%(实测)。UDP 镜像单向推送永不阻塞飞控; 单 master 收全部 N 机
#   (帧内 sysid 区分, MAVProxy 原生支持多机)。
masters="--master udpin:127.0.0.1:14551"

# 默认模块清单 = 官方默认去掉 wp(wp 会吞 MISSION_ITEM_INT, 监控实例不需要航点)
MODS="log,signing,rally,fence,ftp,param,relay,tuneopt,arm,mode,calibration,rc,auxopt,misc,cmdlong,battery,terrain,output,adsb,layout"

# ★ --map(地图窗口) 与 --console(状态控制台窗口) 两个都要(MAVProxy console 是独立 Tk 窗口,
#   不是终端里的 curses, 后台模式同样正常弹出, 不会污染日志文件)
WIN_ARGS=""
if [ -n "${DISPLAY:-}" ]; then
    WIN_ARGS="--map --console"
else
    echo "  [- ] 无 DISPLAY, 地图/控制台窗口起不来(仅记录日志)"
fi
ARGS="$masters $WIN_ARGS --streamrate 10 --default-modules=$MODS"

echo "==== 启动地图监控 (MAVProxy ⇄ serial2, $N_DRONES 架, 只读) ===="
echo "  连接: tcp:127.0.0.1:$BASE_PORT2+10*i (serial2 调试口)"
echo "  说明: 地图/控制台窗口只读; 发指令请走 14550 桥入口"

if [ "$MODE" = "log" ]; then
    tail -f /dev/null | mavproxy.py $ARGS > "$LOG_DIR/map_monitor.log" 2>&1 &
    echo "  已后台启动, 日志: $LOG_DIR/map_monitor.log (停止: $0 --stop)"
    echo "  地图/控制台窗口已弹出(若有 DISPLAY); 关闭窗口不影响进程"
else
    echo "  Ctrl+C 停止"
    # tail -f /dev/null 保持 stdin 打开(MAVProxy 读 EOF 会卸载全部模块并退出)
    tail -f /dev/null | mavproxy.py $ARGS
fi
