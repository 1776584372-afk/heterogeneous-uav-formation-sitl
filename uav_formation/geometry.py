"""固定队形槽位生成 —— 可扩展到任意机数(N≤10), 不变形(单一队形贯穿全程)。

槽位坐标系: 以【虚拟中心】为原点, x 向前(编队巡航方向), y 向右, z 向上。
uav0(leader) 默认在队形最前/顶点; 其余机按队形规则排在后方两侧。

所有函数返回 list[(x, y, z)], 长度 = n。
"""
import math
import itertools

# ---- 一字横排 (LINE): 沿 y 展开, 中心对称 ----
def line_slots(n, spacing):
    return [(0.0, spacing * (i - (n - 1) / 2.0), 0.0) for i in range(n)]

# ---- 纵队 (COLUMN): 沿 x 展开, uav0 在最前 ----
def column_slots(n, spacing):
    return [(-spacing * i, 0.0, 0.0) for i in range(n)]

# ---- V 字 (WEDGE): uav0 在顶点, 左右两翼向后展开 ----
def wedge_slots(n, spacing):
    if n <= 1:
        return [(0.0, 0.0, 0.0)]
    slots = [(0.0, 0.0, 0.0)]
    left = (n - 1 + 1) // 2          # 左翼机数(含优先)
    right = (n - 1) // 2             # 右翼机数
    k = 1
    while len(slots) < n:
        if k <= left:
            slots.append((-spacing * k, spacing * k * 0.6, 0.0))
        if len(slots) < n and k <= right:
            slots.append((-spacing * k, -spacing * k * 0.6, 0.0))
        k += 1
    return slots

# ---- 方阵 (GRID): 尽量接近方阵, 行向 y 展开、列向 x 后退 ----
def grid_slots(n, spacing, cols=None):
    if cols is None:
        cols = max(2, int(math.ceil(math.sqrt(n))))
    rows = (n + cols - 1) // cols
    slots = []
    for r in range(rows):
        for c in range(cols):
            if len(slots) >= n:
                break
            y = spacing * (c - (cols - 1) / 2.0)
            slots.append((-spacing * r, y, 0.0))
    return slots

# ---- 圆环 (CIRCLE): 均匀分布在半径为 radius 的圆上, uav0 在正前方 ----
def circle_slots(n, radius):
    if n <= 1:
        return [(radius, 0.0, 0.0)]
    slots = []
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        slots.append((radius * math.cos(ang), radius * math.sin(ang), 0.0))
    return slots

SHAPES = {
    "line": line_slots,
    "column": column_slots,
    "wedge": wedge_slots,
    "grid": grid_slots,
    "circle": circle_slots,
}


def make_slots(shape, n, spacing=1.5, radius=5.0):
    """统一入口: shape ∈ line/column/wedge/grid/circle, 返回长度 n 的槽位表。"""
    if shape not in SHAPES:
        raise ValueError(f"未知队形 {shape}, 可选: {sorted(SHAPES)}")
    if shape == "circle":
        slots = SHAPES[shape](n, radius)
    else:
        slots = SHAPES[shape](n, spacing)
    if len(slots) != n:
        raise ValueError(f"队形 {shape} 生成 {len(slots)} 个槽位, 与机数 {n} 不符")
    return slots


def rotate_slot(slot, yaw):
    """槽位绕 z 旋转 yaw 弧度(队形整体朝向), 返回 (x, y)。"""
    x, y, _ = slot
    return (x * math.cos(yaw) - y * math.sin(yaw),
            x * math.sin(yaw) + y * math.cos(yaw))


def formation_rms(actuals, targets):
    """队形 RMS: 各机实际位置与目标槽位(公共系)的均方根距离。"""
    if not actuals or len(actuals) != len(targets):
        return float("nan")
    return math.sqrt(sum((a[0] - t[0]) ** 2 + (a[1] - t[1]) ** 2
                         for a, t in zip(actuals, targets)) / len(actuals))


def assign_slots(slots, positions):
    """§8.14 成形防对穿: 最近槽位指派。

    起飞排列与本次编队朝向不匹配时, 按机号固定的槽位会让两机对穿换位
    (实测 1/3 号机对穿, 合拢速度 ~12 m/s)。本函数按"总转移距离最小"
    指派槽位: 每机分到离自己最近的槽位, 零横穿。

    slots/positions: 同一坐标系下的 [(x, y), ...], 等长。
    返回 perm: 第 i 机分到 slots[perm[i]] 的槽位下标列表。
    N≤6 全排列枚举取最优; N>6 贪心(全局最近对逐对摘取)。
    """
    n = len(positions)
    if n == 0:
        return []
    d = [[math.hypot(positions[i][0] - slots[j][0],
                     positions[i][1] - slots[j][1]) for j in range(n)]
         for i in range(n)]
    if n <= 6:
        best_perm, best_s = None, None
        for perm in itertools.permutations(range(n)):
            s = sum(d[i][perm[i]] for i in range(n))
            if best_s is None or s < best_s:
                best_s, best_perm = s, perm
        return list(best_perm)
    free_i, free_j = set(range(n)), set(range(n))
    perm = [0] * n
    for _ in range(n):
        bi, bj, bv = None, None, None
        for i in sorted(free_i):
            for j in sorted(free_j):
                if bv is None or d[i][j] < bv:
                    bi, bj, bv = i, j, d[i][j]
        perm[bi] = bj
        free_i.discard(bi)
        free_j.discard(bj)
    return perm
