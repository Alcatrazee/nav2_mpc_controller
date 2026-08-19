import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 模拟场景：参考线 y=0 (x in [0, 10]) 直接穿过一个大障碍物！
# 障碍物 1: x in [3.0, 7.0], y in [-0.5, 0.15] (障碍物直接覆盖了参考线 y=0, 从 -0.5m 一直挡到 +0.15m)
# 此时在 x in [3, 7] 处，安全走廊必须被整体压缩并推到左侧 y in [0.20, 0.80] (即 d_min=+0.20, d_max=+0.80)!

def is_obstacle(x, y):
    # 障碍物 1: 穿过参考线
    if 3.0 <= x <= 7.0 and -0.6 <= y <= 0.15:
        return True
    # 左侧远端墙壁
    if y >= 0.85:
        return True
    # 右侧远端墙壁
    if y <= -0.85:
        return True
    return False

# 生成参考线
x_pts = np.linspace(0.5, 9.5, 91)
y_pts = np.zeros_like(x_pts)
theta_pts = np.zeros_like(x_pts)

d_min_raw = []
d_max_raw = []

# 全截面扫描算法
d_range = np.linspace(-0.8, 0.8, 81) # 2cm 分辨率

prev_center = 0.0

for x, y, th in zip(x_pts, y_pts, theta_pts):
    norm_x = -np.sin(th)
    norm_y = np.cos(th)
    
    # 寻找截面上的所有自由连续区间
    free_intervals = []
    in_free = False
    start_d = None
    
    for d in d_range:
        test_x = x + d * norm_x
        test_y = y + d * norm_y
        obs = is_obstacle(test_x, test_y)
        
        if not obs:
            if not in_free:
                in_free = True
                start_d = d
        else:
            if in_free:
                in_free = False
                free_intervals.append((start_d, d - 0.02))
    if in_free:
        free_intervals.append((start_d, d_range[-1]))
        
    if not free_intervals:
        d_min_raw.append(-0.1)
        d_max_raw.append(0.1)
        continue
        
    # 选择最匹配连续性的区间 (优先选择覆盖 prev_center 或离 prev_center 最近且足够宽的区间)
    best_interval = None
    min_dist_to_center = 1e9
    for (d1, d2) in free_intervals:
        width = d2 - d1
        if width < 0.15: # 过滤过窄缝隙
            continue
        c = 0.5 * (d1 + d2)
        dist = abs(c - prev_center)
        if dist < min_dist_to_center:
            min_dist_to_center = dist
            best_interval = (d1, d2)
            
    if best_interval is None:
        best_interval = max(free_intervals, key=lambda iv: iv[1] - iv[0])
        
    d_min_raw.append(best_interval[0])
    d_max_raw.append(best_interval[1])
    prev_center = 0.5 * (best_interval[0] + best_interval[1])

d_min = np.array(d_min_raw)
d_max = np.array(d_max_raw)

# 双向最大坡度平滑
max_rate = 0.4
for i in range(1, len(d_min)):
    dx = x_pts[i] - x_pts[i-1]
    d_max[i] = min(d_max[i], d_max[i-1] + max_rate * dx)
    d_min[i] = max(d_min[i], d_min[i-1] - max_rate * dx)

for i in range(len(d_min)-2, -1, -1):
    dx = x_pts[i+1] - x_pts[i]
    d_max[i] = min(d_max[i], d_max[i+1] + max_rate * dx)
    d_min[i] = max(d_min[i], d_min[i+1] - max_rate * dx)

# 计算 Cartesian 边界
left_x = x_pts - d_max * np.sin(theta_pts)
left_y = y_pts + d_max * np.cos(theta_pts)
right_x = x_pts - d_min * np.sin(theta_pts)
right_y = y_pts + d_min * np.cos(theta_pts)

# 绘图
fig, ax = plt.subplots(figsize=(14, 7))

# 障碍物
obs_patch = patches.Rectangle((3.0, -0.6), 4.0, 0.75, color='crimson', alpha=0.7, hatch='//', label="Obstacle Blocking Reference Plan (y in [-0.6, 0.15])")
ax.add_patch(obs_patch)

# 安全走廊
poly_x = np.concatenate([left_x, right_x[::-1]])
poly_y = np.concatenate([left_y, right_y[::-1]])
ax.fill(poly_x, poly_y, color='deepskyblue', alpha=0.3, label="Unconstrained Safe Corridor (Shifted & Compressed to Left)")
ax.plot(left_x, left_y, color='forestgreen', linewidth=2.5, label="Corridor Left Boundary (d_max)")
ax.plot(right_x, right_y, color='darkorange', linewidth=2.5, label="Corridor Right Boundary (d_min Shifted to > 0)")

# 原参考线 (被穿透)
ax.plot(x_pts, y_pts, 'k--', linewidth=2.0, label="Global Reference Plan (y=0, Blocked by Obstacle)")

# 走廊中心引导线
center_y = 0.5 * (left_y + right_y)
ax.plot(x_pts, center_y, 'b-', linewidth=3.0, label="Corridor Centerline (Smooth Collision-Free Passage)")

ax.set_title("Unconstrained Safe Corridor: Seamless Shift & Compression when Reference Path is Blocked", fontsize=13, fontweight='bold')
ax.set_xlabel("X (m)", fontsize=11)
ax.set_ylabel("Y (m)", fontsize=11)
ax.set_xlim(0.0, 10.0)
ax.set_ylim(-1.0, 1.2)
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper left', fontsize=10, framealpha=0.9)

plt.tight_layout()
plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/unconstrained_corridor_simulation.png", dpi=300)
print("Saved unconstrained corridor simulation figure.")
