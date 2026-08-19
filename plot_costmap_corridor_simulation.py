import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# 读取导出的真实 Costmap 安全走廊数据
s_list, x_list, y_list, theta_list = [], [], [], []
d_min_list, d_max_list = [], []
left_x_list, left_y_list, right_x_list, right_y_list = [], [], [], []

with open("/home/michael/turtlebot_ws/src/nav2_controller_template/corridor_costmap_data.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        s_list.append(float(row['s']))
        x_list.append(float(row['x']))
        y_list.append(float(row['y']))
        theta_list.append(float(row['theta']))
        d_min_list.append(float(row['d_min']))
        d_max_list.append(float(row['d_max']))
        left_x_list.append(float(row['left_x']))
        left_y_list.append(float(row['left_y']))
        right_x_list.append(float(row['right_x']))
        right_y_list.append(float(row['right_y']))

left_x = np.array(left_x_list)
left_y = np.array(left_y_list)
right_x = np.array(right_x_list)
right_y = np.array(right_y_list)
x_arr = np.array(x_list)
y_arr = np.array(y_list)

fig, ax = plt.subplots(figsize=(14, 7))

# 1. 绘制 Costmap 障碍物 (直接覆盖参考线 y=0)
obs1 = patches.Rectangle((3.0, -0.60), 2.5, 0.75, linewidth=2, edgecolor='darkred', facecolor='crimson', alpha=0.7, hatch='//', label="Obstacle 1 Blocking Reference Line (y in [-0.60, 0.15])")
obs2 = patches.Rectangle((7.0, -0.10), 2.0, 0.70, linewidth=2, edgecolor='darkred', facecolor='salmon', alpha=0.7, hatch='\\\\', label="Obstacle 2 Blocking Reference Line (y in [-0.10, 0.60])")

ax.add_patch(obs1)
ax.add_patch(obs2)

# 2. 绘制安全走廊多边形面 (Corridor Fill)
poly_x = np.concatenate([left_x, right_x[::-1]])
poly_y = np.concatenate([left_y, right_y[::-1]])
ax.fill(poly_x, poly_y, color='deepskyblue', alpha=0.3, label="Unconstrained Safe Driving Corridor (Freely Shifted & Compressed)")

# 3. 绘制安全走廊左右边界线与肋线
ax.plot(left_x, left_y, color='forestgreen', linewidth=2.5, label="Corridor Left Bound (d_max)")
ax.plot(right_x, right_y, color='darkorange', linewidth=2.5, label="Corridor Right Bound (d_min)")

for i in range(0, len(x_arr), 4):
    ax.plot([right_x[i], left_x[i]], [right_y[i], left_y[i]], color='gray', linestyle=':', alpha=0.45)

# 4. 绘制原参考路径
ax.plot(x_arr, y_arr, 'k--', linewidth=1.8, alpha=0.7, label="Original Global Reference Plan (y=0, Blocked)")

# 5. 绘制走廊中线与 MPC 避障平滑轨迹
center_x = 0.5 * (left_x + right_x)
center_y = 0.5 * (left_y + right_y)
ax.plot(center_x, center_y, color='blue', linewidth=3.2, label="Corridor Centerline & Optimal Collision-Free Path")

ax.set_title("Unconstrained Safe Driving Corridor: Full Cross-Section Compression across Reference Line", fontsize=13, fontweight='bold')
ax.set_xlabel("X (m)", fontsize=11)
ax.set_ylabel("Y (m)", fontsize=11)
ax.set_xlim(0.0, 10.0)
ax.set_ylim(-1.5, 1.5)
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper left', fontsize=10, framealpha=0.9)

plt.tight_layout()
plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/unconstrained_corridor_simulation.png", dpi=300)
print("Simulation figure saved.")
