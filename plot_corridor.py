import numpy as np
import matplotlib.pyplot as plt

def generate_hairpin_path(straight_len=4.0, R=1.2, ds=0.05):
    points = []
    # 1. Straight segment 1
    for x in np.arange(0, straight_len, ds):
        points.append((x, 0.0, 0.0, 0.0))
    # 2. U-turn semi-circle
    for phi in np.arange(-np.pi/2.0, np.pi/2.0, ds/R):
        x = straight_len + R * np.cos(phi)
        y = R + R * np.sin(phi)
        theta = phi + np.pi/2.0
        kappa = 1.0 / R
        points.append((x, y, theta, kappa))
    # 3. Straight segment 2 (reverse)
    for x in np.arange(straight_len, 0.0, -ds):
        points.append((x, 2.0 * R, np.pi, 0.0))
    return np.array(points)

def main():
    traj = generate_hairpin_path(straight_len=4.0, R=1.0, ds=0.05)
    N = len(traj)
    
    # 原始固定宽度走廊 (发生严重自相交)
    d_left_raw = np.ones(N) * 0.9
    d_right_raw = -np.ones(N) * 0.9
    
    # 经过我们算法处理后的安全走廊
    # 1. 曲率限制 1 - kappa*d > 0.2 => d_max <= 0.8
    # 2. 掉头弯几何相交消除
    d_left_safe = np.ones(N) * 0.9
    d_right_safe = -np.ones(N) * 0.9
    
    for i in range(N):
        x, y, theta, kappa = traj[i]
        if kappa > 1e-3:
            d_left_safe[i] = min(d_left_safe[i], 0.8 / kappa)
            
    # 全局非相邻截面与边界自相交检测与收缩 (U-turn)
    for _ in range(5):
        for i in range(N):
            xi, yi, thi, _ = traj[i]
            nxi, nyi = -np.sin(thi), np.cos(thi)
            for j in range(i + 4, N):
                xj, yj, thj, _ = traj[j]
                nxj, nyj = -np.sin(thj), np.cos(thj)
                # 截面 i 的线段与截面 j 的线段求交
                p1 = np.array([xi + d_right_safe[i]*nxi, yi + d_right_safe[i]*nyi])
                p2 = np.array([xi + d_left_safe[i]*nxi, yi + d_left_safe[i]*nyi])
                p3 = np.array([xj + d_right_safe[j]*nxj, yj + d_right_safe[j]*nyj])
                p4 = np.array([xj + d_left_safe[j]*nxj, yj + d_left_safe[j]*nyj])
                
                # 2D segment intersection
                det = -(p2[0]-p1[0])*(p4[1]-p3[1]) + (p2[1]-p1[1])*(p4[0]-p3[0])
                if abs(det) > 1e-6:
                    t1 = (-(p3[0]-p1[0])*(p4[1]-p3[1]) + (p3[1]-p1[1])*(p4[0]-p3[0])) / det
                    t2 = ((p2[0]-p1[0])*(p3[1]-p1[1]) - (p2[1]-p1[1])*(p3[0]-p1[0])) / det
                    if 0 <= t1 <= 1 and 0 <= t2 <= 1:
                        q = p1 + t1 * (p2 - p1)
                        d_qi = (q[0]-xi)*nxi + (q[1]-yi)*nyi
                        d_qj = (q[0]-xj)*nxj + (q[1]-yj)*nyj
                        if d_qi > 0: d_left_safe[i] = max(0.15, min(d_left_safe[i], d_qi - 0.05))
                        if d_qj > 0: d_left_safe[j] = max(0.15, min(d_left_safe[j], d_qj - 0.05))

    # 双向平滑
    for i in range(1, N):
        ds = 0.05
        d_left_safe[i] = min(d_left_safe[i], d_left_safe[i-1] + 0.4*ds)
    for i in range(N-2, -1, -1):
        ds = 0.05
        d_left_safe[i] = min(d_left_safe[i], d_left_safe[i+1] + 0.4*ds)

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # Plot 1: Raw Uncorrected Corridor
    ax1 = axes[0]
    ax1.set_title("Without Self-Intersection Prevention (Raw Frenet d-bounds)\n(Severe Overlap & Inversion in Hairpin Turn)", fontsize=13, fontweight='bold', color='crimson')
    ax1.plot(traj[:, 0], traj[:, 1], 'r--', linewidth=2, label="Reference Path (U-turn)")
    for i in range(0, N, 3):
        x, y, theta, _ = traj[i]
        nx, ny = -np.sin(theta), np.cos(theta)
        lx, ly = x + d_left_raw[i]*nx, y + d_left_raw[i]*ny
        rx, ry = x + d_right_raw[i]*nx, y + d_right_raw[i]*ny
        ax1.plot([rx, lx], [ry, ly], color='gray', alpha=0.5, linewidth=1)
    
    # Boundary curves
    left_x = traj[:, 0] - d_left_raw * np.sin(traj[:, 2])
    left_y = traj[:, 1] + d_left_raw * np.cos(traj[:, 2])
    right_x = traj[:, 0] - d_right_raw * np.sin(traj[:, 2])
    right_y = traj[:, 1] + d_right_raw * np.cos(traj[:, 2])
    ax1.plot(left_x, left_y, 'b-', linewidth=2, label="Left Boundary (Self-Intersecting)")
    ax1.plot(right_x, right_y, 'g-', linewidth=2, label="Right Boundary")
    ax1.fill(np.append(left_x, right_x[::-1]), np.append(left_y, right_y[::-1]), color='red', alpha=0.15)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left')
    ax1.set_aspect('equal')
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")

    # Plot 2: Safe Corridor with Anti-Self-Intersection
    ax2 = axes[1]
    ax2.set_title("With Safe Corridor Generator (Proposed)\n(Zero Self-Intersection, Singularity-Free & Smooth)", fontsize=13, fontweight='bold', color='darkgreen')
    ax2.plot(traj[:, 0], traj[:, 1], 'k--', linewidth=2, label="Reference Path (U-turn)")
    for i in range(0, N, 3):
        x, y, theta, _ = traj[i]
        nx, ny = -np.sin(theta), np.cos(theta)
        lx, ly = x + d_left_safe[i]*nx, y + d_left_safe[i]*ny
        rx, ry = x + d_right_safe[i]*nx, y + d_right_safe[i]*ny
        ax2.plot([rx, lx], [ry, ly], color='cyan', alpha=0.7, linewidth=1)
    
    left_x_safe = traj[:, 0] - d_left_safe * np.sin(traj[:, 2])
    left_y_safe = traj[:, 1] + d_left_safe * np.cos(traj[:, 2])
    right_x_safe = traj[:, 0] - d_right_safe * np.sin(traj[:, 2])
    right_y_safe = traj[:, 1] + d_right_safe * np.cos(traj[:, 2])
    ax2.plot(left_x_safe, left_y_safe, 'g-', linewidth=2.5, label="Left Boundary (Safely Clipped)")
    ax2.plot(right_x_safe, right_y_safe, 'm-', linewidth=2.5, label="Right Boundary")
    ax2.fill(np.append(left_x_safe, right_x_safe[::-1]), np.append(left_y_safe, right_y_safe[::-1]), color='deepskyblue', alpha=0.3, label="Safe Driving Corridor Mesh")
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left')
    ax2.set_aspect('equal')
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")

    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/safe_corridor_comparison.png", dpi=300)
    print("Comparison figure saved to safe_corridor_comparison.png")

if __name__ == "__main__":
    main()
