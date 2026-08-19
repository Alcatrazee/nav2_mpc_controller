import numpy as np
import matplotlib.pyplot as plt
import casadi as ca

def create_mpc_solver(N=10, dt=0.1):
    opti = ca.Opti()
    X = opti.variable(4, N + 1)
    U = opti.variable(2, N)
    Slack_L = opti.variable(N)
    Slack_R = opti.variable(N)

    X0 = opti.parameter(4)
    Ref_s = opti.parameter(N)
    Ref_v = opti.parameter(N)
    Ref_w = opti.parameter(N)
    Ref_kappa = opti.parameter(N)
    Corr_d_min = opti.parameter(N)
    Corr_d_max = opti.parameter(N)

    opti.subject_to(X[:, 0] == X0)
    cost = 0

    for k in range(N):
        denom = ca.fmax(1.0 - Ref_kappa[k] * X[1, k], 0.1)
        s_dot = X[3, k] * ca.cos(X[2, k]) / denom
        d_dot = X[3, k] * ca.sin(X[2, k])
        e_psi_dot = U[1, k] - Ref_kappa[k] * s_dot

        opti.subject_to(X[0, k+1] == X[0, k] + s_dot * dt)
        opti.subject_to(X[1, k+1] == X[1, k] + d_dot * dt)
        opti.subject_to(X[2, k+1] == X[2, k] + e_psi_dot * dt)
        opti.subject_to(X[3, k+1] == X[3, k] + U[0, k] * dt)

        opti.subject_to(opti.bounded(0.0, X[3, k+1], 0.6))
        opti.subject_to(opti.bounded(-1.2, U[1, k], 1.2))
        opti.subject_to(opti.bounded(-1.0, U[0, k], 1.0))

        opti.subject_to(Slack_L[k] >= 0.0)
        opti.subject_to(Slack_R[k] >= 0.0)
        opti.subject_to(X[1, k+1] <= Corr_d_max[k] + Slack_L[k])
        opti.subject_to(X[1, k+1] >= Corr_d_min[k] - Slack_R[k])

        d_center = 0.5 * (Corr_d_max[k] + Corr_d_min[k])
        left_dist = Corr_d_max[k] - 0.1
        right_dist = Corr_d_min[k] + 0.1
        barrier_cost = (ca.fmax(0.0, X[1, k+1] - left_dist))**2 + \
                       (ca.fmax(0.0, right_dist - X[1, k+1]))**2

        cost += 2.0 * (X[0, k+1] - Ref_s[k])**2
        cost += 25.0 * (X[1, k+1] - d_center)**2
        cost += 8.0 * (X[2, k+1])**2
        cost += 1.0 * (X[3, k+1] - Ref_v[k])**2
        cost += 0.5 * (U[1, k] - Ref_w[k])**2
        cost += 0.2 * (U[0, k])**2
        cost += 40.0 * barrier_cost
        cost += 1000.0 * (Slack_L[k]**2 + Slack_R[k]**2)

    opti.minimize(cost)
    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes", "ipopt.hessian_approximation": "exact", "ipopt.max_iter": 15}
    opti.solver("ipopt", opts)

    return opti.to_function("mpc", [X0, Ref_s, Ref_v, Ref_w, Ref_kappa, Corr_d_min, Corr_d_max], [U, X])

def run_simulation():
    mpc_fn = create_mpc_solver()
    dt = 0.1
    sim_steps = 220
    
    # 模拟场景：直线道路上 (s 从 0 到 10m)，在 s in [3m, 7m] 处右侧有障碍物
    # 导致安全走廊右边界在 [3m, 7m] 从 -0.8m 收缩到 +0.25m (阻挡了原中心线 d=0!)
    def get_corridor(s_val):
        if 3.0 <= s_val <= 7.0:
            return 0.25, 0.85 # 右边界收缩到 0.25m (迫使机器人向左避障)
        elif 2.0 <= s_val < 3.0:
            ratio = (s_val - 2.0) / 1.0
            return -0.8 + ratio * 1.05, 0.85
        elif 7.0 < s_val <= 8.0:
            ratio = (8.0 - s_val) / 1.0
            return -0.8 + ratio * 1.05, 0.85
        else:
            return -0.8, 0.8

    # 1. 纯跟线模式 (enable_safe_corridor = False)
    state_pure = np.array([0.0, 0.0, 0.0, 0.3])
    traj_pure = []
    
    for step in range(sim_steps):
        s_curr = state_pure[0]
        traj_pure.append(state_pure.copy())
        ref_s = [s_curr + (k + 1) * 0.4 * dt for k in range(10)]
        ref_v = [0.4] * 10
        ref_w = [0.0] * 10
        ref_kappa = [0.0] * 10
        d_min_wide = [-5.0] * 10
        d_max_wide = [5.0] * 10
        
        in_args = [ca.DM(state_pure), ca.DM(ref_s), ca.DM(ref_v), ca.DM(ref_w), ca.DM(ref_kappa), ca.DM(d_min_wide), ca.DM(d_max_wide)]
        res = mpc_fn(*in_args)
        u_opt = np.array(res[0])[:, 0]
        
        v = state_pure[3] + u_opt[0] * dt
        w = u_opt[1]
        state_pure[0] += v * np.cos(state_pure[2]) * dt
        state_pure[1] += v * np.sin(state_pure[2]) * dt
        state_pure[2] += w * dt
        state_pure[3] = v

    # 2. 安全走廊绕障模式 (enable_safe_corridor = True)
    state_avoid = np.array([0.0, 0.0, 0.0, 0.3])
    traj_avoid = []
    
    for step in range(sim_steps):
        s_curr = state_avoid[0]
        traj_avoid.append(state_avoid.copy())
        ref_s = [s_curr + (k + 1) * 0.4 * dt for k in range(10)]
        ref_v = [0.4] * 10
        ref_w = [0.0] * 10
        ref_kappa = [0.0] * 10
        
        d_min_corr = []
        d_max_corr = []
        for r_s in ref_s:
            dmin, dmax = get_corridor(r_s)
            d_min_corr.append(dmin)
            d_max_corr.append(dmax)
            
        in_args = [ca.DM(state_avoid), ca.DM(ref_s), ca.DM(ref_v), ca.DM(ref_w), ca.DM(ref_kappa), ca.DM(d_min_corr), ca.DM(d_max_corr)]
        res = mpc_fn(*in_args)
        u_opt = np.array(res[0])[:, 0]
        
        v = state_avoid[3] + u_opt[0] * dt
        w = u_opt[1]
        state_avoid[0] += v * np.cos(state_avoid[2]) * dt
        state_avoid[1] += v * np.sin(state_avoid[2]) * dt
        state_avoid[2] += w * dt
        state_avoid[3] = v

    traj_pure = np.array(traj_pure)
    traj_avoid = np.array(traj_avoid)

    # 绘图对比
    s_axis = np.linspace(0, 10, 200)
    corr_left = [get_corridor(s)[1] for s in s_axis]
    corr_right = [get_corridor(s)[0] for s in s_axis]

    fig, ax = plt.subplots(figsize=(13, 6))
    ax.fill_between(s_axis, corr_right, corr_left, color='deepskyblue', alpha=0.25, label="Safe Driving Corridor [d_min(s), d_max(s)]")
    ax.plot(s_axis, corr_left, 'g-', linewidth=2, label="Corridor Left Boundary")
    ax.plot(s_axis, corr_right, 'm-', linewidth=2, label="Corridor Right Boundary (Obstacle Encroachment)")

    # 障碍物方块展示
    obs_rect = plt.Rectangle((3.0, -0.8), 4.0, 1.05, color='crimson', alpha=0.6, hatch='//', label="Obstacle Region (Blocks Reference Line d=0)")
    ax.add_patch(obs_rect)

    # 绘制纯跟线轨迹 vs 安全走廊避障轨迹
    ax.plot(traj_pure[:, 0], traj_pure[:, 1], 'r--', linewidth=2.5, label="Pure Tracking Mode (enable_safe_corridor = False, Sticking to d=0 -> Collides)")
    ax.plot(traj_avoid[:, 0], traj_avoid[:, 1], 'b-', linewidth=3.0, label="Safe Corridor Mode (enable_safe_corridor = True, Smooth Bypass in Corridor)")

    # 参考中心线
    ax.plot([0, 10], [0, 0], 'k:', linewidth=1.5, label="Reference Center Line (d=0)")

    ax.set_title("MPC Controller: Pure Path Tracking vs. Safe Corridor Obstacle Avoidance", fontsize=14, fontweight='bold')
    ax.set_xlabel("Longitudinal Arc Length s (m)", fontsize=12)
    ax.set_ylabel("Lateral Deviation d (m)", fontsize=12)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left', framealpha=0.9)
    ax.set_ylim(-1.0, 1.2)
    ax.set_xlim(0, 9.5)

    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/mpc_corridor_tracking_vs_avoidance.png", dpi=300)
    print("Simulation plot saved to mpc_corridor_tracking_vs_avoidance.png")

if __name__ == "__main__":
    run_simulation()
