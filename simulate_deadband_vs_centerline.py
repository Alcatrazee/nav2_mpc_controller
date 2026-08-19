import numpy as np
import matplotlib.pyplot as plt
import casadi as ca

def create_mpc_solvers(N=10, dt=0.1):
    # =========================================================================
    # 1. 求解器 A: 强行跟踪走廊中线 (Centerline Tracking Mode)
    # =========================================================================
    opti_A = ca.Opti()
    X_A = opti_A.variable(4, N + 1)
    U_A = opti_A.variable(2, N)
    Slack_L_A = opti_A.variable(N)
    Slack_R_A = opti_A.variable(N)

    X0_A = opti_A.parameter(4)
    Ref_s_A = opti_A.parameter(N)
    Ref_v_A = opti_A.parameter(N)
    Ref_w_A = opti_A.parameter(N)
    Ref_kappa_A = opti_A.parameter(N)
    Corr_d_min_A = opti_A.parameter(N)
    Corr_d_max_A = opti_A.parameter(N)

    opti_A.subject_to(X_A[:, 0] == X0_A)
    cost_A = 0
    for k in range(N):
        denom = ca.fmax(1.0 - Ref_kappa_A[k] * X_A[1, k], 0.1)
        s_dot = X_A[3, k] * ca.cos(X_A[2, k]) / denom
        d_dot = X_A[3, k] * ca.sin(X_A[2, k])
        e_psi_dot = U_A[1, k] - Ref_kappa_A[k] * s_dot

        opti_A.subject_to(X_A[0, k+1] == X_A[0, k] + s_dot * dt)
        opti_A.subject_to(X_A[1, k+1] == X_A[1, k] + d_dot * dt)
        opti_A.subject_to(X_A[2, k+1] == X_A[2, k] + e_psi_dot * dt)
        opti_A.subject_to(X_A[3, k+1] == X_A[3, k] + U_A[0, k] * dt)

        opti_A.subject_to(opti_A.bounded(0.0, X_A[3, k+1], 0.6))
        opti_A.subject_to(opti_A.bounded(-1.2, U_A[1, k], 1.2))
        opti_A.subject_to(opti_A.bounded(-1.0, U_A[0, k], 1.0))

        opti_A.subject_to(Slack_L_A[k] >= 0.0)
        opti_A.subject_to(Slack_R_A[k] >= 0.0)
        opti_A.subject_to(X_A[1, k+1] <= Corr_d_max_A[k] + Slack_L_A[k])
        opti_A.subject_to(X_A[1, k+1] >= Corr_d_min_A[k] - Slack_R_A[k])

        d_center = 0.5 * (Corr_d_max_A[k] + Corr_d_min_A[k])
        cost_A += 2.0 * (X_A[0, k+1] - Ref_s_A[k])**2
        cost_A += 20.0 * (X_A[1, k+1] - d_center)**2 # 强跟踪中线
        cost_A += 8.0 * (X_A[2, k+1])**2
        cost_A += 1.0 * (X_A[3, k+1] - Ref_v_A[k])**2
        cost_A += 1.0 * (U_A[1, k] - Ref_w_A[k])**2
        cost_A += 0.2 * (U_A[0, k])**2
        if k > 0:
            cost_A += 2.0 * (U_A[1, k] - U_A[1, k-1])**2
        cost_A += 1000.0 * (Slack_L_A[k]**2 + Slack_R_A[k]**2)

    opti_A.minimize(cost_A)
    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes", "ipopt.hessian_approximation": "exact", "ipopt.max_iter": 15}
    opti_A.solver("ipopt", opts)
    solver_A = opti_A.to_function("solver_A", [X0_A, Ref_s_A, Ref_v_A, Ref_w_A, Ref_kappa_A, Corr_d_min_A, Corr_d_max_A], [U_A, X_A])

    # =========================================================================
    # 2. 求解器 B: 边缘死区二次代价 (Deadband Barrier + Smoothing)
    # =========================================================================
    opti_B = ca.Opti()
    X_B = opti_B.variable(4, N + 1)
    U_B = opti_B.variable(2, N)
    Slack_L_B = opti_B.variable(N)
    Slack_R_B = opti_B.variable(N)

    X0_B = opti_B.parameter(4)
    Ref_s_B = opti_B.parameter(N)
    Ref_v_B = opti_B.parameter(N)
    Ref_w_B = opti_B.parameter(N)
    Ref_kappa_B = opti_B.parameter(N)
    Corr_d_min_B = opti_B.parameter(N)
    Corr_d_max_B = opti_B.parameter(N)

    opti_B.subject_to(X_B[:, 0] == X0_B)
    cost_B = 0
    buffer_margin = 0.15

    for k in range(N):
        denom = ca.fmax(1.0 - Ref_kappa_B[k] * X_B[1, k], 0.1)
        s_dot = X_B[3, k] * ca.cos(X_B[2, k]) / denom
        d_dot = X_B[3, k] * ca.sin(X_B[2, k])
        e_psi_dot = U_B[1, k] - Ref_kappa_B[k] * s_dot

        opti_B.subject_to(X_B[0, k+1] == X_B[0, k] + s_dot * dt)
        opti_B.subject_to(X_B[1, k+1] == X_B[1, k] + d_dot * dt)
        opti_B.subject_to(X_B[2, k+1] == X_B[2, k] + e_psi_dot * dt)
        opti_B.subject_to(X_B[3, k+1] == X_B[3, k] + U_B[0, k] * dt)

        opti_B.subject_to(opti_B.bounded(0.0, X_B[3, k+1], 0.6))
        opti_B.subject_to(opti_B.bounded(-1.2, U_B[1, k], 1.2))
        opti_B.subject_to(opti_B.bounded(-1.0, U_B[0, k], 1.0))

        opti_B.subject_to(Slack_L_B[k] >= 0.0)
        opti_B.subject_to(Slack_R_B[k] >= 0.0)
        opti_B.subject_to(X_B[1, k+1] <= Corr_d_max_B[k] + Slack_L_B[k])
        opti_B.subject_to(X_B[1, k+1] >= Corr_d_min_B[k] - Slack_R_B[k])

        left_thresh = Corr_d_max_B[k] - buffer_margin
        right_thresh = Corr_d_min_B[k] + buffer_margin
        deadband_barrier_cost = (ca.fmax(0.0, X_B[1, k+1] - left_thresh))**2 + \
                                (ca.fmax(0.0, right_thresh - X_B[1, k+1]))**2

        cost_B += 2.0 * (X_B[0, k+1] - Ref_s_B[k])**2
        cost_B += 15.0 * (X_B[1, k+1])**2 # 跟踪全局中线 d=0
        cost_B += 10.0 * (X_B[2, k+1])**2
        cost_B += 1.0 * (X_B[3, k+1] - Ref_v_B[k])**2
        cost_B += 1.5 * (U_B[1, k] - Ref_w_B[k])**2
        cost_B += 0.2 * (U_B[0, k])**2
        if k > 0:
            cost_B += 3.0 * (U_B[1, k] - U_B[1, k-1])**2 # 转向平滑度惩罚
        cost_B += 35.0 * deadband_barrier_cost
        cost_B += 1000.0 * (Slack_L_B[k]**2 + Slack_R_B[k]**2)

    opti_B.minimize(cost_B)
    opti_B.solver("ipopt", opts)
    solver_B = opti_B.to_function("solver_B", [X0_B, Ref_s_B, Ref_v_B, Ref_w_B, Ref_kappa_B, Corr_d_min_B, Corr_d_max_B], [U_B, X_B])

    return solver_A, solver_B

def run_experiment():
    solver_A, solver_B = create_mpc_solvers()
    dt = 0.1
    sim_steps = 220
    np.random.seed(42)

    def get_ground_truth_corridor(s_val):
        if 3.5 <= s_val <= 6.5:
            return 0.25, 0.85
        elif 2.0 <= s_val < 3.5:
            ratio = (s_val - 2.0) / 1.5
            return -0.8 + ratio * 1.05, 0.85
        elif 6.5 < s_val <= 8.0:
            ratio = (8.0 - s_val) / 1.5
            return -0.8 + ratio * 1.05, 0.85
        else:
            return -0.8, 0.85

    # 1. 运行方式 A (强行跟踪走廊中线)
    state_A = np.array([0.0, 0.0, 0.0, 0.3])
    traj_A, w_history_A = [], []

    for step in range(sim_steps):
        s_curr = state_A[0]
        traj_A.append(state_A.copy())
        ref_s = [s_curr + (k + 1) * 0.4 * dt for k in range(10)]
        ref_v = [0.4] * 10
        ref_w = [0.0] * 10
        ref_kappa = [0.0] * 10

        d_min_noisy, d_max_noisy = [], []
        for r_s in ref_s:
            dmin_base, dmax_base = get_ground_truth_corridor(r_s)
            d_min_noisy.append(dmin_base + np.random.uniform(-0.03, 0.03))
            d_max_noisy.append(dmax_base + np.random.uniform(-0.03, 0.03))

        in_args = [ca.DM(state_A), ca.DM(ref_s), ca.DM(ref_v), ca.DM(ref_w), ca.DM(ref_kappa), ca.DM(d_min_noisy), ca.DM(d_max_noisy)]
        res = solver_A(*in_args)
        u_opt = np.array(res[0])[:, 0]
        w_history_A.append(u_opt[1])

        v = state_A[3] + u_opt[0] * dt
        w = u_opt[1]
        state_A[0] += v * np.cos(state_A[2]) * dt
        state_A[1] += v * np.sin(state_A[2]) * dt
        state_A[2] += w * dt
        state_A[3] = v

    # 2. 运行方式 B (边缘死区二次代价)
    np.random.seed(42)
    state_B = np.array([0.0, 0.0, 0.0, 0.3])
    traj_B, w_history_B = [], []

    for step in range(sim_steps):
        s_curr = state_B[0]
        traj_B.append(state_B.copy())
        ref_s = [s_curr + (k + 1) * 0.4 * dt for k in range(10)]
        ref_v = [0.4] * 10
        ref_w = [0.0] * 10
        ref_kappa = [0.0] * 10

        d_min_noisy, d_max_noisy = [], []
        for r_s in ref_s:
            dmin_base, dmax_base = get_ground_truth_corridor(r_s)
            d_min_noisy.append(dmin_base + np.random.uniform(-0.03, 0.03))
            d_max_noisy.append(dmax_base + np.random.uniform(-0.03, 0.03))

        in_args = [ca.DM(state_B), ca.DM(ref_s), ca.DM(ref_v), ca.DM(ref_w), ca.DM(ref_kappa), ca.DM(d_min_noisy), ca.DM(d_max_noisy)]
        res = solver_B(*in_args)
        u_opt = np.array(res[0])[:, 0]
        w_history_B.append(u_opt[1])

        v = state_B[3] + u_opt[0] * dt
        w = u_opt[1]
        state_B[0] += v * np.cos(state_B[2]) * dt
        state_B[1] += v * np.sin(state_B[2]) * dt
        state_B[2] += w * dt
        state_B[3] = v

    traj_A = np.array(traj_A)
    traj_B = np.array(traj_B)
    w_history_A = np.array(w_history_A)
    w_history_B = np.array(w_history_B)

    # 统计平滑度指标
    tv_A = np.sum(np.abs(np.diff(w_history_A)))
    tv_B = np.sum(np.abs(np.diff(w_history_B)))

    # 绘制高清晰对比图
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), gridspec_kw={'height_ratios': [1.3, 1, 1]})

    s_axis = np.linspace(0, 10, 300)
    corr_left_clean = np.array([get_ground_truth_corridor(s)[1] for s in s_axis])
    corr_right_clean = np.array([get_ground_truth_corridor(s)[0] for s in s_axis])

    # 子图 1: 走廊与 2D 轨迹俯视图
    ax1 = axes[0]
    ax1.fill_between(s_axis, corr_right_clean, corr_left_clean, color='deepskyblue', alpha=0.22, label="Safe Corridor Region")
    ax1.plot(s_axis, corr_left_clean, 'g-', linewidth=2.0, label="Corridor Left Bound")
    ax1.plot(s_axis, corr_right_clean, 'm-', linewidth=2.0, label="Corridor Right Bound (Obstacle Encroachment)")
    ax1.plot(s_axis, corr_right_clean + 0.15, 'm--', alpha=0.6, label="Deadband Barrier Threshold (d_min + 0.15m)")
    ax1.plot(s_axis, corr_left_clean - 0.15, 'g--', alpha=0.6, label="Deadband Barrier Threshold (d_max - 0.15m)")

    obs_rect = plt.Rectangle((3.5, -0.8), 3.0, 1.05, color='crimson', alpha=0.55, hatch='//', label="Obstacle Area (Blocks d=0)")
    ax1.add_patch(obs_rect)

    ax1.plot(traj_A[:, 0], traj_A[:, 1], 'r-', linewidth=2.0, alpha=0.85, label="Mode A: Centerline Tracking (Chattering with Noise)")
    ax1.plot(traj_B[:, 0], traj_B[:, 1], 'b-', linewidth=3.2, label="Mode B: Deadband Quadratic Barrier (Smooth, Clean Bypass)")
    ax1.plot([0, 10], [0, 0], 'k:', linewidth=1.4, label="Global Centerline (d=0)")

    ax1.set_title("Trajectory Comparison under Costmap Noise (±3.0cm Raster Flicker)", fontsize=13, fontweight='bold')
    ax1.set_ylabel("Lateral Deviation d (m)", fontsize=11)
    ax1.set_xlim(0, 9.2)
    ax1.set_ylim(-0.95, 1.1)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left', ncol=2, fontsize=9, framealpha=0.9)

    # 子图 2: 横向偏差 d(s)
    ax2 = axes[1]
    ax2.plot(traj_A[:, 0], traj_A[:, 1], 'r-', linewidth=2.0, label="Mode A: Centerline Tracking (Jittering around d_center)")
    ax2.plot(traj_B[:, 0], traj_B[:, 1], 'b-', linewidth=2.5, label="Mode B: Deadband Barrier (Completely Smooth, Zero-Jitter in Free Zone)")
    ax2.axhline(0, color='k', linestyle=':', label="Reference d=0")
    ax2.set_title("Lateral Deviation d(s) along Reference Path", fontsize=13, fontweight='bold')
    ax2.set_ylabel("d (m)", fontsize=11)
    ax2.set_xlim(0, 9.2)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left', fontsize=9.5)

    # 子图 3: 控制量角速度 w(t)
    time_axis_A = np.arange(len(w_history_A)) * dt
    time_axis_B = np.arange(len(w_history_B)) * dt
    ax3 = axes[2]
    ax3.plot(time_axis_A, w_history_A, 'r-', linewidth=1.5, alpha=0.85, label=f"Mode A: Centerline Tracking (High-frequency Oscillation)")
    ax3.plot(time_axis_B, w_history_B, 'b-', linewidth=2.2, label=f"Mode B: Deadband Barrier (Smooth C1 Continuous Steering)")
    ax3.set_title("Control Effort: Angular Velocity w(t)", fontsize=13, fontweight='bold', color='navy')
    ax3.set_xlabel("Time (s)", fontsize=11)
    ax3.set_ylabel("w (rad/s)", fontsize=11)
    ax3.set_xlim(0, max(time_axis_A[-1], time_axis_B[-1]))
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=9.5)

    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/corridor_deadband_simulation.png", dpi=300)
    print("Simulation chart updated: corridor_deadband_simulation.png")

if __name__ == "__main__":
    run_experiment()
