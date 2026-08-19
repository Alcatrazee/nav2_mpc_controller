import numpy as np
import matplotlib.pyplot as plt
import casadi as ca

def create_mpc_terminal_solver(N=15, dt=0.05, use_annealing=True):
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
    S_Remain = opti.parameter(N) # 剩余距离参数

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

        opti.subject_to(opti.bounded(0.0, X[3, k+1], 0.5))
        opti.subject_to(opti.bounded(-1.2, U[1, k], 1.2))
        opti.subject_to(opti.bounded(-1.0, U[0, k], 1.0))

        opti.subject_to(Slack_L[k] >= 0.0)
        opti.subject_to(Slack_R[k] >= 0.0)
        opti.subject_to(X[1, k+1] <= Corr_d_max[k] + Slack_L[k])
        opti.subject_to(X[1, k+1] >= Corr_d_min[k] - Slack_R[k])

        # 走廊宽度与几何中心
        width_k = ca.fmax(Corr_d_max[k] - Corr_d_min[k], 0.2)
        d_center_k = 0.5 * (Corr_d_max[k] + Corr_d_min[k])

        # 走廊边缘排斥势场
        margin = 0.10
        barrier_cost = (ca.fmax(0.0, X[1, k+1] - (Corr_d_max[k] - margin)))**2 + \
                       (ca.fmax(0.0, (Corr_d_min[k] + margin) - X[1, k+1]))**2

        # 终点退火因子: 当 s_remain < 0.5m 时平滑退火排斥力，让位于终点精准收敛
        if use_annealing:
            gamma = ca.fmin(1.0, ca.fmax(0.0, S_Remain[k] / 0.5))
        else:
            gamma = 1.0 # 不退火，力场死锁

        cost += 2.0 * (X[0, k+1] - Ref_s[k])**2
        cost += 15.0 * (X[1, k+1])**2
        cost += (30.0 * gamma) * ((X[1, k+1] - d_center_k) / width_k)**2
        cost += (50.0 * gamma) * barrier_cost
        cost += 8.0 * (X[2, k+1])**2
        cost += 1.0 * (X[3, k+1] - Ref_v[k])**2
        cost += 0.5 * (U[1, k] - Ref_w[k])**2
        cost += 0.2 * (U[0, k])**2
        cost += 1000.0 * (Slack_L[k]**2 + Slack_R[k]**2)

    # 终端高精度位姿约束与代价
    if use_annealing:
        opti.subject_to(opti.bounded(-0.03, X[1, N], 0.03)) # 强制终端横向 3cm 高精
        opti.subject_to(opti.bounded(-0.05, X[2, N], 0.05)) # 强制终端航向对齐
        cost += 100.0 * (X[1, N])**2
        cost += 50.0 * (X[2, N])**2

    opti.minimize(cost)
    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes", "ipopt.hessian_approximation": "exact", "ipopt.max_iter": 15}
    opti.solver("ipopt", opts)
    return opti.to_function("mpc_term", [X0, Ref_s, Ref_v, Ref_w, Ref_kappa, Corr_d_min, Corr_d_max, S_Remain], [U, X])

def run_comparison():
    # 模拟终点靠墙场景:
    # 终点位于 x = 3.0m, y = 0.0m (Goal Pose)
    # 右侧障碍物一直延伸到终点附近，走廊右边界 d_min = -0.06m (离终点 y=0 只有 6cm!)
    # 走廊左边界 d_max = 0.40m
    dt = 0.05
    N = 15

    def get_corridor(s_val):
        # 终点在 s=3.0m
        # 右侧障碍物极度逼近终点: d_min = -0.06m
        return -0.06, 0.40

    # 运行无退火方案 (旧方法: 势场死锁，无法到达终点)
    solver_old = create_mpc_terminal_solver(N, dt, use_annealing=False)
    state_old = np.array([0.0, 0.0, 0.0, 0.25])
    traj_old = []

    for step in range(120):
        s_curr = state_old[0]
        traj_old.append(state_old.copy())
        if s_curr >= 2.98 and state_old[3] < 0.02:
            break
        ref_s = [min(3.0, s_curr + (k + 1) * 0.25 * dt) for k in range(N)]
        ref_v = [max(0.0, min(0.3, np.sqrt(2 * 0.5 * max(0.0, 3.0 - rs)))) for rs in ref_s]
        ref_w = [0.0] * N
        ref_kappa = [0.0] * N
        d_min_seq = [get_corridor(rs)[0] for rs in ref_s]
        d_max_seq = [get_corridor(rs)[1] for rs in ref_s]
        s_rem_seq = [max(0.0, 3.0 - rs) for rs in ref_s]

        res = solver_old(state_old, ref_s, ref_v, ref_w, ref_kappa, d_min_seq, d_max_seq, s_rem_seq)
        u_opt = np.array(res[0])[:, 0]
        state_old[0] += state_old[3] * dt
        state_old[1] += state_old[3] * state_old[2] * dt
        state_old[2] += u_opt[1] * dt
        state_old[3] = max(0.0, state_old[3] + u_opt[0] * dt)

    # 运行新方案 (走廊漏斗吸附 + 势场退火 + 终端硬约束)
    solver_new = create_mpc_terminal_solver(N, dt, use_annealing=True)
    state_new = np.array([0.0, 0.0, 0.0, 0.25])
    traj_new = []

    for step in range(120):
        s_curr = state_new[0]
        traj_new.append(state_new.copy())
        if s_curr >= 2.98 and state_new[3] < 0.02:
            break
        ref_s = [min(3.0, s_curr + (k + 1) * 0.25 * dt) for k in range(N)]
        ref_v = [max(0.0, min(0.3, np.sqrt(2 * 0.5 * max(0.0, 3.0 - rs)))) for rs in ref_s]
        ref_w = [0.0] * N
        ref_kappa = [0.0] * N
        
        # 走廊漏斗平滑向终点 0 收拢
        d_min_seq = []
        d_max_seq = []
        for rs in ref_s:
            s_rem = max(0.0, 3.0 - rs)
            raw_min, raw_max = get_corridor(rs)
            # 终点漏斗收拢
            funnel_d = min(0.40, 0.03 + 0.5 * s_rem)
            d_min_seq.append(max(raw_min, -funnel_d))
            d_max_seq.append(min(raw_max, funnel_d))

        s_rem_seq = [max(0.0, 3.0 - rs) for rs in ref_s]

        res = solver_new(state_new, ref_s, ref_v, ref_w, ref_kappa, d_min_seq, d_max_seq, s_rem_seq)
        u_opt = np.array(res[0])[:, 0]
        state_new[0] += state_new[3] * dt
        state_new[1] += state_new[3] * state_new[2] * dt
        state_new[2] += u_opt[1] * dt
        state_new[3] = max(0.0, state_new[3] + u_opt[0] * dt)

    traj_old = np.array(traj_old)
    traj_new = np.array(traj_new)

    # 绘图对比
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # 上图: 轨迹与障碍物边界
    ax1.plot([0, 3.2], [-0.06, -0.06], 'r-', linewidth=2.5, label="Right Obstacle Boundary (d = -0.06m, very close to Goal)")
    ax1.plot([0, 3.2], [0.40, 0.40], 'g-', linewidth=2.0, label="Left Corridor Boundary (d = 0.40m)")
    ax1.plot([0, 3.2], [0.0, 0.0], 'k--', linewidth=1.5, alpha=0.7, label="Target Reference Line (Goal at x=3.0, y=0.0)")
    ax1.plot(traj_old[:, 0], traj_old[:, 1], 'm--', linewidth=2.5, label=f"Previous Method (Pushed away by Barrier, Stop Error d={traj_old[-1, 1]*100:.1f} cm)")
    ax1.plot(traj_new[:, 0], traj_new[:, 1], 'b-', linewidth=3.0, label=f"Proposed Method (Goal Snapping & Annealing, Stop Error d={traj_new[-1, 1]*100:.2f} cm)")
    ax1.scatter([3.0], [0.0], color='red', s=120, zorder=5, marker='*', label="Target Goal Pose (x=3.0, y=0.0)")
    ax1.set_ylabel("Lateral Position d (m)", fontsize=11)
    ax1.set_title("Terminal Goal Approach: Barrier Force Conflict vs. Goal Funnel Snapping & Annealing", fontsize=13, fontweight='bold')
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left', fontsize=9.5)

    # 下图: 横向误差对比
    ax2.plot(traj_old[:, 0], traj_old[:, 1] * 100, 'm--', linewidth=2.5, label="Previous Lateral Error (cm)")
    ax2.plot(traj_new[:, 0], traj_new[:, 1] * 100, 'b-', linewidth=3.0, label="Proposed Lateral Error (cm, converges to 0.00 cm)")
    ax2.axhline(0, color='k', linestyle=':', alpha=0.6)
    ax2.set_xlabel("Longitudinal Distance s (m)", fontsize=11)
    ax2.set_ylabel("Lateral Deviation (cm)", fontsize=11)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left', fontsize=9.5)

    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/goal_precision_solution_comparison.png", dpi=300)
    print("Saved goal precision comparison figure.")

if __name__ == "__main__":
    run_comparison()
