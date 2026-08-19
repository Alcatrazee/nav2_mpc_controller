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

        # 1. 窄通道自适应归一化居中势场 (宽度越窄，居中推力成平方倍放大!)
        centering_cost = ((X[1, k+1] - d_center_k) / width_k)**2

        # 2. 走廊边缘绝对防碰撞缓冲势场 (靠近边缘 0.1m 时的强非线性排斥)
        margin = 0.10
        barrier_cost = (ca.fmax(0.0, X[1, k+1] - (Corr_d_max[k] - margin)))**2 + \
                       (ca.fmax(0.0, (Corr_d_min[k] + margin) - X[1, k+1]))**2

        cost += 2.0 * (X[0, k+1] - Ref_s[k])**2
        cost += 5.0 * (X[1, k+1])**2               # 全局参考线跟踪
        cost += 40.0 * centering_cost             # 窄通道自适应居中势场!
        cost += 60.0 * barrier_cost               # 边缘防碰撞斥力
        cost += 8.0 * (X[2, k+1])**2
        cost += 1.0 * (X[3, k+1] - Ref_v[k])**2
        cost += 0.5 * (U[1, k] - Ref_w[k])**2
        cost += 0.2 * (U[0, k])**2
        if k > 0:
            cost += 2.0 * (U[1, k] - U[1, k-1])**2
        cost += 1000.0 * (Slack_L[k]**2 + Slack_R[k]**2)

    opti.minimize(cost)
    opts = {"ipopt.print_level": 0, "print_time": 0, "ipopt.sb": "yes", "ipopt.hessian_approximation": "exact", "ipopt.max_iter": 15}
    opti.solver("ipopt", opts)
    return opti.to_function("mpc_narrow", [X0, Ref_s, Ref_v, Ref_w, Ref_kappa, Corr_d_min, Corr_d_max], [U, X])

def test_narrow_passage():
    solver = create_mpc_solver()
    dt = 0.1
    sim_steps = 150

    def get_corridor(s_val):
        if 3.0 <= s_val <= 7.0:
            return 0.10, 0.50 # 窄通道: d_min=0.1, d_max=0.5, 几何中心 d_center = 0.30m
        elif 2.0 <= s_val < 3.0:
            r = (s_val - 2.0) / 1.0
            return -0.8 + r * 0.9, 0.8 - r * 0.3
        elif 7.0 < s_val <= 8.0:
            r = (8.0 - s_val) / 1.0
            return -0.8 + r * 0.9, 0.8 - r * 0.3
        else:
            return -0.8, 0.8

    state = np.array([0.0, 0.0, 0.0, 0.3])
    traj = []

    for step in range(sim_steps):
        s_curr = state[0]
        traj.append(state.copy())
        ref_s = [s_curr + (k + 1) * 0.35 * dt for k in range(10)]
        ref_v = [0.35] * 10
        ref_w = [0.0] * 10
        ref_kappa = [0.0] * 10

        d_min_seq = [get_corridor(rs)[0] for rs in ref_s]
        d_max_seq = [get_corridor(rs)[1] for rs in ref_s]

        in_args = [ca.DM(state), ca.DM(ref_s), ca.DM(ref_v), ca.DM(ref_w), ca.DM(ref_kappa), ca.DM(d_min_seq), ca.DM(d_max_seq)]
        res = solver(*in_args)
        u_opt = np.array(res[0])[:, 0]

        v = state[3] + u_opt[0] * dt
        w = u_opt[1]
        state[0] += v * np.cos(state[2]) * dt
        state[1] += v * np.sin(state[2]) * dt
        state[2] += w * dt
        state[3] = v

    traj = np.array(traj)

    # 绘图
    s_axis = np.linspace(0, 9.5, 200)
    c_left = [get_corridor(s)[1] for s in s_axis]
    c_right = [get_corridor(s)[0] for s in s_axis]
    c_center = [0.5 * (l + r) for l, r in zip(c_left, c_right)]

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.fill_between(s_axis, c_right, c_left, color='deepskyblue', alpha=0.25, label="Narrow Corridor Region [0.10, 0.50] m")
    ax.plot(s_axis, c_left, 'g-', linewidth=2.0, label="Corridor Left Boundary")
    ax.plot(s_axis, c_right, 'm-', linewidth=2.0, label="Corridor Right Boundary")
    ax.plot(s_axis, c_center, 'k--', linewidth=2.0, label="Corridor Centerline Target (d = 0.30 m)")

    # 障碍物区域
    ax.add_patch(plt.Rectangle((3.0, -0.8), 4.0, 0.9, color='crimson', alpha=0.5, hatch='//', label="Right Obstacle (Blocks d=0)"))
    ax.add_patch(plt.Rectangle((3.0, 0.5), 4.0, 0.5, color='orange', alpha=0.5, hatch='\\\\', label="Left Wall"))

    # 机器人轨迹
    ax.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=3.2, label="MPC Trajectory (Accurately Centered at d = 0.30 m)")
    ax.plot([0, 9.5], [0, 0], 'gray', linestyle=':', label="Original Global Reference Line (d=0)")

    ax.set_title("Narrow Passage Navigation: Width-Normalized Adaptive Centering", fontsize=13, fontweight='bold')
    ax.set_xlabel("Longitudinal s (m)", fontsize=11)
    ax.set_ylabel("Lateral Deviation d (m)", fontsize=11)
    ax.set_xlim(0, 9.2)
    ax.set_ylim(-0.9, 1.1)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left', fontsize=9.5, framealpha=0.9)

    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/narrow_corridor_centering_simulation.png", dpi=300)
    print("Narrow corridor simulation saved to narrow_corridor_centering_simulation.png")

if __name__ == "__main__":
    test_narrow_passage()
