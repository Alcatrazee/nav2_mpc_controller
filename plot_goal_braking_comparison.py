import numpy as np
import matplotlib.pyplot as plt

def test_braking():
    # 模拟从 2.0m 进近终点 (s_remain 从 2.0 减小到 0.0m)
    s_remain = np.linspace(2.0, 0.0, 200)
    a_min = 0.5
    v_max = 0.5
    goal_approach_dist = 0.6
    
    # 1. 之前的硬开关方式 (Hard Switch at 0.6m)
    v_term_hard = []
    for s in s_remain:
        if s <= goal_approach_dist:
            v_term_hard.append(0.01) # 瞬间跳变到 0.01!
        else:
            v_term_hard.append(v_max)
            
    # 2. 连续物理运动学漏斗约束 (Continuous Kinematic Funnel)
    v_term_smooth = []
    for s in s_remain:
        # 基于后向运动学平滑包络
        v_smooth = min(v_max, np.sqrt(2.0 * a_min * max(0.0, s)))
        v_term_smooth.append(v_smooth)
        
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(s_remain, v_term_hard, 'r--', linewidth=2.5, label="Previous Method: Hard Switch at 0.6m (Sudden Jerk & Hard Braking)")
    ax.plot(s_remain, v_term_smooth, 'b-', linewidth=3.0, label="Improved Method: Kinematic Funnel Envelope (Continuous & Smooth Braking)")
    ax.axvline(0.6, color='gray', linestyle=':', label="Approach Threshold 0.6m")
    
    ax.set_title("Terminal Velocity Bound: Hard Step Switch vs. Smooth Kinematic Funnel", fontsize=13, fontweight='bold')
    ax.set_xlabel("Remaining Distance to Goal s_remain (m)", fontsize=11)
    ax.set_ylabel("Terminal Velocity Upper Bound v_term (m/s)", fontsize=11)
    ax.invert_xaxis() # 距离向 0 递减
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper left', fontsize=10.5)
    
    plt.tight_layout()
    plt.savefig("/home/michael/turtlebot_ws/src/nav2_controller_template/goal_braking_comparison.png", dpi=300)
    print("Braking comparison plot saved.")

if __name__ == "__main__":
    test_braking()
