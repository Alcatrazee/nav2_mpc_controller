import numpy as np

def normalize_angle(angle):
    while angle > np.pi: angle -= 2.0 * np.pi
    while angle < -np.pi: angle += 2.0 * np.pi
    return angle

def test_overshoot_and_reversing():
    print("=" * 60)
    print("Testing Reversing Approach & Overshoot Calculations")
    print("=" * 60)

    # Goal at (0, 0), goal_yaw = 0.0
    goal_x, goal_y = 0.0, 0.0
    goal_yaw = 0.0

    # ----------------------------------------------------
    # Case 1: Forward Driving into Goal
    # Approach direction: along +x (theta = 0.0)
    # ----------------------------------------------------
    approach_theta_fwd = 0.0
    # Robot at 20cm before goal: x = -0.20, y = 0
    curr_x, curr_y = -0.20, 0.0
    dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)
    
    old_fwd_proj = (curr_x - goal_x) * np.cos(goal_yaw) + (curr_y - goal_y) * np.sin(goal_yaw)
    old_overshot_fwd = (old_fwd_proj > 0.02 and dist_to_goal < 0.30)
    
    new_fwd_proj = (curr_x - goal_x) * np.cos(approach_theta_fwd) + (curr_y - goal_y) * np.sin(approach_theta_fwd)
    new_overshot_fwd = (new_fwd_proj > 0.02 and dist_to_goal < 0.30)

    print(f"Forward (20cm before goal):")
    print(f"  Old overshot: {old_overshot_fwd} (proj={old_fwd_proj:.2f})")
    print(f"  New overshot: {new_overshot_fwd} (proj={new_fwd_proj:.2f})")
    assert not old_overshot_fwd and not new_overshot_fwd

    # ----------------------------------------------------
    # Case 2: Reversing (倒车) Driving into Goal
    # Robot is backing into goal from x = +0.25m to (0, 0).
    # Approach direction: along -x (approach_theta = pi)
    # Robot heading: current_theta = 0.0 (faces +x, moves in -x)
    # ----------------------------------------------------
    approach_theta_rev = np.pi
    curr_x, curr_y = 0.25, 0.0
    current_theta = 0.0
    dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)

    old_rev_proj = (curr_x - goal_x) * np.cos(goal_yaw) + (curr_y - goal_y) * np.sin(goal_yaw)
    old_overshot_rev = (old_rev_proj > 0.02 and dist_to_goal < 0.30)

    new_rev_proj = (curr_x - goal_x) * np.cos(approach_theta_rev) + (curr_y - goal_y) * np.sin(approach_theta_rev)
    new_overshot_rev = (new_rev_proj > 0.02 and dist_to_goal < 0.30)

    print(f"\nReversing (25cm before goal):")
    print(f"  Old overshot: {old_overshot_rev} (proj={old_rev_proj:.2f}) -> BUG: Premature stop & spin!")
    print(f"  New overshot: {new_overshot_rev} (proj={new_rev_proj:.2f}) -> FIXED: Continues reversing!")
    assert old_overshot_rev == True, "Old logic should have reproduced the bug"
    assert new_overshot_rev == False, "New logic should prevent premature overshoot"

    # ----------------------------------------------------
    # Case 3: Reversing (倒车) - True Overshoot
    # Robot backs past goal to x = -0.03m (3cm overshoot)
    # ----------------------------------------------------
    curr_x, curr_y = -0.03, 0.0
    dist_to_goal = np.hypot(goal_x - curr_x, goal_y - curr_y)
    new_rev_proj_over = (curr_x - goal_x) * np.cos(approach_theta_rev) + (curr_y - goal_y) * np.sin(approach_theta_rev)
    new_overshot_true = (new_rev_proj_over > 0.02 and dist_to_goal < 0.30)

    print(f"\nReversing (3cm past goal):")
    print(f"  New overshot: {new_overshot_true} (proj={new_rev_proj_over:.2f}) -> Correctly detected overshoot!")
    assert new_overshot_true == True

    # ----------------------------------------------------
    # Case 4: Line-of-sight Heading Error on Reversing
    # Robot at x = +0.80m, goal at (0, 0).
    # ----------------------------------------------------
    curr_x, curr_y = 0.80, 0.0
    current_theta = 0.0 # Facing away from goal while reversing into it
    los_yaw = np.arctan2(goal_y - curr_y, goal_x - curr_x) # np.pi
    
    old_direct_err = normalize_angle(los_yaw - current_theta) # np.pi -> 180 deg error!
    
    # New logic: when reversing, target los yaw points rear to goal
    target_los_yaw = normalize_angle(los_yaw + np.pi) # 0.0
    new_direct_err = normalize_angle(target_los_yaw - current_theta) # 0.0

    print(f"\nLine-of-Sight Attraction at 0.8m (Reversing):")
    print(f"  Old heading error: {np.degrees(old_direct_err):.1f} deg -> BUG: Injects max turn rate!")
    print(f"  New heading error: {np.degrees(new_direct_err):.1f} deg -> FIXED: Stays aligned!")
    assert abs(old_direct_err) > 3.0
    assert abs(new_direct_err) < 1e-4

    print("\n>>> ALL REVERSING VERIFICATION TESTS PASSED SUCCESSFULLY! <<<")

if __name__ == "__main__":
    test_overshoot_and_reversing()
