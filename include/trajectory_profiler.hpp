#ifndef NAV2_MPC_CONTROLLER__TRAJECTORY_PROFILER_HPP_
#define NAV2_MPC_CONTROLLER__TRAJECTORY_PROFILER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>

namespace nav2_mpc_controller
{

/**
 * @brief Continuous geometric path point
 */
struct PathPoint {
  double x, y, theta, kappa, s;
};

/**
 * @brief Time-parameterized trajectory point with target velocity
 */
struct TrajectoryPoint {
  double x, y, theta, kappa, s;
  double v; // Target velocity [m/s]
  double t; // Relative time stamp [s]
};

/**
 * @brief Dynamic constraints and limits for profiling
 */
struct ProfilerConfig {
  double max_velocity = 1.5;     // Absolute max forward speed [m/s]
  double max_a = 1.0;            // Max forward acceleration [m/s^2]
  double min_a = -1.0;           // Min forward acceleration (deceleration) [m/s^2]
  double max_lat_accel = 1.5;    // Max lateral acceleration [m/s^2]
};

/**
 * @class TrajectoryProfiler
 * @brief Generates a time-parameterized trajectory (LUT) based on curvature limits and 
 *        kinematics constraints, providing high-frequency target extraction for MPC.
 */
class TrajectoryProfiler
{
public:
  /**
   * @brief Construct a new Trajectory Profiler object
   * @param config Kinematics and dynamic limits
   */
  explicit TrajectoryProfiler(const ProfilerConfig & config);
  ~TrajectoryProfiler() = default;

  /**
   * @brief Applies kinematic constraints to generate a time-parameterized LUT
   * @param path Dense spatial path
   * @param current_speed Current initial speed of the robot
   */
  void generate_profile(const std::vector<PathPoint> & path, double current_speed);

  /**
   * @brief O(log N) query function to extract the interpolated reference point.
   *        Does not allocate dynamic memory; safe for high-frequency MPC calls.
   * @param target_time The query lookahead time
   * @return TrajectoryPoint The linearly interpolated reference state
   */
  TrajectoryPoint get_reference_point(double target_time) const;

  /**
   * @brief Retrieve the generated high-resolution trajectory LUT
   */
  const std::vector<TrajectoryPoint> & get_trajectory() const { return trajectory_; }

private:
  /**
   * @brief Normalize angle to [-pi, pi]
   */
  double normalize_angle(double angle) const;

  ProfilerConfig config_;
  std::vector<TrajectoryPoint> trajectory_;
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__TRAJECTORY_PROFILER_HPP_