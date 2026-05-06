#include "trajectory_profiler.hpp"

namespace nav2_mpc_controller
{

TrajectoryProfiler::TrajectoryProfiler(const ProfilerConfig & config)
: config_(config)
{
}

double TrajectoryProfiler::normalize_angle(double angle) const
{
  // Using atan2 to guarantee wrapping strictly between -pi and pi
  return std::atan2(std::sin(angle), std::cos(angle));
}

void TrajectoryProfiler::generate_profile(const std::vector<PathPoint> & path, double current_speed)
{
  if (path.empty()) {
    trajectory_.clear();
    return;
  }

  size_t N = path.size();
  trajectory_.resize(N);

  // Vector allocations only happen once per profile generation.
  std::vector<double> v_max(N, 0.0);
  std::vector<double> v_fwd(N, 0.0);
  std::vector<double> v_final(N, 0.0);

  // Step 1: Curvature-Based Speed Limit
  for (size_t i = 0; i < N; ++i) {
    double v_limit = std::sqrt(config_.max_lat_accel / (std::abs(path[i].kappa) + 1e-6));
    v_max[i] = std::min(v_limit, config_.max_velocity);
  }

  // Step 2: Forward Kinematic Pass (Acceleration)
  v_fwd[0] = std::max(0.0, current_speed); // Assuming we only move forward
  for (size_t i = 1; i < N; ++i) {
    double ds = path[i].s - path[i - 1].s;
    if (ds < 0.0) ds = 0.0;

    double v_fwd_next = std::sqrt(std::pow(v_fwd[i - 1], 2) + 2.0 * config_.max_lon_accel * ds);
    v_fwd[i] = std::min(v_max[i], v_fwd_next);
  }

  // Step 3: Backward Kinematic Pass (Deceleration)
  v_final[N - 1] = 0.0; // Assume full stop at the goal
  for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
    double ds = path[i + 1].s - path[i].s;
    if (ds < 0.0) ds = 0.0;

    double v_back = std::sqrt(std::pow(v_final[i + 1], 2) + 2.0 * std::abs(config_.max_lon_decel) * ds);
    v_final[i] = std::min(v_fwd[i], v_back);
  }

  // Step 4: Time Integration
  trajectory_[0].x = path[0].x;
  trajectory_[0].y = path[0].y;
  trajectory_[0].theta = path[0].theta;
  trajectory_[0].kappa = path[0].kappa;
  trajectory_[0].s = path[0].s;
  trajectory_[0].v = v_final[0];
  trajectory_[0].t = 0.0;

  for (size_t i = 1; i < N; ++i) {
    double ds = path[i].s - path[i - 1].s;
    if (ds < 0.0) ds = 0.0;

    // Trapezoidal time integration
    double dt = (2.0 * ds) / (v_final[i] + v_final[i - 1] + 1e-6);

    trajectory_[i].x = path[i].x;
    trajectory_[i].y = path[i].y;
    trajectory_[i].theta = path[i].theta;
    trajectory_[i].kappa = path[i].kappa;
    trajectory_[i].s = path[i].s;
    trajectory_[i].v = v_final[i];
    trajectory_[i].t = trajectory_[i - 1].t + dt;
  }
}

TrajectoryPoint TrajectoryProfiler::get_reference_point(double target_time) const
{
  // Guard: empty path
  if (trajectory_.empty()) {
    return TrajectoryPoint{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // Guard: time in the past
  if (target_time <= trajectory_.front().t) {
    return trajectory_.front();
  }

  // Guard: time exceeds total trajectory time
  if (target_time >= trajectory_.back().t) {
    return trajectory_.back();
  }

  // Step 5: MPC Target Extraction (O(log N) lookup)
  // Finds the first element in trajectory_ where `p.t >= target_time`
  auto it = std::lower_bound(
    trajectory_.begin(), trajectory_.end(), target_time,
    [](const auto& p, double t_val) {
        return p.t < t_val;
    }
  );

  // Fallback protection, though the above guards generally prevent this
  if (it == trajectory_.begin()) {
    return trajectory_.front();
  }

  auto it_left = std::prev(it);
  auto it_right = it;

  double t_left = it_left->t;
  double t_right = it_right->t;
  double dt = t_right - t_left;

  // Prevent division by zero just in case consecutive points share a timestamp
  if (dt < 1e-6) {
    return *it_left;
  }

  // Linear interpolation ratio
  double ratio = (target_time - t_left) / dt;

  TrajectoryPoint pt;
  // Interpolate continuous coordinates and path details
  pt.x = it_left->x + ratio * (it_right->x - it_left->x);
  pt.y = it_left->y + ratio * (it_right->y - it_left->y);
  pt.kappa = it_left->kappa + ratio * (it_right->kappa - it_left->kappa);
  pt.s = it_left->s + ratio * (it_right->s - it_left->s);
  pt.v = it_left->v + ratio * (it_right->v - it_left->v);
  pt.t = target_time;

  // Specific handling for angular orientation (avoiding -pi / pi flip)
  double diff_theta = normalize_angle(it_right->theta - it_left->theta);
  pt.theta = normalize_angle(it_left->theta + ratio * diff_theta);

  return pt;
}

}  // namespace nav2_mpc_controller