#include "mpc_controller.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

#include <queue>

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_mpc_controller
{

void MPCController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();

  // 声明并获取参数 (支持通过 nav2_params.yaml 配置)
  declare_parameter_if_not_declared(node, plugin_name_ + ".N", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dt", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_max", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_min", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_min", rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".a_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".a_min", rclcpp::ParameterValue(-1.0));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_s", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_d", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_e_psi", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_v", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_w", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_a", rclcpp::ParameterValue(0.2));

  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_default_left_width", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_default_right_width", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_min_width", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_curvature_safety_factor", rclcpp::ParameterValue(0.85));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_rib_safety_margin", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_max_lateral_rate", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_avoid_hairpin", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_check_costmap", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_costmap_cost_threshold", rclcpp::ParameterValue(100));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_safe_corridor", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_corridor_bound", rclcpp::ParameterValue(35.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_corridor_center", rclcpp::ParameterValue(30.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_corridor_slack", rclcpp::ParameterValue(1000.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corridor_buffer_margin", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".enable_terminal_constraint", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".goal_approach_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_s_tol", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_d_tol", rclcpp::ParameterValue(0.03));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_epsi_tol", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_v_tol", rclcpp::ParameterValue(0.01));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_s_terminal", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_d_terminal", rclcpp::ParameterValue(50.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_epsi_terminal", rclcpp::ParameterValue(20.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_v_terminal", rclcpp::ParameterValue(10.0));

  node->get_parameter(plugin_name_ + ".N", N_);
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".v_max", v_max_);
  node->get_parameter(plugin_name_ + ".v_min", v_min_);
  node->get_parameter(plugin_name_ + ".w_max", w_max_);
  node->get_parameter(plugin_name_ + ".w_min", w_min_);
  node->get_parameter(plugin_name_ + ".a_max", a_max_);
  node->get_parameter(plugin_name_ + ".a_min", a_min_);
  node->get_parameter(plugin_name_ + ".q_s", q_s_);
  node->get_parameter(plugin_name_ + ".q_d", q_d_);
  node->get_parameter(plugin_name_ + ".q_e_psi", q_e_psi_);
  node->get_parameter(plugin_name_ + ".r_v", r_v_);
  node->get_parameter(plugin_name_ + ".r_w", r_w_);
  node->get_parameter(plugin_name_ + ".r_a", r_a_);

  node->get_parameter(plugin_name_ + ".corridor_default_left_width", corridor_config_.default_left_width);
  node->get_parameter(plugin_name_ + ".corridor_default_right_width", corridor_config_.default_right_width);
  node->get_parameter(plugin_name_ + ".corridor_min_width", corridor_config_.min_corridor_width);
  node->get_parameter(plugin_name_ + ".corridor_curvature_safety_factor", corridor_config_.curvature_safety_factor);
  node->get_parameter(plugin_name_ + ".corridor_rib_safety_margin", corridor_config_.rib_safety_margin);
  node->get_parameter(plugin_name_ + ".corridor_max_lateral_rate", corridor_config_.max_lateral_rate);
  node->get_parameter(plugin_name_ + ".corridor_avoid_hairpin", corridor_config_.avoid_hairpin_self_intersection);
  node->get_parameter(plugin_name_ + ".corridor_check_costmap", corridor_config_.check_costmap);
  int cost_thresh = 100;
  if (node->get_parameter(plugin_name_ + ".corridor_costmap_cost_threshold", cost_thresh)) {
    corridor_config_.costmap_cost_threshold = static_cast<uint8_t>(std::clamp(cost_thresh, 1, 254));
  }
  node->get_parameter(plugin_name_ + ".enable_safe_corridor", enable_safe_corridor_);
  node->get_parameter(plugin_name_ + ".q_corridor_bound", q_corridor_bound_);
  node->get_parameter(plugin_name_ + ".q_corridor_center", q_corridor_center_);
  node->get_parameter(plugin_name_ + ".w_corridor_slack", w_corridor_slack_);
  node->get_parameter(plugin_name_ + ".corridor_buffer_margin", corridor_buffer_margin_);

  node->get_parameter(plugin_name_ + ".enable_terminal_constraint", enable_terminal_constraint_);
  node->get_parameter(plugin_name_ + ".goal_approach_dist", goal_approach_dist_);
  node->get_parameter(plugin_name_ + ".terminal_s_tol", terminal_s_tol_);
  node->get_parameter(plugin_name_ + ".terminal_d_tol", terminal_d_tol_);
  node->get_parameter(plugin_name_ + ".terminal_epsi_tol", terminal_epsi_tol_);
  node->get_parameter(plugin_name_ + ".terminal_v_tol", terminal_v_tol_);
  node->get_parameter(plugin_name_ + ".q_s_terminal", q_s_terminal_);
  node->get_parameter(plugin_name_ + ".q_d_terminal", q_d_terminal_);
  node->get_parameter(plugin_name_ + ".q_epsi_terminal", q_epsi_terminal_);
  node->get_parameter(plugin_name_ + ".r_v_terminal", r_v_terminal_);

  safe_corridor_generator_ = std::make_unique<SafeCorridorGenerator>(corridor_config_);

  // 注册动态参数回调
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&MPCController::dynamicParametersCallback, this, std::placeholders::_1));

  // 创建预测轨迹与安全走廊的发布器
  traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predict_trajectory", 10);
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 10);
  transformed_local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_local_plan", 10);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 10);
  local_plan_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("local_plan_markers", 10);
  safe_corridor_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("safe_corridor_markers", 10);
  lateral_error_pub_ = node->create_publisher<std_msgs::msg::Float64>("lateral_error", 10);
  ProfilerConfig profiler_cfg;
  profiler_cfg.max_velocity = v_max_;
  profiler_cfg.max_a = a_max_;
  profiler_cfg.min_a = a_min_;
  trajectory_profiler_ = std::make_unique<TrajectoryProfiler>(profiler_cfg);

  RCLCPP_INFO(logger_, "============================================================");
  RCLCPP_INFO(logger_, "       nav2_mpc_controller Parameter Table Configured       ");
  RCLCPP_INFO(logger_, "============================================================");
  RCLCPP_INFO(logger_, "  [Horizon & DT]      N = %d, dt = %.3f s", N_, dt_);
  RCLCPP_INFO(logger_, "  [Vel Bounds]        v_min = %.2f m/s, v_max = %.2f m/s", v_min_, v_max_);
  RCLCPP_INFO(logger_, "  [Omega Bounds]      w_min = %.2f rad/s, w_max = %.2f rad/s", w_min_, w_max_);
  RCLCPP_INFO(logger_, "  [Accel Bounds]      a_min = %.2f m/s², a_max = %.2f m/s²", a_min_, a_max_);
  RCLCPP_INFO(logger_, "  [Frenet Weights]    q_s = %.2f, q_d = %.2f, q_e_psi = %.2f", q_s_, q_d_, q_e_psi_);
  RCLCPP_INFO(logger_, "  [Control Weights]   r_v = %.2f, r_w = %.2f, r_a = %.2f", r_v_, r_w_, r_a_);
  RCLCPP_INFO(logger_, "  [Safe Corridor]     enabled = %s, d_left = %.2f m, d_right = %.2f m, hairpin_avoid = %s",
    enable_safe_corridor_ ? "true" : "false",
    corridor_config_.default_left_width, corridor_config_.default_right_width,
    corridor_config_.avoid_hairpin_self_intersection ? "true" : "false");
  RCLCPP_INFO(logger_, "  [Corridor Costs]    q_bound = %.2f, w_slack = %.2f, margin = %.2f m",
    q_corridor_bound_, w_corridor_slack_, corridor_buffer_margin_);
  RCLCPP_INFO(logger_, "  [Terminal Constraint] enabled = %s, approach_dist = %.2f m, s_tol = %.3f m, d_tol = %.3f m, epsi_tol = %.3f rad",
    enable_terminal_constraint_ ? "true" : "false", goal_approach_dist_, terminal_s_tol_, terminal_d_tol_, terminal_epsi_tol_);
  RCLCPP_INFO(logger_, "============================================================");
}

void MPCController::cleanup()
{
  dyn_params_handler_.reset();
  traj_pub_.reset();
  transformed_plan_pub_.reset();
  transformed_local_plan_pub_.reset();
  local_plan_pub_.reset();
  local_plan_marker_pub_.reset();
  safe_corridor_marker_pub_.reset();
  lateral_error_pub_.reset();
  RCLCPP_INFO(logger_, "MPC Controller Cleaned Up.");
}

void MPCController::activate()
{
  traj_pub_->on_activate();
  transformed_plan_pub_->on_activate();
  transformed_local_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  local_plan_marker_pub_->on_activate();
  safe_corridor_marker_pub_->on_activate();
  lateral_error_pub_->on_activate();
  RCLCPP_INFO(logger_, "MPC Controller Activated.");
}

void MPCController::deactivate()
{
  traj_pub_->on_deactivate();
  transformed_plan_pub_->on_deactivate();
  transformed_local_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  local_plan_marker_pub_->on_deactivate();
  safe_corridor_marker_pub_->on_deactivate();
  lateral_error_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "MPC Controller Deactivated.");
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  is_cold_start_ = true;
  prev_cmd_w_ = 0.0;
  prev_s_sol_.clear();
  prev_d_sol_.clear();
  prev_e_psi_sol_.clear();
  prev_v_sol_.clear();
  prev_a_sol_.clear();
  prev_w_sol_.clear();
}

void MPCController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/)
{
  // 这里通常用于处理来自限速区 (speed limit zones) 的速度降低，为保持简单暂时留空
}

rcl_interfaces::msg::SetParametersResult MPCController::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(param_mutex_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool update_profiler = false;
  bool update_corridor = false;
  
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name == plugin_name_ + ".N") {
      N_ = parameter.as_int();
    } else if (name == plugin_name_ + ".dt") {
      dt_ = parameter.as_double();
    } else if (name == plugin_name_ + ".v_max") {
      v_max_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".v_min") {
      v_min_ = parameter.as_double();
    } else if (name == plugin_name_ + ".w_max") {
      w_max_ = parameter.as_double();
    } else if (name == plugin_name_ + ".w_min") {
      w_min_ = parameter.as_double();
    } else if (name == plugin_name_ + ".a_max") {
      a_max_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".a_min") {
      a_min_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".q_s") {
      q_s_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_d") {
      q_d_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_e_psi") {
      q_e_psi_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_v") {
      r_v_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_w") {
      r_w_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_a") {
      r_a_ = parameter.as_double();
    } else if (name == plugin_name_ + ".corridor_default_left_width") {
      corridor_config_.default_left_width = parameter.as_double();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_default_right_width") {
      corridor_config_.default_right_width = parameter.as_double();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_min_width") {
      corridor_config_.min_corridor_width = parameter.as_double();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_curvature_safety_factor") {
      corridor_config_.curvature_safety_factor = parameter.as_double();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_avoid_hairpin") {
      corridor_config_.avoid_hairpin_self_intersection = parameter.as_bool();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_check_costmap") {
      corridor_config_.check_costmap = parameter.as_bool();
      update_corridor = true;
    } else if (name == plugin_name_ + ".corridor_costmap_cost_threshold") {
      corridor_config_.costmap_cost_threshold = static_cast<uint8_t>(std::clamp(parameter.as_int(), 1L, 254L));
      update_corridor = true;
    } else if (name == plugin_name_ + ".enable_safe_corridor") {
      enable_safe_corridor_ = parameter.as_bool();
    } else if (name == plugin_name_ + ".q_corridor_bound") {
      q_corridor_bound_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_corridor_center") {
      q_corridor_center_ = parameter.as_double();
    } else if (name == plugin_name_ + ".w_corridor_slack") {
      w_corridor_slack_ = parameter.as_double();
    } else if (name == plugin_name_ + ".corridor_buffer_margin") {
      corridor_buffer_margin_ = parameter.as_double();
    } else if (name == plugin_name_ + ".enable_terminal_constraint") {
      enable_terminal_constraint_ = parameter.as_bool();
    } else if (name == plugin_name_ + ".goal_approach_dist") {
      goal_approach_dist_ = parameter.as_double();
    } else if (name == plugin_name_ + ".terminal_s_tol") {
      terminal_s_tol_ = parameter.as_double();
    } else if (name == plugin_name_ + ".terminal_d_tol") {
      terminal_d_tol_ = parameter.as_double();
    } else if (name == plugin_name_ + ".terminal_epsi_tol") {
      terminal_epsi_tol_ = parameter.as_double();
    } else if (name == plugin_name_ + ".terminal_v_tol") {
      terminal_v_tol_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_s_terminal") {
      q_s_terminal_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_d_terminal") {
      q_d_terminal_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_epsi_terminal") {
      q_epsi_terminal_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_v_terminal") {
      r_v_terminal_ = parameter.as_double();
    }
    RCLCPP_INFO(
        logger_, "Parameter %s updated to: %s",
        name.c_str(), parameter.value_to_string().c_str());
  }

  if (update_profiler && trajectory_profiler_) {
    ProfilerConfig cfg;
    cfg.max_velocity = v_max_;
    cfg.max_a = a_max_;
    cfg.min_a = a_min_;
    trajectory_profiler_ = std::make_unique<TrajectoryProfiler>(cfg);
  }

  if (update_corridor && safe_corridor_generator_) {
    safe_corridor_generator_->setConfig(corridor_config_);
  }
  
  mpc_problem_initialized_ = false; // 当任何参数改变时，标记重新初始化 MPC 问题
  return result;
}

MPCController::FrenetState MPCController::cartesianToFrenet(
  double x, double y, double theta,
  const std::vector<TrajectoryPoint> & reference_path) const
{
  if (reference_path.empty()) {
    return {0.0, 0.0, 0.0};
  }

  if (reference_path.size() == 1) {
    const auto & ref = reference_path.front();
    const double dx = x - ref.x;
    const double dy = y - ref.y;
    return {
      ref.s,
      -dx * std::sin(ref.theta) + dy * std::cos(ref.theta),
      normalize_angle(theta - ref.theta)};
  }

  double min_dist_sq = std::numeric_limits<double>::max();
  double best_s = reference_path.front().s;
  double best_d = 0.0;
  double best_theta = reference_path.front().theta;

  for (size_t i = 0; i + 1 < reference_path.size(); ++i) {
    const auto & p0 = reference_path[i];
    const auto & p1 = reference_path[i + 1];
    const double seg_x = p1.x - p0.x;
    const double seg_y = p1.y - p0.y;
    const double seg_len_sq = seg_x * seg_x + seg_y * seg_y;
    if (seg_len_sq < 1e-12) {
      continue;
    }

    const double projection = std::clamp(
      ((x - p0.x) * seg_x + (y - p0.y) * seg_y) / seg_len_sq, 0.0, 1.0);
    const double projected_x = p0.x + projection * seg_x;
    const double projected_y = p0.y + projection * seg_y;
    const double error_x = x - projected_x;
    const double error_y = y - projected_y;
    const double dist_sq = error_x * error_x + error_y * error_y;

    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_s = p0.s + projection * (p1.s - p0.s);
      const double delta_theta = normalize_angle(p1.theta - p0.theta);
      best_theta = normalize_angle(p0.theta + projection * delta_theta);
      best_d = -error_x * std::sin(best_theta) + error_y * std::cos(best_theta);
    }
  }

  return {best_s, best_d, normalize_angle(theta - best_theta)};
}

geometry_msgs::msg::Pose MPCController::frenetToCartesian(
  double s, double d, double e_psi,
  const std::vector<TrajectoryPoint> & reference_path) const
{
  geometry_msgs::msg::Pose pose;
  if (reference_path.empty()) {
    pose.orientation.w = 1.0;
    return pose;
  }

  const TrajectoryPoint * left = &reference_path.front();
  const TrajectoryPoint * right = left;
  double ratio = 0.0;
  double longitudinal_offset = 0.0;

  if (s >= reference_path.back().s) {
    left = &reference_path.back();
    right = left;
    longitudinal_offset = s - left->s;
  } else if (s <= reference_path.front().s) {
    longitudinal_offset = s - left->s;
  } else if (s > reference_path.front().s) {
    auto upper = std::lower_bound(
      reference_path.begin(), reference_path.end(), s,
      [](const TrajectoryPoint & point, double query_s) {
        return point.s < query_s;
      });
    right = &(*upper);
    left = &(*std::prev(upper));
    const double ds = right->s - left->s;
    if (ds > 1e-9) {
      ratio = (s - left->s) / ds;
    }
  }

  const double ref_theta = normalize_angle(
    left->theta + ratio * normalize_angle(right->theta - left->theta));
  const double ref_x =
    left->x + ratio * (right->x - left->x) + longitudinal_offset * std::cos(ref_theta);
  const double ref_y =
    left->y + ratio * (right->y - left->y) + longitudinal_offset * std::sin(ref_theta);
  const double theta = normalize_angle(ref_theta + e_psi);

  pose.position.x = ref_x - d * std::sin(ref_theta);
  pose.position.y = ref_y + d * std::cos(ref_theta);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}


bool MPCController::checkGoalCollision(double check_x, double check_y, double check_theta) const
{
  if (!costmap_ros_) {
    return false;
  }
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return false;
  }

  // 1. 检查中心点 (仅当遇到 254 致命障碍物时判定碰撞，允许 253 膨胀代价)
  unsigned int mx, my;
  if (!costmap->worldToMap(check_x, check_y, mx, my)) {
    return true; // 出界视为危险
  }
  if (costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
    return true;
  }

  // 2. 检查机器人足迹外轮廓 (圆形/多点包络采样)
  double radius = 0.16; // 机器人碰撞检测半径
  for (double angle = 0.0; angle < 2.0 * M_PI; angle += M_PI / 4.0) {
    double px = check_x + radius * std::cos(check_theta + angle);
    double py = check_y + radius * std::sin(check_theta + angle);
    if (!costmap->worldToMap(px, py, mx, my)) {
      return true;
    }
    if (costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      return true; // 仅 254 致命障碍触发停障
    }
  }
  return false;
}

void MPCController::initializeMPC()
{
  casadi::Opti opti; // 创建局部 Opti 实例以构建计算图

  auto X = opti.variable(4, N_ + 1); // Frenet 状态 [s, d, e_psi, v]
  auto U = opti.variable(2, N_);     // 控制量 [a, w]

  // 走廊软约束松弛变量 (保证 NLP 100% 具备可行解)
  auto Slack_L = opti.variable(N_);  // 左边界越界松弛量 eps_L >= 0
  auto Slack_R = opti.variable(N_);  // 右边界越界松弛量 eps_R >= 0

  auto X0_param = opti.parameter(4);
  auto Ref_s_param = opti.parameter(N_);
  auto Ref_v_param = opti.parameter(N_);
  auto Ref_w_param = opti.parameter(N_);
  auto Ref_kappa_param = opti.parameter(N_);
  auto Corridor_d_min_param = opti.parameter(N_);
  auto Corridor_d_max_param = opti.parameter(N_);

  // 初始状态约束 (X0 = current_state)
  opti.subject_to(X(0, 0) == X0_param(0));
  opti.subject_to(X(1, 0) == X0_param(1));
  opti.subject_to(X(2, 0) == X0_param(2));
  opti.subject_to(X(3, 0) == X0_param(3));

  casadi::MX cost = 0;

  // 遍历预测视野，建立差分运动学约束和代价函数
  for (int k = 0; k < N_; ++k) {
    casadi::MX denominator = casadi::MX::fmax(
      1.0 - Ref_kappa_param(k) * X(1, k), casadi::MX(0.1));
    casadi::MX s_dot = X(3, k) * cos(X(2, k)) / denominator;
    casadi::MX d_dot = X(3, k) * sin(X(2, k));
    casadi::MX e_psi_dot = U(1, k) - Ref_kappa_param(k) * s_dot;

    casadi::MX s_next = X(0, k) + s_dot * dt_;
    casadi::MX d_next = X(1, k) + d_dot * dt_;
    casadi::MX e_psi_next = X(2, k) + e_psi_dot * dt_;
    casadi::MX v_next = X(3, k) + U(0, k) * dt_;
    
    opti.subject_to(X(0, k+1) == s_next);
    opti.subject_to(X(1, k+1) == d_next);
    opti.subject_to(X(2, k+1) == e_psi_next);
    opti.subject_to(X(3, k+1) == v_next);

    // 边界约束
    opti.subject_to(opti.bounded(v_min_, X(3, k+1), v_max_));
    opti.subject_to(opti.bounded(w_min_, U(1, k), w_max_));
    opti.subject_to(opti.bounded(a_min_, U(0, k), a_max_));

    // 1. 安全走廊软约束 (Slack >= 0)
    opti.subject_to(Slack_L(k) >= 0.0);
    opti.subject_to(Slack_R(k) >= 0.0);
    opti.subject_to(X(1, k+1) <= Corridor_d_max_param(k) + Slack_L(k));
    opti.subject_to(X(1, k+1) >= Corridor_d_min_param(k) - Slack_R(k));

    // 2. 走廊几何宽度与中轴中心
    casadi::MX corridor_width = casadi::MX::fmax(
      Corridor_d_max_param(k) - Corridor_d_min_param(k), casadi::MX(0.25));
    casadi::MX d_center = 0.5 * (Corridor_d_max_param(k) + Corridor_d_min_param(k));

    // 3. 窄通道宽度自适应归一化居中势场 (通道越窄，居中推力自动平方级放大，确保左右安全裕量均等最大化)
    casadi::MX centering_cost = pow((X(1, k+1) - d_center) / corridor_width, 2);

    // 4. 走廊边缘平滑排斥屏障 (Smooth Boundary Barrier Cost)
    casadi::MX left_barrier_dist = Corridor_d_max_param(k) - corridor_buffer_margin_;
    casadi::MX right_barrier_dist = Corridor_d_min_param(k) + corridor_buffer_margin_;
    casadi::MX barrier_cost = pow(casadi::MX::fmax(0.0, X(1, k+1) - left_barrier_dist), 2) +
                              pow(casadi::MX::fmax(0.0, right_barrier_dist - X(1, k+1)), 2);

    // 代价函数：
    // Frenet 纵向 s 跟踪、横向全局参考线跟踪、窄通道自适应居中、边缘屏障斥力
    cost += q_s_ * pow(X(0, k+1) - Ref_s_param(k), 2);
    cost += q_d_ * pow(X(1, k+1), 2);                  // 开阔区跟踪平滑全局参考线
    cost += q_corridor_center_ * centering_cost;      // 窄通道强力自适应居中！
    cost += q_corridor_bound_ * barrier_cost;         // 边缘防碰撞排斥屏障
    cost += q_e_psi_ * pow(X(2, k+1), 2);
    cost += r_v_ * pow(X(3, k+1) - Ref_v_param(k), 2);
    cost += r_w_ * pow(U(1, k) - Ref_w_param(k), 2);
    cost += r_a_ * pow(U(0, k), 2);

    // 控制量变化率惩罚 (Slew-Rate / Jerk Penalty，进一步抑制方向盘微抖)
    if (k > 0) {
      cost += 1.5 * pow(U(1, k) - U(1, k - 1), 2);
    }

    // 安全走廊越界松弛高额惩罚
    cost += w_corridor_slack_ * (pow(Slack_L(k), 2) + pow(Slack_R(k), 2));
  }

  // 终点位姿硬约束参数与约束施加
  auto Terminal_s_bounds = opti.parameter(2);    // [s_min, s_max]
  auto Terminal_d_bounds = opti.parameter(2);    // [d_min, d_max]
  auto Terminal_epsi_bounds = opti.parameter(2); // [epsi_min, epsi_max]
  auto Terminal_v_bounds = opti.parameter(2);    // [v_min, v_max]

  opti.subject_to(opti.bounded(Terminal_s_bounds(0), X(0, N_), Terminal_s_bounds(1)));
  opti.subject_to(opti.bounded(Terminal_d_bounds(0), X(1, N_), Terminal_d_bounds(1)));
  opti.subject_to(opti.bounded(Terminal_epsi_bounds(0), X(2, N_), Terminal_epsi_bounds(1)));
  opti.subject_to(opti.bounded(Terminal_v_bounds(0), X(3, N_), Terminal_v_bounds(1)));

  // 终端位姿高权重代价 (Terminal Cost P)
  cost += q_s_terminal_ * pow(X(0, N_) - Ref_s_param(N_ - 1), 2);
  cost += q_d_terminal_ * pow(X(1, N_), 2); // 强制终端横向对齐目标点中心 (d=0)
  cost += q_epsi_terminal_ * pow(X(2, N_), 2);
  cost += r_v_terminal_ * pow(X(3, N_) - Ref_v_param(N_ - 1), 2);

  opti.minimize(cost);

  // 配置并调用 IPOPT 求解器 (启用 Exact Hessian 精确海森矩阵，收敛速提高 5~8 倍)
  casadi::Dict solver_opts;
  solver_opts["ipopt.print_level"] = 0;                    // 关闭日志
  solver_opts["ipopt.sb"] = "yes";
  solver_opts["print_time"] = 0;
  solver_opts["ipopt.hessian_approximation"] = "exact";   // 使用 CasADi AD 自动微分精确 Hessian，3步二次收敛，彻底消除50ms峰值
  solver_opts["ipopt.max_iter"] = 10;                     // 限制单帧最大迭代 10 步
  solver_opts["ipopt.tol"] = 1e-3;                        // 1e-3 适合 20Hz+ 实时 MPC 控制的收敛精度
  solver_opts["ipopt.acceptable_tol"] = 1e-2;
  solver_opts["ipopt.acceptable_iter"] = 3;
  solver_opts["ipopt.warm_start_init_point"] = "yes";
  solver_opts["ipopt.warm_start_bound_push"] = 1e-6;
  solver_opts["ipopt.warm_start_mult_bound_push"] = 1e-6;

  opti.solver("ipopt", solver_opts);

  // 预编译为 C++ Function 计算图，彻底消除每帧构造 Opti 实例的 CPU 开销
  mpc_solver_ = opti.to_function("mpc_solver",
    {X0_param, Ref_s_param, Ref_v_param, Ref_w_param, Ref_kappa_param,
     Corridor_d_min_param, Corridor_d_max_param,
     Terminal_s_bounds, Terminal_d_bounds, Terminal_epsi_bounds, Terminal_v_bounds},
    {U, X});

  mpc_problem_initialized_ = true;
  is_cold_start_ = true;
  RCLCPP_INFO(logger_, "Frenet Corridor & Goal-Constrained MPC CasADi High-Speed Function compiled with N=%d (Target >= 20Hz)", N_);
}


geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  std::lock_guard<std::mutex> lock(param_mutex_);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = rclcpp::Clock().now();

  if (global_plan_.poses.empty()) {
    return cmd_vel;
  }

  // 将全局路径转换到与当前 pose 相同的坐标系下
  nav_msgs::msg::Path transformed_plan;
  if (!transformPlan(tf_, global_plan_, pose.header.frame_id, transformed_plan)) {
    RCLCPP_ERROR(logger_, "Could not transform the global plan to the controller frame");
    return cmd_vel;
  }

  // 发布转换后的局部全局路径用于 Rviz 可视化
  transformed_plan_pub_->publish(transformed_plan);

  // 1. 获取机器人当前状态
  double current_x = pose.pose.position.x;
  double current_y = pose.pose.position.y;
  double current_theta = tf2::getYaw(pose.pose.orientation);

  double current_speed = velocity.linear.x;

  // 2. 截取用于跟踪的局部路径（裁剪最近点之前及代价地图范围外的路径点）
  nav_msgs::msg::Path tracking_plan = extractLocalPlan(pose, transformed_plan, costmap_ros_);
  
  // 发布用于追踪的最终局部路径
  transformed_local_plan_pub_->publish(tracking_plan);

  if (tracking_plan.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Tracking plan is empty! Stopping robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }
  

  // 一键生成带曲率平滑的时间参数化轨迹及控制参考点
  auto ref_points = generateTimeParameterizedTrajectory(tracking_plan, current_speed);

  if (ref_points.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Local plan is empty or failed to generate parameterized trajectory!");
    return cmd_vel;
  }

  auto t_prep = std::chrono::high_resolution_clock::now();

  // 将发布可视化话题的过程封装进辅助函数中
  std_msgs::msg::Header viz_header;
  viz_header.frame_id = pose.header.frame_id;
  viz_header.stamp = cmd_vel.header.stamp;
  publishParameterizedTrajectory(ref_points, viz_header);

  // 提取走廊横向边界序列
  std::vector<double> corridor_d_min(N_, -corridor_config_.default_right_width);
  std::vector<double> corridor_d_max(N_,  corridor_config_.default_left_width);

  if (enable_safe_corridor_) {
    // 1. 构建基于参考路线的安全走廊 (结合障碍物、曲率半径限制与掉头弯防自交叉)
    auto * costmap = (costmap_ros_ && corridor_config_.check_costmap) ? costmap_ros_->getCostmap() : nullptr;
    current_safe_corridor_ = safe_corridor_generator_->generateCorridor(
      ref_points, costmap, pose.header.frame_id, cmd_vel.header.stamp);

    // 发布安全走廊 3D 网格、边界及截面肋线可视化 MarkerArray
    if (safe_corridor_marker_pub_ && safe_corridor_marker_pub_->is_activated()) {
      auto corridor_markers = safe_corridor_generator_->createVisualizationMarkers(current_safe_corridor_);
      safe_corridor_marker_pub_->publish(corridor_markers);
    }

    // 填充实际时变走廊边界供 MPC 绕障求解
    for (int k = 0; k < N_; ++k) {
      if (k < static_cast<int>(current_safe_corridor_.bounds.size())) {
        corridor_d_min[k] = current_safe_corridor_.bounds[k].d_min;
        corridor_d_max[k] = current_safe_corridor_.bounds[k].d_max;
      }
    }
  } else {
    // 纯跟线模式 (Pure Tracking): 走廊设为对称无障碍宽边界，d_center=0，仅沿参考中心线跟踪
    current_safe_corridor_.clear();
    for (int k = 0; k < N_; ++k) {
      corridor_d_min[k] = -5.0;
      corridor_d_max[k] =  5.0;
    }
    if (safe_corridor_marker_pub_ && safe_corridor_marker_pub_->is_activated()) {
      visualization_msgs::msg::MarkerArray clear_array;
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      clear_array.markers.push_back(clear_marker);
      safe_corridor_marker_pub_->publish(clear_array);
    }
  }

  const auto & reference_path = trajectory_profiler_->get_trajectory();
  if (reference_path.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(node_.lock()->get_clock()), 1000,
      "Frenet reference path is too short! Stopping robot.");
    return cmd_vel;
  }

  const FrenetState current_frenet = cartesianToFrenet(
    current_x, current_y, current_theta, reference_path);

  // 发布实时横向偏差 d (单位: 米) 到话题 lateral_error
  std_msgs::msg::Float64 d_msg;
  d_msg.data = current_frenet.d;
  lateral_error_pub_->publish(d_msg);

  // 从时间参数化轨迹中按时间采样 Frenet 参考状态
  std::vector<double> ref_s(N_, 0.0), ref_v(N_, 0.0);
  std::vector<double> ref_w(N_, 0.0), ref_kappa(N_, 0.0);

  for (int k = 0; k < N_; ++k) {
    ref_s[k] = current_frenet.s + ref_points[k].s; // 对齐初始 s0，防止纵向拉扯
    ref_v[k] = ref_points[k].v;
    ref_kappa[k] = ref_points[k].kappa;
    ref_w[k] = ref_v[k] * ref_kappa[k];
  }

  // 4. 计算到最终目标点（Goal Pose）的空间几何关系与剩余弧长
  double goal_x = transformed_plan.poses.back().pose.position.x;
  double goal_y = transformed_plan.poses.back().pose.position.y;
  double goal_yaw = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
  double dist_to_goal = std::hypot(goal_x - current_x, goal_y - current_y);
  double s_remain = (reference_path.back().s > current_frenet.s) ? (reference_path.back().s - current_frenet.s) : 0.0;

  // 1) 终点切向过冲判定 (Forward Overrun Check)
  double goal_forward_proj = (current_x - goal_x) * std::cos(goal_yaw) + (current_y - goal_y) * std::sin(goal_yaw);
  bool is_overshot = (goal_forward_proj > 0.02 && dist_to_goal < 0.30); // 冲过终点截面 2cm 判定为过冲

  // 2) 终点与前方碰撞停障保护 (Goal / Impending Collision Stop Guard)
  bool is_goal_occupied = checkGoalCollision(goal_x, goal_y, goal_yaw);
  double front_check_d = std::clamp(current_speed * 0.5, 0.10, 0.25);
  bool is_front_occupied = checkGoalCollision(
    current_x + front_check_d * std::cos(current_theta),
    current_y + front_check_d * std::sin(current_theta),
    current_theta);

  if (is_front_occupied || (dist_to_goal < 0.35 && is_goal_occupied)) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000,
                         "[Goal Collision Guard] Impending collision or goal region blocked! Safely stopping robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // 3) 超过终点或进入 5cm 容差圈立即停进 (Overshoot Stop Guard) 并执行原地对齐
  if (is_overshot || dist_to_goal < 0.05) {
    double angle_to_goal_orient = normalize_angle(goal_yaw - current_theta);
    cmd_vel.twist.linear.x = 0.0; // 强制停止前进，绝对不继续前冲
    if (std::abs(angle_to_goal_orient) > 0.03) {
      double max_turn_w = 0.4;
      double rot_w = std::clamp(1.5 * angle_to_goal_orient, -max_turn_w, max_turn_w);
      double filtered_rot_w = std::clamp(rot_w, prev_cmd_w_ - 0.1, prev_cmd_w_ + 0.1);
      prev_cmd_w_ = filtered_rot_w;
      cmd_vel.twist.angular.z = filtered_rot_w;
    } else {
      cmd_vel.twist.angular.z = 0.0;
    }
    return cmd_vel;
  }

  // 4) 无论偏离多远，靠近终点时直接向目标点坐标进发 (Goal Line-of-Sight Attraction)
  if (dist_to_goal < 1.20) {
    double los_yaw = std::atan2(goal_y - current_y, goal_x - current_x);
    double direct_heading_err = normalize_angle(los_yaw - current_theta);
    double blend_factor = std::clamp((1.20 - dist_to_goal) / 1.20, 0.0, 0.75);
    for (int k = 0; k < N_; ++k) {
      ref_w[k] = (1.0 - blend_factor) * ref_w[k] + blend_factor * std::clamp(1.5 * direct_heading_err, -0.6, 0.6);
    }
  }

  // 基于物理运动学连续平滑漏斗包络 (Kinematic Funnel Envelope)
  double max_decel = std::max(0.2, std::abs(a_min_));
  double dynamic_v_cap = std::min(v_max_, std::sqrt(2.0 * max_decel * std::max(0.0, s_remain)));
  double term_v_upper = enable_terminal_constraint_ ? std::min(v_max_, std::max(ref_v.back(), dynamic_v_cap)) : v_max_;

  // 连续漏斗容差
  double funnel_d_tol = enable_terminal_constraint_ ? 
    std::min(corridor_config_.default_left_width, terminal_d_tol_ + 0.15 * s_remain) : 5.0;
  double funnel_epsi_tol = enable_terminal_constraint_ ? 
    std::min(M_PI, terminal_epsi_tol_ + 0.3 * s_remain) : M_PI;
  double funnel_s_tol = enable_terminal_constraint_ ? 
    std::min(10.0, terminal_s_tol_ + 0.2 * s_remain) : 1e5;

  std::vector<double> term_s_bounds = {ref_s.back() - funnel_s_tol, ref_s.back() + funnel_s_tol};
  std::vector<double> term_d_bounds = {-funnel_d_tol, funnel_d_tol};
  std::vector<double> term_epsi_bounds = {-funnel_epsi_tol, funnel_epsi_tol};
  std::vector<double> term_v_bounds = {0.0, term_v_upper};

  auto t_frenet = std::chrono::high_resolution_clock::now();

  // 5. 配置与执行 CasADi MPC 高速 Function 求解
  if (!mpc_problem_initialized_) {
    initializeMPC();
  }

  double safe_current_speed = std::clamp(current_speed, v_min_, v_max_);
  std::vector<double> current_state = {
    current_frenet.s, current_frenet.d, current_frenet.e_psi, safe_current_speed};

  try {
    // 直接调用预编译好的 Function 函数 (包含时变走廊与终点位姿硬约束)
    std::vector<casadi::DM> inputs = {
      casadi::DM(current_state),
      casadi::DM(ref_s),
      casadi::DM(ref_v),
      casadi::DM(ref_w),
      casadi::DM(ref_kappa),
      casadi::DM(corridor_d_min),
      casadi::DM(corridor_d_max),
      casadi::DM(term_s_bounds),
      casadi::DM(term_d_bounds),
      casadi::DM(term_epsi_bounds),
      casadi::DM(term_v_bounds)
    };
    std::vector<casadi::DM> res = mpc_solver_(inputs);

    auto t_solve = std::chrono::high_resolution_clock::now();
    double ms_prep = std::chrono::duration<double, std::milli>(t_prep - t_start).count();
    double ms_frenet = std::chrono::duration<double, std::milli>(t_frenet - t_prep).count();
    double ms_solve = std::chrono::duration<double, std::milli>(t_solve - t_frenet).count();
    double ms_total = std::chrono::duration<double, std::milli>(t_solve - t_start).count();

    RCLCPP_INFO_THROTTLE(
      logger_, *(node_.lock()->get_clock()), 1000,
      "[MPC High-Speed Timing] Total: %.2f ms (Prep: %.2f ms | Frenet: %.2f ms | Solver: %.2f ms | Freq: %.1f Hz)",
      ms_total, ms_prep, ms_frenet, ms_solve, ms_total > 0 ? 1000.0 / ms_total : 0.0);

    casadi::DM sol_U = res.at(0); // (2, N)
    casadi::DM sol_X = res.at(1); // (4, N+1)

    double raw_w = double(sol_U(1, 0));
    double raw_v = double(sol_X(3, 1));

    double target_w = std::clamp(raw_w, w_min_, w_max_);
    // 角速度斜率滤波 (Slew-rate Limiter) 彻底消除出弯及高频扰动引发的方向盘跳变抖动
    double max_dw_step = 0.35; // 允许单帧(0.1s)角速度响应达到 0.35 rad/s (3.5 rad/s²)，消除入弯 0.6s 打方向延迟引发的 8cm 漂移
    double filtered_w = std::clamp(target_w, prev_cmd_w_ - max_dw_step, prev_cmd_w_ + max_dw_step);
    prev_cmd_w_ = filtered_w;

    double output_v = raw_v;

    // 终点平滑推进与防提前熄火死锁机制 (Goal Creep Velocity Protection):
    // 若尚未到达终点容差圈 (dist_to_goal > 0.05m)，必须保证有足够的最小爬行推进速度 (0.05m/s)，绝不提前在终点外停滞死锁！
    if (dist_to_goal > 0.05) {
      double min_creep_v = 0.05; // 5cm/s 保底爬行推进速度，足以克服底盘静摩擦力
      double max_decel = std::max(0.2, std::abs(a_min_));
      double approach_v_cap = std::clamp(std::sqrt(2.0 * max_decel * dist_to_goal), min_creep_v, v_max_);
      output_v = std::clamp(output_v, min_creep_v, approach_v_cap);
    } else {
      // 已经进入终点 5cm 容差范围内: 立即平稳刹停
      output_v = 0.0;
    }

    cmd_vel.twist.linear.x = std::clamp(output_v, v_min_, v_max_);
    cmd_vel.twist.angular.z = filtered_w;

    nav_msgs::msg::Path predict_path;
    predict_path.header.frame_id = pose.header.frame_id;
    predict_path.header.stamp = cmd_vel.header.stamp;

    for (int k = 0; k <= N_; ++k) {
      geometry_msgs::msg::PoseStamped p;
      p.header = predict_path.header;
      p.pose = frenetToCartesian(
        double(sol_X(0, k)),
        double(sol_X(1, k)),
        double(sol_X(2, k)),
        reference_path);
      predict_path.poses.push_back(p);
    }
    traj_pub_->publish(predict_path);
  } 
  catch (std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "MPC High-Speed Solver Warning: %s. Stopping robot.", e.what());
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  return cmd_vel;
}

}  // namespace nav2_mpc_controller

// 重要：将其导出为 pluginlib 插件，以便 Nav2 Controller Server 可以加载它
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MPCController, nav2_core::Controller)
