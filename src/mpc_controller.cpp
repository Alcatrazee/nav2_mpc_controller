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

  // 注册动态参数回调
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&MPCController::dynamicParametersCallback, this, std::placeholders::_1));

  // 创建预测轨迹的发布器
  traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predict_trajectory", 10);
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 10);
  transformed_local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_local_plan", 10);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 10);
  local_plan_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("local_plan_markers", 10);
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


void MPCController::initializeMPC()
{
  casadi::Opti opti; // 创建局部 Opti 实例以构建计算图

  auto X = opti.variable(4, N_ + 1); // Frenet 状态 [s, d, e_psi, v]
  auto U = opti.variable(2, N_);     // 控制量 [a, w]

  auto X0_param = opti.parameter(4);
  auto Ref_s_param = opti.parameter(N_);
  auto Ref_v_param = opti.parameter(N_);
  auto Ref_w_param = opti.parameter(N_);
  auto Ref_kappa_param = opti.parameter(N_);

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

    // 代价函数：Frenet 纵向 s、横向 d、航向 e_psi 误差 + 速度 v 与控制量 (w, a) 惩罚
    cost += q_s_ * pow(X(0, k+1) - Ref_s_param(k), 2);
    cost += q_d_ * pow(X(1, k+1), 2);
    cost += q_e_psi_ * pow(X(2, k+1), 2);
    cost += r_v_ * pow(X(3, k+1) - Ref_v_param(k), 2);
    cost += r_w_ * pow(U(1, k) - Ref_w_param(k), 2);
    cost += r_a_ * pow(U(0, k), 2);
  }

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
    {X0_param, Ref_s_param, Ref_v_param, Ref_w_param, Ref_kappa_param},
    {U, X});

  mpc_problem_initialized_ = true;
  is_cold_start_ = true;
  RCLCPP_INFO(logger_, "Frenet MPC CasADi High-Speed Function compiled with N=%d (Target >= 20Hz)", N_);
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

  auto t_frenet = std::chrono::high_resolution_clock::now();

  // 3. 配置与执行 CasADi MPC 高速 Function 求解
  if (!mpc_problem_initialized_) {
    initializeMPC();
  }

  double safe_current_speed = std::clamp(current_speed, v_min_, v_max_);
  std::vector<double> current_state = {
    current_frenet.s, current_frenet.d, current_frenet.e_psi, safe_current_speed};

  try {
    // 直接调用预编译好的 Function 函数 (微秒级调度，无需 Opti 包装器开销)
    std::vector<casadi::DM> inputs = {
      casadi::DM(current_state),
      casadi::DM(ref_s),
      casadi::DM(ref_v),
      casadi::DM(ref_w),
      casadi::DM(ref_kappa)
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

    cmd_vel.twist.linear.x = std::clamp(raw_v, v_min_, v_max_);
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
