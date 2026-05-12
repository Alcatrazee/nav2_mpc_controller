#include "mpc_controller.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"

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
  declare_parameter_if_not_declared(node, plugin_name_ + ".N", rclcpp::ParameterValue(15));
  declare_parameter_if_not_declared(node, plugin_name_ + ".dt", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_max", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".v_min", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".w_min", rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".a_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".a_min", rclcpp::ParameterValue(-1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_lat_accel", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".accel_ratio", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".decel_ratio", rclcpp::ParameterValue(0.3));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_x", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_y", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_theta", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_v", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_w", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_a", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_turn_radius", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".min_turning_radius", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".cmd_vel_filter_alpha", rclcpp::ParameterValue(0.5));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_local_plan", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_base_lookahead_dist", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_max_lookahead_dist", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lookahead_time", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lat_range", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lat_step", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lon_ratios", rclcpp::ParameterValue(std::vector<double>{0.6, 0.8, 1.0}));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_obs", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_lat", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_smooth", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_lat_change", rclcpp::ParameterValue(1.5));

  node->get_parameter(plugin_name_ + ".N", N_);
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".v_max", v_max_);
  node->get_parameter(plugin_name_ + ".v_min", v_min_);
  node->get_parameter(plugin_name_ + ".w_max", w_max_);
  node->get_parameter(plugin_name_ + ".w_min", w_min_);
  node->get_parameter(plugin_name_ + ".a_max", a_max_);
  node->get_parameter(plugin_name_ + ".a_min", a_min_);
  node->get_parameter(plugin_name_ + ".max_lat_accel", max_lat_accel_);
  node->get_parameter(plugin_name_ + ".accel_ratio", accel_ratio_);
  node->get_parameter(plugin_name_ + ".decel_ratio", decel_ratio_);
  node->get_parameter(plugin_name_ + ".q_x", q_x_);
  node->get_parameter(plugin_name_ + ".q_y", q_y_);
  node->get_parameter(plugin_name_ + ".q_theta", q_theta_);
  node->get_parameter(plugin_name_ + ".r_v", r_v_);
  node->get_parameter(plugin_name_ + ".r_w", r_w_);
  node->get_parameter(plugin_name_ + ".r_a", r_a_);
  node->get_parameter(plugin_name_ + ".weight_turn_radius", weight_turn_radius_);
  node->get_parameter(plugin_name_ + ".min_turning_radius", min_turning_radius_);
  node->get_parameter(plugin_name_ + ".cmd_vel_filter_alpha", cmd_vel_filter_alpha_);
  
  node->get_parameter(plugin_name_ + ".use_local_plan", use_local_plan_);
  node->get_parameter(plugin_name_ + ".lattice_base_lookahead_dist", lattice_base_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lattice_max_lookahead_dist", lattice_max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lattice_lookahead_time", lattice_lookahead_time_);
  node->get_parameter(plugin_name_ + ".lattice_lat_range", lattice_lat_range_);
  node->get_parameter(plugin_name_ + ".lattice_lat_step", lattice_lat_step_);
  node->get_parameter(plugin_name_ + ".lattice_lon_ratios", lattice_lon_ratios_);
  node->get_parameter(plugin_name_ + ".weight_obs", weight_obs_);
  node->get_parameter(plugin_name_ + ".weight_lat", weight_lat_);
  node->get_parameter(plugin_name_ + ".weight_smooth", weight_smooth_);
  node->get_parameter(plugin_name_ + ".weight_lat_change", weight_lat_change_);

  // 注册动态参数回调
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&MPCController::dynamicParametersCallback, this, std::placeholders::_1));

  // 创建预测轨迹的发布器
  traj_pub_ = node->create_publisher<nav_msgs::msg::Path>("predict_trajectory", 10);
  transformed_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_global_plan", 10);
  transformed_local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("transformed_local_plan", 10);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 10);
  local_plan_marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("local_plan_markers", 10);
  lattice_candidates_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("lattice_candidates", 10);

  ProfilerConfig profiler_cfg;
  profiler_cfg.max_velocity = v_max_;
  // 使用比例参数控制规划器，调小 decel_ratio 可以让参考速度提早下降，实现平滑早刹车
  profiler_cfg.max_a = a_max_ * accel_ratio_;
  profiler_cfg.min_a = a_min_ * decel_ratio_;
  profiler_cfg.max_lat_accel = max_lat_accel_;
  trajectory_profiler_ = std::make_unique<TrajectoryProfiler>(profiler_cfg);

  RCLCPP_INFO(logger_, "MPC Controller Configured.");
}

void MPCController::cleanup()
{
  dyn_params_handler_.reset();
  traj_pub_.reset();
  transformed_plan_pub_.reset();
  transformed_local_plan_pub_.reset();
  local_plan_pub_.reset();
  local_plan_marker_pub_.reset();
  lattice_candidates_pub_.reset();
  RCLCPP_INFO(logger_, "MPC Controller Cleaned Up.");
}

void MPCController::activate()
{
  traj_pub_->on_activate();
  transformed_plan_pub_->on_activate();
  transformed_local_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  local_plan_marker_pub_->on_activate();
  lattice_candidates_pub_->on_activate();
  RCLCPP_INFO(logger_, "MPC Controller Activated.");
}

void MPCController::deactivate()
{
  traj_pub_->on_deactivate();
  transformed_plan_pub_->on_deactivate();
  transformed_local_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  local_plan_marker_pub_->on_deactivate();
  lattice_candidates_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "MPC Controller Deactivated.");
}

void MPCController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  prev_local_plan_.poses.clear(); // 收到新的全局路径时清除旧缓存
  prev_best_lat_ = 0.0;           // 重置横向偏移缓存，重新起步时不带历史惯性
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
      if (trajectory_profiler_) {
        ProfilerConfig cfg;
        cfg.max_velocity = v_max_;
        trajectory_profiler_ = std::make_unique<TrajectoryProfiler>(cfg);
      }
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
    } else if (name == plugin_name_ + ".max_lat_accel") {
      max_lat_accel_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".accel_ratio") {
      accel_ratio_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".decel_ratio") {
      decel_ratio_ = parameter.as_double();
      update_profiler = true;
    } else if (name == plugin_name_ + ".q_x") {
      q_x_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_y") {
      q_y_ = parameter.as_double();
    } else if (name == plugin_name_ + ".q_theta") {
      q_theta_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_v") {
      r_v_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_w") {
      r_w_ = parameter.as_double();
    } else if (name == plugin_name_ + ".r_a") {
      r_a_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_turn_radius") {
      weight_turn_radius_ = parameter.as_double();
    } else if (name == plugin_name_ + ".min_turning_radius") {
      min_turning_radius_ = parameter.as_double();
    } else if (name == plugin_name_ + ".cmd_vel_filter_alpha") {
      cmd_vel_filter_alpha_ = parameter.as_double();
    } else if (name == plugin_name_ + ".use_local_plan") {
      use_local_plan_ = parameter.as_bool();
    } else if (name == plugin_name_ + ".lattice_base_lookahead_dist") {
      lattice_base_lookahead_dist_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_max_lookahead_dist") {
      lattice_max_lookahead_dist_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lookahead_time") {
      lattice_lookahead_time_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lat_range") {
      lattice_lat_range_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lat_step") {
      lattice_lat_step_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lon_ratios") {
      auto new_ratios = parameter.as_double_array();
      if (new_ratios.empty()) {
        RCLCPP_WARN(logger_, "lattice_lon_ratios cannot be empty! Rejecting update.");
        result.successful = false;
        result.reason = "Empty lattice_lon_ratios array";
      } else {
        std::sort(new_ratios.begin(), new_ratios.end()); // 必须保证递增，否则 S_max 提取错误
        lattice_lon_ratios_ = new_ratios;
      }
    } else if (name == plugin_name_ + ".weight_obs") {
      weight_obs_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_lat") {
      weight_lat_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_smooth") {
      weight_smooth_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_lat_change") {
      weight_lat_change_ = parameter.as_double();
    }
    RCLCPP_INFO(
        logger_, "Parameter %s updated to: %s",
        name.c_str(), parameter.value_to_string().c_str());
  }

  if (update_profiler && trajectory_profiler_) {
    ProfilerConfig cfg;
    cfg.max_velocity = v_max_;
    cfg.max_a = a_max_ * accel_ratio_;
    cfg.min_a = a_min_ * decel_ratio_;
    cfg.max_lat_accel = max_lat_accel_;
    trajectory_profiler_ = std::make_unique<TrajectoryProfiler>(cfg);
  }
  
  mpc_problem_initialized_ = false; // 当任何参数改变时，标记重新初始化 MPC 问题
  return result;
}


void MPCController::initializeMPC()
{
  opti_ = casadi::Opti(); // 重置优化问题

  X_ = opti_.variable(4, N_ + 1); // 状态 [x, y, theta, v]
  U_ = opti_.variable(2, N_);     // 控制量 [a, w]

  // 声明为 Parameter (而不是常量)，使其允许在循环求解阶段动态更新而无需重构树
  X0_param_ = opti_.parameter(4);
  Ref_x_param_ = opti_.parameter(N_);
  Ref_y_param_ = opti_.parameter(N_);
  Ref_theta_param_ = opti_.parameter(N_);
  Ref_v_param_ = opti_.parameter(N_);
  Ref_w_param_ = opti_.parameter(N_);

  // 初始状态约束 (X0 = current_state)
  opti_.subject_to(X_(0, 0) == X0_param_(0));
  opti_.subject_to(X_(1, 0) == X0_param_(1));
  opti_.subject_to(X_(2, 0) == X0_param_(2));
  opti_.subject_to(X_(3, 0) == X0_param_(3));

  casadi::MX cost = 0;

  // 遍历预测视野，建立差分运动学约束和代价函数
  for (int k = 0; k < N_; ++k) {
    // 差速运动学 (欧拉前向离散化)
    casadi::MX x_next = X_(0, k) + X_(3, k) * cos(X_(2, k)) * dt_;
    casadi::MX y_next = X_(1, k) + X_(3, k) * sin(X_(2, k)) * dt_;
    casadi::MX theta_next = X_(2, k) + U_(1, k) * dt_; // w 作为直接控制量输入
    casadi::MX v_next = X_(3, k) + U_(0, k) * dt_;
    
    opti_.subject_to(X_(0, k+1) == x_next);
    opti_.subject_to(X_(1, k+1) == y_next);
    opti_.subject_to(X_(2, k+1) == theta_next);
    opti_.subject_to(X_(3, k+1) == v_next);

    // 边界约束
    opti_.subject_to(opti_.bounded(v_min_, X_(3, k+1), v_max_));
    opti_.subject_to(opti_.bounded(w_min_, U_(1, k), w_max_));
    opti_.subject_to(opti_.bounded(a_min_, U_(0, k), a_max_));

    // 代价函数：位置追踪误差 + 速度追踪误差 + a加速平滑
    cost += q_x_ * pow(X_(0, k+1) - Ref_x_param_(k), 2);
    cost += q_y_ * pow(X_(1, k+1) - Ref_y_param_(k), 2);
    cost += q_theta_ * pow(X_(2, k+1) - Ref_theta_param_(k), 2);
    cost += r_v_ * pow(X_(3, k+1) - Ref_v_param_(k), 2);
    cost += r_w_ * pow(U_(1, k) - Ref_w_param_(k), 2);
    cost += r_a_ * pow(U_(0, k), 2);
    
    // 软约束优化：惩罚侧向向心加速度 (v * w)
    // 彻底修复原先 w^2/v^2 导致分母为 v 从而恶意奖励小车超速冲线的问题。
    // 现在，当遇到弯道或需要调整航向 (w较大) 时，小车会为了降低代价而天然主动减速！
    cost += weight_turn_radius_ * pow(X_(3, k+1) * U_(1, k), 2);

    // 硬约束：最小转弯半径
    if (min_turning_radius_ > 0.001) {
      opti_.subject_to(pow(X_(3, k+1), 2) >= pow(min_turning_radius_ * U_(1, k), 2));
    }
  }

  opti_.minimize(cost);

  // 配置并调用 IPOPT 求解器
  casadi::Dict solver_opts;
  solver_opts["ipopt.print_level"] = 0;    // 关闭日志以保证终端清爽
  solver_opts["ipopt.sb"] = "yes";
  solver_opts["print_time"] = 0;
  opti_.solver("ipopt", solver_opts);

  mpc_problem_initialized_ = true;
  is_cold_start_ = true; // 初始化后第一次必然是冷启动
  RCLCPP_INFO(logger_, "MPC CasADi problem (re)initialized with N=%d", N_);
}


geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(param_mutex_);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = rclcpp::Clock().now();

  if (global_plan_.poses.empty()) {
    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
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

  // 动态更新 Lattice 前瞻距离: 基础距离 + 速度 * 时间增益，且不超过最大前瞻距离
  double current_speed = velocity.linear.x;
  lattice_lookahead_dist_ = std::clamp(
    lattice_base_lookahead_dist_ + std::abs(current_speed) * lattice_lookahead_time_,
    lattice_base_lookahead_dist_,
    lattice_max_lookahead_dist_
  );

  // 2. 截取局部规划基准路径 (裁剪掉最近点之前的点，并限制在代价地图范围内)
  nav_msgs::msg::Path base_local_plan = extractLocalPlan(pose, transformed_plan, costmap_ros_);
  
  double dist_to_goal = std::numeric_limits<double>::max();
  if (!transformed_plan.poses.empty()) {
    dist_to_goal = std::hypot(current_x - transformed_plan.poses.back().pose.position.x,
                              current_y - transformed_plan.poses.back().pose.position.y);
  }

  // 判断当前截取出的局部路径是否包含了全局终点
  bool is_plan_contain_goal = false;
  if (!base_local_plan.poses.empty() && !transformed_plan.poses.empty()) {
    double dx = base_local_plan.poses.back().pose.position.x - transformed_plan.poses.back().pose.position.x;
    double dy = base_local_plan.poses.back().pose.position.y - transformed_plan.poses.back().pose.position.y;
    if (std::hypot(dx, dy) < 0.1) {
      is_plan_contain_goal = true;
    }
  }

  nav_msgs::msg::Path tracking_plan;

  if (!use_local_plan_) {
    // 屏蔽 Lattice Planner，直接使用原始局部截取路径
    tracking_plan = base_local_plan;
  } else {
    // 3. 使用 Lattice Planner 采样并评估生成无碰的最优局部路径
    nav_msgs::msg::Path local_plan;
    if (dist_to_goal <= 0.1 && !prev_local_plan_.poses.empty()) {
      // 在最后的 0.1m 处保留在此之前的局部路径不重新规划，防止目标点前路径消失导致停车
      // 关键修正：必须从当前位姿截取缓存的上一帧路径，防止起始参考点落后于车体导致刹车停滞
      local_plan = extractLocalPlan(pose, prev_local_plan_, costmap_ros_);
      local_plan.header = pose.header;
    } else {
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_ros_->getCostmap());
      local_plan = generateLatticePlan(pose, base_local_plan, checker, is_plan_contain_goal);
      if (!local_plan.poses.empty()) {
        prev_local_plan_ = local_plan; // 备份成功生成的局部路径
      }
    }
    tracking_plan = local_plan;
  }
  
  // 发布用于追踪的最终局部路径
  transformed_local_plan_pub_->publish(tracking_plan);

  if (tracking_plan.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Tracking plan is empty! Stopping robot.");
    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }
  

  // 一键生成带曲率平滑的时间参数化轨迹及控制参考点
  auto ref_points = generateTimeParameterizedTrajectory(tracking_plan, velocity.linear.x);

  if (ref_points.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Local plan is empty or failed to generate parameterized trajectory!");
    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
    return cmd_vel;
  }

  // 将发布可视化话题的过程封装进辅助函数中
  std_msgs::msg::Header viz_header;
  viz_header.frame_id = pose.header.frame_id;
  viz_header.stamp = cmd_vel.header.stamp;
  publishParameterizedTrajectory(ref_points, viz_header);

  // 从时间参数化轨迹中按时间采样未来 N 步的参考点
  std::vector<double> ref_x(N_, 0.0), ref_y(N_, 0.0), ref_theta(N_, 0.0);
  std::vector<double> ref_v(N_, 0.0), ref_w(N_, 0.0);
  
  for (int k = 0; k < N_; ++k) {
    ref_x[k] = ref_points[k].x;
    ref_y[k] = ref_points[k].y;
    ref_v[k] = ref_points[k].v;
    // 根据角速度物理公式：角速度 = 线速度 × 曲率
    ref_w[k] = ref_points[k].v * ref_points[k].kappa;
    
    // 展开角度差异以避免 2*PI 跳变导致的自旋
    double diff = normalize_angle(ref_points[k].theta - current_theta);
    ref_theta[k] = current_theta + diff; 
  }

  // 3. 配置 CasADi MPC 优化器
  if (!mpc_problem_initialized_) {
    initializeMPC();
  }

  // 传入实时参数 (当前位姿与参考轨迹)
  std::vector<double> current_state = {current_x, current_y, current_theta, velocity.linear.x};
  opti_.set_value(X0_param_, current_state);
  opti_.set_value(Ref_x_param_, ref_x);
  opti_.set_value(Ref_y_param_, ref_y);
  opti_.set_value(Ref_theta_param_, ref_theta);
  opti_.set_value(Ref_v_param_, ref_v);
  opti_.set_value(Ref_w_param_, ref_w);

  // 配置初值 (Warm Start & Cold Start)
  // 利用 std::vector<std::vector<double>> 天然构建二维矩阵
  std::vector<std::vector<double>> init_X(4, std::vector<double>(N_ + 1, 0.0));
  std::vector<std::vector<double>> init_U(2, std::vector<double>(N_, 0.0));

  if (!is_cold_start_ && prev_x_sol_.size() == static_cast<size_t>(N_ + 1)) {
    // 热启动 (Warm Start) - 将上一帧的轨迹向前平移一步作为初始猜想
    for (int k = 0; k < N_; ++k) {
      init_X[0][k] = prev_x_sol_[k+1];
      init_X[1][k] = prev_y_sol_[k+1];
      init_X[2][k] = prev_theta_sol_[k+1];
      init_X[3][k] = prev_v_sol_[k+1];
      
      if (k < N_ - 1) {
        init_U[0][k] = prev_a_sol_[k+1];
        init_U[1][k] = prev_w_sol_[k+1];
      } else {
        init_U[0][k] = prev_a_sol_[k];
        init_U[1][k] = prev_w_sol_[k];
      }
    }
    init_X[0][N_] = prev_x_sol_[N_];
    init_X[1][N_] = prev_y_sol_[N_];
    init_X[2][N_] = prev_theta_sol_[N_];
    init_X[3][N_] = prev_v_sol_[N_];
  } else {
    // 冷启动 (Cold Start) - 赋予当前值作为所有预测点的初始猜想
    for (int k = 0; k <= N_; ++k) {
      init_X[0][k] = current_x;
      init_X[1][k] = current_y;
      init_X[2][k] = current_theta;
      init_X[3][k] = velocity.linear.x;
    }
    for (int k = 0; k < N_; ++k) {
      init_U[0][k] = 0.0;
      init_U[1][k] = velocity.angular.z;
    }
  }
  
  // CasADi C++ API 支持直接将二维 vector 转化为具有正确维度的 DM 矩阵
  opti_.set_initial(X_, casadi::DM(init_X));
  opti_.set_initial(U_, casadi::DM(init_U));

  try {
    casadi::OptiSol sol = opti_.solve();
    
    // 提取计算出的第一步最优控制指令 U[:, 0]
    casadi::Slice all;
    prev_a_sol_ = static_cast<std::vector<double>>(sol.value(U_(0, all)));
    prev_w_sol_ = static_cast<std::vector<double>>(sol.value(U_(1, all)));

    // 提取最优状态预测轨迹并发布
    prev_x_sol_ = static_cast<std::vector<double>>(sol.value(X_(0, all)));
    prev_y_sol_ = static_cast<std::vector<double>>(sol.value(X_(1, all)));
    prev_theta_sol_ = static_cast<std::vector<double>>(sol.value(X_(2, all)));
    prev_v_sol_ = static_cast<std::vector<double>>(sol.value(X_(3, all)));

    // 提取原始控制指令 (线速度取一步积分后值，角速度直接取当前控制输入)
    double raw_v = prev_v_sol_[1];
    double raw_w = prev_w_sol_[0];

    // 一阶低通滤波 (Exponential Moving Average) 柔化硬件输出抖动
    last_cmd_v_ = cmd_vel_filter_alpha_ * raw_v + (1.0 - cmd_vel_filter_alpha_) * last_cmd_v_;
    last_cmd_w_ = cmd_vel_filter_alpha_ * raw_w + (1.0 - cmd_vel_filter_alpha_) * last_cmd_w_;
    cmd_vel.twist.linear.x = last_cmd_v_;
    cmd_vel.twist.angular.z = last_cmd_w_;

    is_cold_start_ = false; // 求解成功，下一帧启用热启动

    nav_msgs::msg::Path predict_path;
    predict_path.header.frame_id = pose.header.frame_id;
    predict_path.header.stamp = cmd_vel.header.stamp;

    for (int k = 0; k <= N_; ++k) {
      geometry_msgs::msg::PoseStamped p;
      p.header = predict_path.header;
      p.pose.position.x = prev_x_sol_[k];
      p.pose.position.y = prev_y_sol_[k];
      
      tf2::Quaternion q;
      q.setRPY(0, 0, prev_theta_sol_[k]);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      predict_path.poses.push_back(p);
    }
    RCLCPP_INFO(logger_,"size: %zu",predict_path.poses.size());
    traj_pub_->publish(predict_path);
  } 
  catch (std::exception & e) {
    // 求解失败时的回退机制：停车
    is_cold_start_ = true; // 求解失败，下一帧回退到冷启动
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "MPC Failed: %s. Stopping robot.", e.what());
    last_cmd_v_ = 0.0;
    last_cmd_w_ = 0.0;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  return cmd_vel;
}

}  // namespace nav2_mpc_controller

// 重要：将其导出为 pluginlib 插件，以便 Nav2 Controller Server 可以加载它
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MPCController, nav2_core::Controller)