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
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_x", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_y", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_theta", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_v", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_w", rclcpp::ParameterValue(0.1));
  
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lookahead_dist", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lat_range", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(node, plugin_name_ + ".lattice_lat_step", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lattice_lon_ratios", rclcpp::ParameterValue(std::vector<double>{0.6, 0.8, 1.0}));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_obs", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_lat", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".weight_smooth", rclcpp::ParameterValue(0.5));

  node->get_parameter(plugin_name_ + ".N", N_);
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".v_max", v_max_);
  node->get_parameter(plugin_name_ + ".v_min", v_min_);
  node->get_parameter(plugin_name_ + ".w_max", w_max_);
  node->get_parameter(plugin_name_ + ".w_min", w_min_);
  node->get_parameter(plugin_name_ + ".q_x", q_x_);
  node->get_parameter(plugin_name_ + ".q_y", q_y_);
  node->get_parameter(plugin_name_ + ".q_theta", q_theta_);
  node->get_parameter(plugin_name_ + ".r_v", r_v_);
  node->get_parameter(plugin_name_ + ".r_w", r_w_);
  
  node->get_parameter(plugin_name_ + ".lattice_lookahead_dist", lattice_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lattice_lat_range", lattice_lat_range_);
  node->get_parameter(plugin_name_ + ".lattice_lat_step", lattice_lat_step_);
  node->get_parameter(plugin_name_ + ".lattice_lon_ratios", lattice_lon_ratios_);
  node->get_parameter(plugin_name_ + ".weight_obs", weight_obs_);
  node->get_parameter(plugin_name_ + ".weight_lat", weight_lat_);
  node->get_parameter(plugin_name_ + ".weight_smooth", weight_smooth_);

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
    } else if (name == plugin_name_ + ".v_min") {
      v_min_ = parameter.as_double();
    } else if (name == plugin_name_ + ".w_max") {
      w_max_ = parameter.as_double();
    } else if (name == plugin_name_ + ".w_min") {
      w_min_ = parameter.as_double();
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
    } else if (name == plugin_name_ + ".lattice_lookahead_dist") {
      lattice_lookahead_dist_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lat_range") {
      lattice_lat_range_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lat_step") {
      lattice_lat_step_ = parameter.as_double();
    } else if (name == plugin_name_ + ".lattice_lon_ratios") {
      lattice_lon_ratios_ = parameter.as_double_array();
    } else if (name == plugin_name_ + ".weight_obs") {
      weight_obs_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_lat") {
      weight_lat_ = parameter.as_double();
    } else if (name == plugin_name_ + ".weight_smooth") {
      weight_smooth_ = parameter.as_double();
    }
    RCLCPP_INFO(
        logger_, "Parameter %s updated to: %s",
        name.c_str(), parameter.value_to_string().c_str());
  }
  return result;
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

  // 2. 截取局部规划基准路径 (裁剪掉最近点之前的点，并限制在代价地图范围内)
  nav_msgs::msg::Path base_local_plan = extractLocalPlan(pose, transformed_plan, costmap_ros_);
  
  // 判断当前截取出的局部路径是否包含了全局终点
  bool is_plan_contain_goal = false;
  if (!base_local_plan.poses.empty() && !transformed_plan.poses.empty()) {
    double dx = base_local_plan.poses.back().pose.position.x - transformed_plan.poses.back().pose.position.x;
    double dy = base_local_plan.poses.back().pose.position.y - transformed_plan.poses.back().pose.position.y;
    if (std::hypot(dx, dy) < 0.1) {
      is_plan_contain_goal = true;
    }
  }

  // 3. 使用 Lattice Planner 采样并评估生成无碰的最优局部路径
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_ros_->getCostmap());
  nav_msgs::msg::Path local_plan = generateLatticePlan(pose, base_local_plan, checker, is_plan_contain_goal);
  
  // 发布截取后的局部路径
  transformed_local_plan_pub_->publish(local_plan);

  // lattice planner plan
  if (local_plan.poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Local plan is empty! Stopping robot.");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }
  

  // 一键生成带曲率平滑的时间参数化轨迹及控制参考点
  auto ref_points = generateTimeParameterizedTrajectory(local_plan, velocity.linear.x);

  if (ref_points.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "Local plan is empty or failed to generate parameterized trajectory!");
    return cmd_vel;
  }

  // 将发布可视化话题的过程封装进辅助函数中
  std_msgs::msg::Header viz_header;
  viz_header.frame_id = pose.header.frame_id;
  viz_header.stamp = cmd_vel.header.stamp;
  publishParameterizedTrajectory(ref_points, viz_header);

  // 从时间参数化轨迹中按时间采样未来 N 步的参考点
  std::vector<double> ref_x(N_, 0.0), ref_y(N_, 0.0), ref_theta(N_, 0.0);
  
  for (int k = 0; k < N_; ++k) {
    ref_x[k] = ref_points[k].x;
    ref_y[k] = ref_points[k].y;
    
    // 展开角度差异以避免 2*PI 跳变导致的自旋
    double diff = normalize_angle(ref_points[k].theta - current_theta);
    ref_theta[k] = current_theta + diff; 
  }

  // 3. 构建 CasADi 优化问题
  // 注意：出于演示简单目的，这里在每个周期重新创建 Opti。
  // 生产环境中，最好将其创建为类的成员变量并使用 opti.set_value() 传递参数，以节省初始化开销。
  casadi::Opti opti;

  casadi::MX X = opti.variable(3, N_ + 1); // 状态 [x, y, theta]
  casadi::MX U = opti.variable(2, N_);     // 控制量 [v, w]

  // 初始状态约束 (X0 = current_state)
  opti.subject_to(X(0, 0) == current_x);
  opti.subject_to(X(1, 0) == current_y);
  opti.subject_to(X(2, 0) == current_theta);

  casadi::MX cost = 0;

  // 遍历预测视野，建立差分运动学约束和代价函数
  for (int k = 0; k < N_; ++k) {
    // 差速运动学 (欧拉前向离散化)
    casadi::MX x_next = X(0, k) + U(0, k) * cos(X(2, k)) * dt_;
    casadi::MX y_next = X(1, k) + U(0, k) * sin(X(2, k)) * dt_;
    casadi::MX theta_next = X(2, k) + U(1, k) * dt_;
    
    opti.subject_to(X(0, k+1) == x_next);
    opti.subject_to(X(1, k+1) == y_next);
    opti.subject_to(X(2, k+1) == theta_next);

    // 控制量边界约束
    opti.subject_to(opti.bounded(v_min_, U(0, k), v_max_));
    opti.subject_to(opti.bounded(w_min_, U(1, k), w_max_));

    // 代价函数：位置追踪误差 + 控制量平滑
    cost += q_x_ * pow(X(0, k+1) - ref_x[k], 2);
    cost += q_y_ * pow(X(1, k+1) - ref_y[k], 2);
    cost += q_theta_ * pow(X(2, k+1) - ref_theta[k], 2);
    cost += r_v_ * pow(U(0, k), 2);
    cost += r_w_ * pow(U(1, k), 2);
  }

  opti.minimize(cost);

  // 4. 配置并调用 IPOPT 求解器
  casadi::Dict solver_opts;
  solver_opts["ipopt.print_level"] = 0;    // 关闭日志以保证终端清爽
  solver_opts["ipopt.sb"] = "yes";
  solver_opts["print_time"] = 0;
  opti.solver("ipopt", solver_opts);

  try {
    casadi::OptiSol sol = opti.solve();
    
    // 提取计算出的第一步最优控制指令 U[:, 0]
    casadi::Slice all;
    std::vector<double> u_opt = static_cast<std::vector<double>>(sol.value(U(all, 0)));
    
    cmd_vel.twist.linear.x = u_opt[0];
    cmd_vel.twist.angular.z = u_opt[1];

    // 提取最优状态预测轨迹并发布
    std::vector<double> x_opt = static_cast<std::vector<double>>(sol.value(X(0, all)));
    std::vector<double> y_opt = static_cast<std::vector<double>>(sol.value(X(1, all)));
    std::vector<double> theta_opt = static_cast<std::vector<double>>(sol.value(X(2, all)));

    nav_msgs::msg::Path predict_path;
    predict_path.header.frame_id = pose.header.frame_id;
    predict_path.header.stamp = cmd_vel.header.stamp;

    for (int k = 0; k <= N_; ++k) {
      geometry_msgs::msg::PoseStamped p;
      p.header = predict_path.header;
      p.pose.position.x = x_opt[k];
      p.pose.position.y = y_opt[k];
      
      tf2::Quaternion q;
      q.setRPY(0, 0, theta_opt[k]);
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
    RCLCPP_WARN_THROTTLE(logger_, *(node_.lock()->get_clock()), 1000, 
                         "MPC Failed: %s. Stopping robot.", e.what());
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  return cmd_vel;
}

}  // namespace nav2_mpc_controller

// 重要：将其导出为 pluginlib 插件，以便 Nav2 Controller Server 可以加载它
PLUGINLIB_EXPORT_CLASS(nav2_mpc_controller::MPCController, nav2_core::Controller)