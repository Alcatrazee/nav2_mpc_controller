#ifndef NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <functional>
#include <limits>
#include <algorithm>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "casadi/casadi.hpp"
#include "nav2_util/node_utils.hpp"
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>
#include "std_msgs/msg/float64.hpp"
#include "trajectory_profiler.hpp"
#include "safe_corridor_generator.hpp"
#include <Eigen/Dense>


namespace nav2_mpc_controller
{

class MPCController : public nav2_core::Controller
{
public:
  MPCController() = default;
  ~MPCController() override = default;

  // Nav2 Controller 核心接口
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  const SafeCorridor & getSafeCorridor() const { return current_safe_corridor_; }

private:
  // ROS 和 Nav2 基础设施
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("MPCController")};
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr transformed_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr transformed_local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_plan_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr safe_corridor_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr lateral_error_pub_;
  nav_msgs::msg::Path global_plan_;

  // 安全走廊生成器与配置
  SafeCorridorConfig corridor_config_;
  std::unique_ptr<SafeCorridorGenerator> safe_corridor_generator_;
  SafeCorridor current_safe_corridor_;
  bool enable_safe_corridor_{true};
  double q_corridor_bound_{35.0};
  double q_corridor_center_{30.0};    // 窄通道宽度自适应居中势场权重
  double w_corridor_slack_{1000.0};
  double corridor_buffer_margin_{0.1};

  // 终点位姿硬约束与终端代价参数
  bool enable_terminal_constraint_{true};
  double goal_approach_dist_{0.6};    // 进入终点进近模式的距离阈值 [m]
  double terminal_s_tol_{0.05};        // 终点纵向位置硬约束容差 [m]
  double terminal_d_tol_{0.03};        // 终点横向偏差硬约束容差 [m]
  double terminal_epsi_tol_{0.05};     // 终点航向角误差硬约束容差 [rad]
  double terminal_v_tol_{0.01};        // 终点速度硬约束容差 [m/s]
  double q_s_terminal_{10.0};          // 终端纵向代价权重
  double q_d_terminal_{50.0};          // 终端横向代价权重
  double q_epsi_terminal_{20.0};       // 终端航向代价权重
  double r_v_terminal_{10.0};          // 终端速度代价权重

  // MPC 参数
  int N_;                // 预测步数
  double dt_;            // 预测基准步长 (秒)
  bool use_variable_dt_{false}; // 是否启用非均匀时间步长 (默认关闭，使用统一等步长)
  std::string solver_type_{"osqp"}; // 求解器类型 ("osqp" 或 "ipopt")
  std::vector<double> dt_vec_;    // 各步离散时间步长
  std::vector<double> dt_cumsum_; // 累计时间戳
  void updateDtVectors();

  double v_max_;         // 最大线速度
  double v_min_;         // 最小线速度
  double w_max_;         // 最大角速度
  double w_min_;         // 最小角速度
  double a_max_;         // 最大线加速度
  double a_min_;         // 最小线加速度
  
  // Frenet 状态与控制量权重
  double q_s_;
  double q_d_;
  double q_e_psi_;
  double r_v_;
  double r_w_;
  double r_a_;
  double q_prev_d_{8.0}; // 帧间轨迹一致性惩罚权重 (Inter-sample consistency)

  // 动态参数调整
  std::mutex param_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<TrajectoryProfiler> trajectory_profiler_;

  // 时间参数化核心函数
  std::vector<TrajectoryPoint> generateTimeParameterizedTrajectory(
    const nav_msgs::msg::Path & local_plan, 
    double current_speed);

  // 将速度参数化后的局部路径及MarkerArray发布到可视化话题的辅助函数
  void publishParameterizedTrajectory(
    const std::vector<TrajectoryPoint> & ref_points,
    const std_msgs::msg::Header & header);

  // CasADi MPC 优化器及预编译 Function 函数
  casadi::Function mpc_solver_;

  bool mpc_problem_initialized_{false};
  bool is_cold_start_{true};
  
  double prev_cmd_w_{0.0};

  // 用于存储上一帧的解以供热启动 (Warm Start)
  casadi::DM prev_sol_u_;
  casadi::DM prev_sol_x_;

  // 用于存储上一帧的 Frenet 状态解
  std::vector<double> prev_s_sol_;
  std::vector<double> prev_d_sol_;
  std::vector<double> prev_e_psi_sol_;
  std::vector<double> prev_v_sol_;
  std::vector<double> prev_w_sol_;
  std::vector<double> prev_a_sol_;

  struct FrenetState
  {
    double s;
    double d;
    double e_psi;
  };

  FrenetState cartesianToFrenet(
    double x, double y, double theta,
    const std::vector<TrajectoryPoint> & reference_path,
    bool is_reversing = false) const;

  geometry_msgs::msg::Pose frenetToCartesian(
    double s, double d, double e_psi,
    const std::vector<TrajectoryPoint> & reference_path,
    bool is_reversing = false) const;

  void initializeMPC();
  bool checkGoalCollision(double check_x, double check_y, double check_theta) const;

  // 角度归一化辅助函数
  double normalize_angle(double angle) const
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

// 坐标转换辅助函数声明
bool transformPlan(
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const nav_msgs::msg::Path & path,
  const std::string & target_frame,
  nav_msgs::msg::Path & output_path);

// 局部路径截取辅助函数声明
nav_msgs::msg::Path extractLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & transformed_plan,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros);

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
