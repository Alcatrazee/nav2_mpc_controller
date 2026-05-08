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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "trajectory_profiler.hpp"
#include <Eigen/Dense>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"


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
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_candidates_pub_;
  nav_msgs::msg::Path global_plan_;

  // MPC 参数
  int N_;                // 预测步数
  double dt_;            // 预测步长 (秒)
  double v_max_;         // 最大线速度
  double v_min_;         // 最小线速度
  double w_max_;         // 最大角速度
  double w_min_;         // 最小角速度
  
  // 权重参数
  double q_x_;
  double q_y_;
  double q_theta_;
  double r_v_;
  double r_w_;
  
  // Lattice Planner 参数
  double lattice_lookahead_dist_;
  double lattice_lat_range_;
  double lattice_lat_step_;
  std::vector<double> lattice_lon_ratios_;
  double weight_obs_;
  double weight_lat_;
  double weight_smooth_;

  // 动态参数调整
  std::mutex param_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  std::unique_ptr<TrajectoryProfiler> trajectory_profiler_;

  // 新增的时间参数化核心函数
  std::vector<TrajectoryPoint> generateTimeParameterizedTrajectory(
    const nav_msgs::msg::Path & local_plan, 
    double current_speed);

  // 新增的最简 Lattice Planner，用于生成候选曲线并结合障碍物评价选出最优轨迹
  nav_msgs::msg::Path generateLatticePlan(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & base_plan,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & checker,
    bool is_plan_contain_goal);

  // 将速度参数化后的局部路径及MarkerArray发布到可视化话题的辅助函数
  void publishParameterizedTrajectory(
    const std::vector<TrajectoryPoint> & ref_points,
    const std_msgs::msg::Header & header);


  // 角度归一化辅助函数
  double normalize_angle(double angle)
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