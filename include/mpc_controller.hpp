#ifndef NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "casadi/casadi.hpp"
#include "nav2_util/node_utils.hpp"
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>


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

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
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

  // 角度归一化辅助函数
  double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

}  // namespace nav2_mpc_controller

#endif  // NAV2_MPC_CONTROLLER__MPC_CONTROLLER_HPP_