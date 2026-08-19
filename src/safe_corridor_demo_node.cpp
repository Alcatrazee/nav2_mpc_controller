#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "safe_corridor_generator.hpp"

namespace nav2_mpc_controller
{

class SafeCorridorDemoNode : public rclcpp::Node
{
public:
  SafeCorridorDemoNode()
  : Node("safe_corridor_demo_node")
  {
    // 声明参数
    this->declare_parameter<double>("default_left_width", 0.8);
    this->declare_parameter<double>("default_right_width", 0.8);
    this->declare_parameter<double>("min_corridor_width", 0.15);
    this->declare_parameter<double>("curvature_safety_factor", 0.85);
    this->declare_parameter<double>("rib_safety_margin", 0.05);
    this->declare_parameter<double>("max_lateral_rate", 0.4);
    this->declare_parameter<bool>("avoid_hairpin_self_intersection", true);
    this->declare_parameter<double>("hairpin_radius", 1.2); // 掉头弯道半径 (米)
    this->declare_parameter<double>("straight_length", 4.0); // 直道长度 (米)

    updateConfigFromParams();

    // 发布器
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("safe_corridor_demo/reference_path", 10);
    left_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("safe_corridor_demo/left_boundary", 10);
    right_boundary_pub_ = this->create_publisher<nav_msgs::msg::Path>("safe_corridor_demo/right_boundary", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("safe_corridor_demo/markers", 10);

    // 动态参数更新
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : params) {
          if (param.get_name() == "default_left_width") {
            config_.default_left_width = param.as_double();
          } else if (param.get_name() == "default_right_width") {
            config_.default_right_width = param.as_double();
          } else if (param.get_name() == "min_corridor_width") {
            config_.min_corridor_width = param.as_double();
          } else if (param.get_name() == "curvature_safety_factor") {
            config_.curvature_safety_factor = param.as_double();
          } else if (param.get_name() == "avoid_hairpin_self_intersection") {
            config_.avoid_hairpin_self_intersection = param.as_bool();
          } else if (param.get_name() == "max_lateral_rate") {
            config_.max_lateral_rate = param.as_double();
          } else if (param.get_name() == "hairpin_radius") {
            hairpin_radius_ = param.as_double();
          } else if (param.get_name() == "straight_length") {
            straight_length_ = param.as_double();
          }
        }
        generator_.setConfig(config_);
        this->generateAndPublish();
        return result;
      });

    generator_.setConfig(config_);

    // 定时发布 (2Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SafeCorridorDemoNode::generateAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "============================================================");
    RCLCPP_INFO(this->get_logger(), "  Safe Corridor Hairpin Demo Node Initialized Successfully! ");
    RCLCPP_INFO(this->get_logger(), "  Topics:                                                  ");
    RCLCPP_INFO(this->get_logger(), "    - /safe_corridor_demo/markers (MarkerArray)            ");
    RCLCPP_INFO(this->get_logger(), "    - /safe_corridor_demo/reference_path (Path)            ");
    RCLCPP_INFO(this->get_logger(), "    - /safe_corridor_demo/left_boundary (Path)             ");
    RCLCPP_INFO(this->get_logger(), "    - /safe_corridor_demo/right_boundary (Path)            ");
    RCLCPP_INFO(this->get_logger(), "============================================================");
  }

private:
  void updateConfigFromParams()
  {
    config_.default_left_width = this->get_parameter("default_left_width").as_double();
    config_.default_right_width = this->get_parameter("default_right_width").as_double();
    config_.min_corridor_width = this->get_parameter("min_corridor_width").as_double();
    config_.curvature_safety_factor = this->get_parameter("curvature_safety_factor").as_double();
    config_.rib_safety_margin = this->get_parameter("rib_safety_margin").as_double();
    config_.max_lateral_rate = this->get_parameter("max_lateral_rate").as_double();
    config_.avoid_hairpin_self_intersection = this->get_parameter("avoid_hairpin_self_intersection").as_bool();
    config_.check_costmap = false;

    hairpin_radius_ = this->get_parameter("hairpin_radius").as_double();
    straight_length_ = this->get_parameter("straight_length").as_double();
  }

  std::vector<TrajectoryPoint> buildHairpinTrajectory() const
  {
    std::vector<TrajectoryPoint> traj;
    double s = 0.0;
    double ds = 0.05;
    double v = 0.5;

    // 1. 直线前段
    for (double x = 0.0; x <= straight_length_; x += ds) {
      TrajectoryPoint pt;
      pt.x = x;
      pt.y = 0.0;
      pt.theta = 0.0;
      pt.kappa = 0.0;
      pt.s = s;
      pt.v = v;
      pt.t = s / v;
      traj.push_back(pt);
      s += ds;
    }

    // 2. 180 度掉头圆弧 (左转弯，圆心在 (straight_length_, hairpin_radius_))
    double R = hairpin_radius_;
    double kappa = 1.0 / R;
    double dphi = ds / R;
    for (double phi = -M_PI / 2.0 + dphi; phi <= M_PI / 2.0; phi += dphi) {
      TrajectoryPoint pt;
      pt.x = straight_length_ + R * std::cos(phi);
      pt.y = R + R * std::sin(phi);
      pt.theta = phi + M_PI / 2.0;
      pt.kappa = kappa;
      pt.s = s;
      pt.v = v;
      pt.t = s / v;
      traj.push_back(pt);
      s += ds;
    }

    // 3. 反向直线后段
    double end_y = 2.0 * R;
    for (double x = straight_length_; x >= 0.0; x -= ds) {
      TrajectoryPoint pt;
      pt.x = x;
      pt.y = end_y;
      pt.theta = M_PI;
      pt.kappa = 0.0;
      pt.s = s;
      pt.v = v;
      pt.t = s / v;
      traj.push_back(pt);
      s += ds;
    }

    return traj;
  }

  void generateAndPublish()
  {
    auto stamp = this->now();
    std::string frame_id = "map";

    auto ref_traj = buildHairpinTrajectory();
    if (ref_traj.empty()) return;

    // 1. 生成安全走廊
    SafeCorridor corridor = generator_.generateCorridor(ref_traj, nullptr, frame_id, stamp);

    // 2. 发布参考路径
    nav_msgs::msg::Path ref_path_msg;
    ref_path_msg.header.frame_id = frame_id;
    ref_path_msg.header.stamp = stamp;

    nav_msgs::msg::Path left_bound_msg;
    left_bound_msg.header = ref_path_msg.header;

    nav_msgs::msg::Path right_bound_msg;
    right_bound_msg.header = ref_path_msg.header;

    for (const auto & b : corridor.bounds) {
      // 中心参考点
      geometry_msgs::msg::PoseStamped p;
      p.header = ref_path_msg.header;
      p.pose.position.x = b.x;
      p.pose.position.y = b.y;
      tf2::Quaternion q;
      q.setRPY(0, 0, b.theta);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      ref_path_msg.poses.push_back(p);

      // 左边界点
      geometry_msgs::msg::PoseStamped p_left = p;
      p_left.pose.position.x = b.left_x;
      p_left.pose.position.y = b.left_y;
      left_bound_msg.poses.push_back(p_left);

      // 右边界点
      geometry_msgs::msg::PoseStamped p_right = p;
      p_right.pose.position.x = b.right_x;
      p_right.pose.position.y = b.right_y;
      right_bound_msg.poses.push_back(p_right);
    }

    path_pub_->publish(ref_path_msg);
    left_boundary_pub_->publish(left_bound_msg);
    right_boundary_pub_->publish(right_bound_msg);

    // 3. 发布 MarkerArray 可视化 (包含 3D Mesh 带, 边界线, 截面法向肋线)
    auto marker_array = generator_.createVisualizationMarkers(corridor);
    marker_pub_->publish(marker_array);
  }

  SafeCorridorConfig config_;
  SafeCorridorGenerator generator_;
  double hairpin_radius_{1.2};
  double straight_length_{4.0};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr left_boundary_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr right_boundary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace nav2_mpc_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_mpc_controller::SafeCorridorDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
