#include <mpc_controller.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include <cmath>

namespace nav2_mpc_controller
{

void MPCController::publishParameterizedTrajectory(
    const std::vector<TrajectoryPoint> & ref_points,
    const std_msgs::msg::Header & header)
{
  // 将经过N点参考点采样后的轨迹作为 Path 消息发布到 local_plan 话题
  nav_msgs::msg::Path parameterized_local_plan;
  parameterized_local_plan.header = header;

  for (const auto & pt : ref_points) {
    geometry_msgs::msg::PoseStamped p;
    p.header = header;
    p.pose.position.x = pt.x;
    p.pose.position.y = pt.y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    
    parameterized_local_plan.poses.push_back(p);
  }
  local_plan_pub_->publish(parameterized_local_plan);

  // 同时发布 MarkerArray，将速度大小映射为颜色
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 1. 添加一个 DELETEALL marker 清除 RViz 中上一帧的残留箭头
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  // 2. 遍历参考点，生成彩色箭头 Marker
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & pt = ref_points[i];
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "velocity_arrows";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = pt.x;
    marker.pose.position.y = pt.y;
    marker.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // 箭头尺寸 (x: 长度, y: 箭头宽, z: 箭头高)
    marker.scale.x = 0.08; 
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;

    // 速度颜色映射: 计算速度比例 (绿 -> 红)
    double ratio = std::clamp(std::abs(pt.v) / v_max_, 0.0, 1.0);
    marker.color.r = ratio;           // 速度越接近最大值，红色通道越满
    marker.color.g = 1.0 - ratio;     // 速度越低，绿色通道越满
    marker.color.b = 0.0;
    marker.color.a = 1.0;             // 不透明度

    marker_array.markers.push_back(marker);
  }
  local_plan_marker_pub_->publish(marker_array);
}

}  // namespace nav2_mpc_controller