#include <mpc_controller.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_mpc_controller
{
    bool transformPlan(
        const std::shared_ptr<tf2_ros::Buffer> & tf,
        const nav_msgs::msg::Path & path,
        const std::string & target_frame,
        nav_msgs::msg::Path & output_path)
    {
        if (path.poses.empty()) {
            return false;
        }
        
        output_path.header.frame_id = target_frame;
        output_path.header.stamp = path.header.stamp;
        output_path.poses.clear();
        
        try {
            geometry_msgs::msg::TransformStamped transform = tf->lookupTransform(
                target_frame, path.header.frame_id, tf2::TimePointZero);
                
            for (const auto & pose : path.poses) {
                geometry_msgs::msg::PoseStamped transformed_pose;
                tf2::doTransform(pose, transformed_pose, transform);
                output_path.poses.push_back(transformed_pose);
            }
            return true;
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(rclcpp::get_logger("transformPlan"), "Exception in transformPlan: %s", ex.what());
            return false;
        }
    }

}