#include <mpc_controller.hpp>
#include <limits>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_mpc_controller
{
    nav_msgs::msg::Path extractLocalPlan(
        const geometry_msgs::msg::PoseStamped & pose,
        const nav_msgs::msg::Path & transformed_plan,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
    {
        nav_msgs::msg::Path local_plan;
        local_plan.header = transformed_plan.header;

        if (transformed_plan.poses.empty()) {
            return local_plan;
        }

        // 1. 寻找距离当前位姿最近的点
        double current_x = pose.pose.position.x;
        double current_y = pose.pose.position.y;
        
        size_t closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < transformed_plan.poses.size(); ++i) {
            double dx = transformed_plan.poses[i].pose.position.x - current_x;
            double dy = transformed_plan.poses[i].pose.position.y - current_y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 2. 截取最近点之后的点，并保证在 costmap 范围内
        auto * costmap = costmap_ros->getCostmap();
        for (size_t i = closest_idx; i < transformed_plan.poses.size(); ++i) {
            const auto & p = transformed_plan.poses[i];
            unsigned int mx, my;
            // 检查点是否在代价地图范围内
            if (costmap->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
                local_plan.poses.push_back(p);
            } else {
                // 若超出代价地图范围，则停止截取以保持局部路径在已知区域内连续
                break;
            }
        }
        return local_plan;
    }
}  // namespace nav2_mpc_controller