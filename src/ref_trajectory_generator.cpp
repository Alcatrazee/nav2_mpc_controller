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

    std::vector<TrajectoryPoint> MPCController::generateTimeParameterizedTrajectory(
        const nav_msgs::msg::Path & local_plan, 
        double current_speed)
    {
    std::vector<TrajectoryPoint> ref_traj;
    int M = local_plan.poses.size();
    
    if (M < 2) {
        return ref_traj; // 路径太短，直接返回空
    }

    // 1. 提取原始路径点
    std::vector<PathPoint> path_points(M);
    for (int i = 0; i < M; ++i) {
        path_points[i].x = local_plan.poses[i].pose.position.x;
        path_points[i].y = local_plan.poses[i].pose.position.y;
        path_points[i].theta = tf2::getYaw(local_plan.poses[i].pose.orientation);
    }

    // 1.5 针对原始 A* 等非平滑路径进行带数据保形约束的平滑 (Data-Constrained Smoothing)
    if (!use_local_plan_) {
        // 备份原始路径点，用于提供保形引力
        std::vector<PathPoint> original_points = path_points;
        
        int smooth_iterations = 30; // 增加迭代次数以彻底消除曲率毛刺
        double alpha = 0.3;         // 平滑权重 (拉向相邻点中点)
        double beta = 0.2;          // 保形权重 (拉回原始路径的力度，防止切内弯)
        
        for (int iter = 0; iter < smooth_iterations; ++iter) {
            for (int i = 1; i < M - 1; ++i) {
                double smooth_dx = path_points[i-1].x + path_points[i+1].x - 2.0 * path_points[i].x;
                double smooth_dy = path_points[i-1].y + path_points[i+1].y - 2.0 * path_points[i].y;
                
                double data_dx = original_points[i].x - path_points[i].x;
                double data_dy = original_points[i].y - path_points[i].y;

                path_points[i].x += alpha * smooth_dx + beta * data_dx;
                path_points[i].y += alpha * smooth_dy + beta * data_dy;
            }
        }
        
        // 使用中心差分重新计算平滑后的朝向 (theta)，保留 path_points[0].theta 为当前机器人的真实朝向
        for (int i = 1; i < M - 1; ++i) {
            double dx = path_points[i+1].x - path_points[i-1].x;
            double dy = path_points[i+1].y - path_points[i-1].y;
            if (std::hypot(dx, dy) > 1e-4) {
                path_points[i].theta = std::atan2(dy, dx);
            }
        }
        double dx_end = path_points[M-1].x - path_points[M-2].x;
        double dy_end = path_points[M-1].y - path_points[M-2].y;
        if (std::hypot(dx_end, dy_end) > 1e-4) {
            path_points[M-1].theta = std::atan2(dy_end, dx_end);
        }
    }

    // 1.6 重新计算累积弧长 (s)
    path_points[0].s = 0.0;
    for (int i = 1; i < M; ++i) {
        double dx = path_points[i].x - path_points[i-1].x;
        double dy = path_points[i].y - path_points[i-1].y;
        path_points[i].s = path_points[i-1].s + std::hypot(dx, dy);
    }

    // 2. 利用运动学朝向精确计算离散曲率 (kappa = d_theta / ds)
    // 彻底摒弃高阶多项式带来的龙格震荡灾难
    for (int i = 0; i < M; ++i) {
        if (i == 0 || i == M - 1) {
            path_points[i].kappa = 0.0;
        } else {
            double th1 = path_points[i-1].theta;
            double th2 = path_points[i+1].theta;
            double dth = th2 - th1;
            while (dth > M_PI) dth -= 2.0 * M_PI;
            while (dth < -M_PI) dth += 2.0 * M_PI;
            
            double ds = path_points[i+1].s - path_points[i-1].s;
            path_points[i].kappa = (ds > 1e-4) ? (dth / ds) : 0.0;
        }
    }

    // 3. 几何稠密化重采样 (最高 0.02m 分辨率)，为 Profiler 提供安全且无震荡的路径积分空间
    double max_ds = 0.02; 
    std::vector<PathPoint> dense_points;
    dense_points.push_back(path_points[0]);
    
    for (int i = 1; i < M; ++i) {
        double ds = path_points[i].s - path_points[i-1].s;
        if (ds > max_ds) {
            int num_inserts = std::floor(ds / max_ds);
            for (int j = 1; j <= num_inserts; ++j) {
                double ratio = static_cast<double>(j) / (num_inserts + 1.0);
                PathPoint pt;
                pt.x = path_points[i-1].x + ratio * (path_points[i].x - path_points[i-1].x);
                pt.y = path_points[i-1].y + ratio * (path_points[i].y - path_points[i-1].y);
                
                double th1 = path_points[i-1].theta;
                double th2 = path_points[i].theta;
                double dth = th2 - th1;
                while (dth > M_PI) dth -= 2.0 * M_PI;
                while (dth < -M_PI) dth += 2.0 * M_PI;
                pt.theta = th1 + ratio * dth;
                
                pt.kappa = path_points[i-1].kappa + ratio * (path_points[i].kappa - path_points[i-1].kappa);
                pt.s = path_points[i-1].s + ratio * ds;
                dense_points.push_back(pt);
            }
        }
        dense_points.push_back(path_points[i]);
    }

    // 4. 利用 Profiler 进行速度与时间规划
    trajectory_profiler_->generate_profile(dense_points, current_speed);

    // 5. 等 dt 获取 N 步采样点 (已内嵌精确插值)
    ref_traj.resize(N_);
    for (int k = 0; k < N_; ++k) {
        double target_time = (k + 1) * dt_;
        ref_traj[k] = trajectory_profiler_->get_reference_point(target_time);
    }

    return ref_traj;
    }
}  // namespace nav2_mpc_controller