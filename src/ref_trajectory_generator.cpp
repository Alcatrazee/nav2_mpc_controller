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

    // 1. 计算初始输入路径的累积弧长 (s_raw)
    std::vector<double> s_raw(M, 0.0);
    for (int i = 1; i < M; ++i) {
        double dx = local_plan.poses[i].pose.position.x - local_plan.poses[i-1].pose.position.x;
        double dy = local_plan.poses[i].pose.position.y - local_plan.poses[i-1].pose.position.y;
        s_raw[i] = s_raw[i-1] + std::hypot(dx, dy);
    }
    double S_total = s_raw.back();

    // 2. 利用最小二乘法拟合多项式 (最高5次，动态适应点太少的情况以避免奇异)
    int degree = std::min(5, M - 1);
    Eigen::MatrixXd A(M, degree + 1);
    Eigen::VectorXd B_x(M), B_y(M);

    for (int i = 0; i < M; ++i) {
        double s = s_raw[i];
        for (int j = 0; j <= degree; ++j) {
        A(i, j) = std::pow(s, j);
        }
        B_x(i) = local_plan.poses[i].pose.position.x;
        B_y(i) = local_plan.poses[i].pose.position.y;
    }

    // SVD 分解保证最小二乘求解极度鲁棒
    Eigen::VectorXd C_x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_x);
    Eigen::VectorXd C_y = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_y);

    // 3. 重采样为 0.001 间隔的点
    double ds_resample = 0.001;
    std::vector<PathPoint> path_points;
    int num_resampled = std::ceil(S_total / ds_resample) + 1;
    path_points.reserve(num_resampled);

    for (double s = 0; s <= S_total; s += ds_resample) {
        double x = 0.0, y = 0.0, dx_ds = 0.0, dy_ds = 0.0, d2x_ds2 = 0.0, d2y_ds2 = 0.0;
        for (int j = 0; j <= degree; ++j) {
        double term = std::pow(s, j);
        x += C_x(j) * term;
        y += C_y(j) * term;
        if (j >= 1) {
            double d_term = j * std::pow(s, j - 1);
            dx_ds += C_x(j) * d_term;
            dy_ds += C_y(j) * d_term;
        }
        if (j >= 2) {
            double d2_term = j * (j - 1) * std::pow(s, j - 2);
            d2x_ds2 += C_x(j) * d2_term;
            d2y_ds2 += C_y(j) * d2_term;
        }
        }

        double theta = std::atan2(dy_ds, dx_ds);
        double norm = dx_ds * dx_ds + dy_ds * dy_ds;
        double kappa = (norm > 1e-6) ? (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / std::pow(norm, 1.5) : 0.0;

        path_points.push_back({x, y, theta, kappa, s});
    }

    // 4. 利用 Profiler 进行速度与时间规划
    trajectory_profiler_->generate_profile(path_points, current_speed);

    // 5. 等 dt 获取 N 步采样点 (已内嵌精确插值)
    ref_traj.resize(N_);
    for (int k = 0; k < N_; ++k) {
        double target_time = (k + 1) * dt_;
        ref_traj[k] = trajectory_profiler_->get_reference_point(target_time);
    }

    return ref_traj;
    }
}  // namespace nav2_mpc_controller