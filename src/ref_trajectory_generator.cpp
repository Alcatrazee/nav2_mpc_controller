#include <mpc_controller.hpp>
#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iterator>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace
{
class CubicSpline1D {
public:
  bool build(const std::vector<double>& s, const std::vector<double>& a_val) {
    int n = s.size();
    if (n < 3) return false;
    a = a_val;
    b.resize(n - 1);
    c.resize(n, 0.0);
    d.resize(n - 1);
    s_vec = s;

    std::vector<double> h(n - 1);
    for (int i = 0; i < n - 1; ++i) {
      h[i] = s[i + 1] - s[i];
      if (h[i] <= 1e-6) return false;
    }

    std::vector<double> A(n, 0.0), B(n, 0.0), C(n, 0.0), D(n, 0.0);
    for (int i = 1; i < n - 1; ++i) {
      A[i] = h[i - 1];
      B[i] = 2.0 * (h[i - 1] + h[i]);
      C[i] = h[i];
      D[i] = 3.0 * ((a[i + 1] - a[i]) / h[i] - (a[i] - a[i - 1]) / h[i - 1]);
    }

    std::vector<double> c_tmp(n, 0.0);
    for (int i = 1; i < n - 1; ++i) {
      double m = 1.0 / (B[i] - A[i] * c_tmp[i - 1]);
      c_tmp[i] = C[i] * m;
      D[i] = (D[i] - A[i] * D[i - 1]) * m;
    }
    for (int i = n - 2; i >= 1; --i) {
      c[i] = D[i] - c_tmp[i] * c[i + 1];
    }

    for (int i = 0; i < n - 1; ++i) {
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (2.0 * c[i] + c[i + 1]) / 3.0;
    }
    return true;
  }

  double calc_a(double s_query) const { return eval(s_query, 0); }
  double calc_d1(double s_query) const { return eval(s_query, 1); }
  double calc_d2(double s_query) const { return eval(s_query, 2); }

private:
  double eval(double query_s, int deriv_order) const {
    int n = s_vec.size();
    if (query_s <= s_vec.front()) query_s = s_vec.front();
    if (query_s >= s_vec.back()) query_s = s_vec.back();

    auto it = std::upper_bound(s_vec.begin(), s_vec.end(), query_s);
    int idx = std::max(0, static_cast<int>(std::distance(s_vec.begin(), it)) - 1);
    if (idx >= n - 1) idx = n - 2;

    double ds = query_s - s_vec[idx];
    if (deriv_order == 0) {
      return a[idx] + b[idx] * ds + c[idx] * ds * ds + d[idx] * ds * ds * ds;
    } else if (deriv_order == 1) {
      return b[idx] + 2.0 * c[idx] * ds + 3.0 * d[idx] * ds * ds;
    } else if (deriv_order == 2) {
      return 2.0 * c[idx] + 6.0 * d[idx] * ds;
    }
    return 0.0;
  }

  std::vector<double> a, b, c, d, s_vec;
};
} // namespace

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

        // 2. 截取最近点前后的点，并保证在 costmap 范围内 (向前回退3个点以确保终点处点数充足)
        size_t start_idx = (closest_idx >= 3) ? (closest_idx - 3) : 0;
        auto * costmap = costmap_ros->getCostmap();
        for (size_t i = start_idx; i < transformed_plan.poses.size(); ++i) {
            const auto & p = transformed_plan.poses[i];
            unsigned int mx, my;
            if (costmap->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
                local_plan.poses.push_back(p);
            } else {
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
    int M_raw = local_plan.poses.size();
    
    if (M_raw == 0) {
        return ref_traj;
    }

    // 1. 提取去重后的原始路径点，并计算累积弧长 s
    std::vector<double> raw_x, raw_y, raw_s;
    raw_x.push_back(local_plan.poses[0].pose.position.x);
    raw_y.push_back(local_plan.poses[0].pose.position.y);
    raw_s.push_back(0.0);

    for (int i = 1; i < M_raw; ++i) {
        double px = local_plan.poses[i].pose.position.x;
        double py = local_plan.poses[i].pose.position.y;
        double dx = px - raw_x.back();
        double dy = py - raw_y.back();
        double dist = std::hypot(dx, dy);
        if (dist > 1e-3) {
            raw_x.push_back(px);
            raw_y.push_back(py);
            raw_s.push_back(raw_s.back() + dist);
        }
    }

    std::vector<PathPoint> dense_points;

    // 2. 点数不足 3 个时的鲁棒高密保底插值 (终点极近进近保障)
    if (raw_s.size() < 3) {
        if (raw_s.size() == 1) {
            // 仅剩终点 1 点: 沿目标方向构建微小引导线段
            double th = tf2::getYaw(local_plan.poses.back().pose.orientation);
            double x0 = raw_x.front();
            double y0 = raw_y.front();
            for (int step = 0; step < 5; ++step) {
                PathPoint pt;
                pt.s = step * 0.05;
                pt.x = x0 + pt.s * std::cos(th);
                pt.y = y0 + pt.s * std::sin(th);
                pt.theta = th;
                pt.kappa = 0.0;
                dense_points.push_back(pt);
            }
        } else if (raw_s.size() == 2) {
            // 2 个点: 沿线段线性高密插值
            double x0 = raw_x[0], y0 = raw_y[0];
            double x1 = raw_x[1], y1 = raw_y[1];
            double seg_len = std::max(0.01, raw_s[1]);
            double th = std::atan2(y1 - y0, x1 - x0);
            for (double s = 0.0; s <= seg_len + 1e-4; s += 0.02) {
                double r = std::clamp(s / seg_len, 0.0, 1.0);
                PathPoint pt;
                pt.s = s;
                pt.x = x0 + r * (x1 - x0);
                pt.y = y0 + r * (y1 - y0);
                pt.theta = th;
                pt.kappa = 0.0;
                dense_points.push_back(pt);
            }
        }
    } else {
        // 3. 构建 C2 自然三次样条插值器 (Cubic B-Spline)
        CubicSpline1D spline_x, spline_y;
        if (spline_x.build(raw_s, raw_x) && spline_y.build(raw_s, raw_y)) {
            double total_s = raw_s.back();
            double max_ds = 0.05;
            for (double s = 0.0; s <= total_s; s += max_ds) {
                PathPoint pt;
                pt.s = s;
                pt.x = spline_x.calc_a(s);
                pt.y = spline_y.calc_a(s);

                double dx_ds = spline_x.calc_d1(s);
                double dy_ds = spline_y.calc_d1(s);
                double d2x_ds2 = spline_x.calc_d2(s);
                double d2y_ds2 = spline_y.calc_d2(s);

                double speed_s = std::hypot(dx_ds, dy_ds);
                if (speed_s > 1e-4) {
                    pt.theta = std::atan2(dy_ds, dx_ds);
                    pt.kappa = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / (speed_s * speed_s * speed_s);
                } else {
                    pt.theta = 0.0;
                    pt.kappa = 0.0;
                }
                dense_points.push_back(pt);
            }
        } else {
            // 样条失败保底
            for (size_t i = 0; i < raw_s.size(); ++i) {
                PathPoint pt;
                pt.s = raw_s[i];
                pt.x = raw_x[i];
                pt.y = raw_y[i];
                pt.theta = (i + 1 < raw_s.size()) ? std::atan2(raw_y[i+1]-raw_y[i], raw_x[i+1]-raw_x[i]) : dense_points.back().theta;
                pt.kappa = 0.0;
                dense_points.push_back(pt);
            }
        }
    }

    if (dense_points.size() < 2) {
        PathPoint pt2 = dense_points.front();
        pt2.s += 0.05;
        pt2.x += 0.05 * std::cos(pt2.theta);
        pt2.y += 0.05 * std::sin(pt2.theta);
        dense_points.push_back(pt2);
    }

    // 4. 利用 Profiler 进行速度与时间参数化
    trajectory_profiler_->generate_profile(dense_points, current_speed);

    // 5. 获取 N 步采样点 (支持非均匀时间步长，已内嵌精确插值)
    ref_traj.resize(N_);
    for (int k = 0; k < N_; ++k) {
        double target_time = (k < static_cast<int>(dt_cumsum_.size())) ? dt_cumsum_[k] : (k + 1) * dt_;
        ref_traj[k] = trajectory_profiler_->get_reference_point(target_time);
    }

    return ref_traj;
    }
}  // namespace nav2_mpc_controller
