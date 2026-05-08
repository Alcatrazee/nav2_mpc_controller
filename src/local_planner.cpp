#include <mpc_controller.hpp>
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <limits>

namespace nav2_mpc_controller
{

nav_msgs::msg::Path MPCController::generateLatticePlan(
    const geometry_msgs::msg::PoseStamped & pose,
    const nav_msgs::msg::Path & base_plan,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & checker,
    bool is_plan_contain_goal)
{
  nav_msgs::msg::Path best_plan;
  best_plan.header = base_plan.header;
  
  if (base_plan.poses.empty()) {
    return best_plan;
  }

  double current_x = pose.pose.position.x;
  double current_y = pose.pose.position.y;
  double current_theta = tf2::getYaw(pose.pose.orientation);

  auto footprint = costmap_ros_->getRobotFootprint();

  double best_score = std::numeric_limits<double>::max();
  int best_idx = -1;

  struct Candidate {
      nav_msgs::msg::Path path;
      double score;
      bool collision;
      double lat;
      double obs_cost;
      double smooth_cost;
      double lon_dist;
  };
  std::vector<Candidate> candidates;
  
  // 1. 基于参考路径长度 (s) 构建 Frenet 采样坐标系
  std::vector<double> base_s(base_plan.poses.size(), 0.0);
  for (size_t i = 1; i < base_plan.poses.size(); ++i) {
      double dx = base_plan.poses[i].pose.position.x - base_plan.poses[i-1].pose.position.x;
      double dy = base_plan.poses[i].pose.position.y - base_plan.poses[i-1].pose.position.y;
      base_s[i] = base_s[i-1] + std::hypot(dx, dy);
  }
  double total_base_s = base_s.back();

  if (total_base_s < 0.01) {
      return best_plan;
  }

  // 闭包函数：根据弧长 s 获取参考路径上的插值状态
  auto get_ref_state = [&](double s) {
      if (s <= 0.0) return base_plan.poses.front();
      if (s >= total_base_s) return base_plan.poses.back();
      
      auto it = std::lower_bound(base_s.begin(), base_s.end(), s);
      int idx = std::distance(base_s.begin(), it);
      if (idx == 0) return base_plan.poses.front();
      
      double s1 = base_s[idx - 1];
      double s2 = base_s[idx];
      double ratio = (s - s1) / (s2 - s1 + 1e-6);
      
      const auto & p1 = base_plan.poses[idx - 1].pose;
      const auto & p2 = base_plan.poses[idx].pose;
      
      geometry_msgs::msg::PoseStamped res;
      res.header = base_plan.header;
      res.pose.position.x = p1.position.x + ratio * (p2.position.x - p1.position.x);
      res.pose.position.y = p1.position.y + ratio * (p2.position.y - p1.position.y);
      
      double th1 = tf2::getYaw(p1.orientation);
      double th2 = tf2::getYaw(p2.orientation);
      double dth = th2 - th1;
      while (dth > M_PI) dth -= 2.0 * M_PI;
      while (dth < -M_PI) dth += 2.0 * M_PI;
      
      double th = th1 + ratio * dth;
      tf2::Quaternion q;
      q.setRPY(0, 0, th);
      res.pose.orientation.x = q.x();
      res.pose.orientation.y = q.y();
      res.pose.orientation.z = q.z();
      res.pose.orientation.w = q.w();
      return res;
  };

  // 2. 获取 Frenet 坐标系下的初始状态
  double ref_x0 = base_plan.poses[0].pose.position.x;
  double ref_y0 = base_plan.poses[0].pose.position.y;
  double ref_theta0 = tf2::getYaw(base_plan.poses[0].pose.orientation);

  double dx0 = current_x - ref_x0;
  double dy0 = current_y - ref_y0;
  // 横向初始偏移量 d0
  double d0 = dx0 * (-std::sin(ref_theta0)) + dy0 * std::cos(ref_theta0);

  // 航向角初始偏差，用于计算初始导数 d'
  double delta_theta = current_theta - ref_theta0;
  while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
  while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;
  delta_theta = std::clamp(delta_theta, -M_PI/3.0, M_PI/3.0);
  double d_prime0 = std::tan(delta_theta);

  double previous_target_s = -1.0;

  // 3. 多前瞻距离采样与横向偏移计算
  for (double lon_ratio : lattice_lon_ratios_) {
      double target_s = lattice_lookahead_dist_ * lon_ratio;
      if (target_s > total_base_s) {
          target_s = total_base_s;
      }
      if (target_s < 0.1) {
          continue;
      }

      // 防止局部路径过短导致重复计算
      if (std::abs(target_s - previous_target_s) < 1e-3) {
          continue;
      }
      previous_target_s = target_s;

      bool is_end_of_plan = (target_s >= total_base_s - 1e-3);

      double lat_start = -lattice_lat_range_;
      double lat_end = lattice_lat_range_;
      double lat_step = lattice_lat_step_;

      // 抵达终点附近时收敛横向误差，保证准确到达
      if (is_plan_contain_goal && is_end_of_plan) {
          lat_start = 0.0;
          lat_end = 0.0;
      }
      
      for (double lat = lat_start; lat <= lat_end + 1e-5; lat += lat_step) {
          
          // 在归一化参数 u 上的五次多项式边界条件
          double p0 = d0;
          double v0 = d_prime0 * target_s;
          double a0 = 0.0;
          double p1 = lat;
          double v1 = 0.0;
          double a1 = 0.0;

          auto calc_quintic_coeffs = [](double p0, double v0, double a0, double p1, double v1, double a1) {
              std::vector<double> c(6);
              c[0] = p0;
              c[1] = v0;
              c[2] = a0 / 2.0;
              c[3] = 10*(p1 - p0) - 6*v0 - 4*v1 - 1.5*a0 + 0.5*a1;
              c[4] = -15*(p1 - p0) + 8*v0 + 7*v1 + 1.5*a0 - a1;
              c[5] = 6*(p1 - p0) - 3*v0 - 3*v1 - 0.5*a0 + 0.5*a1;
              return c;
          };

          auto C = calc_quintic_coeffs(p0, v0, a0, p1, v1, a1);

          nav_msgs::msg::Path candidate_path;
          candidate_path.header = base_plan.header;
          
          bool collision = false;
          double obs_cost_sum = 0.0;
          double smooth_cost_sum = 0.0;

          int num_samples = 20; 
          for (int i = 0; i <= num_samples; ++i) {
              double u = static_cast<double>(i) / num_samples;
              double s_i = u * target_s;
              
              double u2 = u*u, u3 = u2*u, u4 = u3*u, u5 = u4*u;
              double d = C[0] + C[1]*u + C[2]*u2 + C[3]*u3 + C[4]*u4 + C[5]*u5;
              double d_prime_u = C[1] + 2*C[2]*u + 3*C[3]*u2 + 4*C[4]*u3 + 5*C[5]*u4;
              double d_prime = d_prime_u / target_s;
              
              // 基于参考路径映射回笛卡尔坐标系
              auto ref_pose = get_ref_state(s_i);
              double ref_x = ref_pose.pose.position.x;
              double ref_y = ref_pose.pose.position.y;
              double ref_theta = tf2::getYaw(ref_pose.pose.orientation);
              
              double x = ref_x - d * std::sin(ref_theta);
              double y = ref_y + d * std::cos(ref_theta);
              double theta = ref_theta + std::atan(d_prime);
              
              double cost = checker.footprintCostAtPose(x, y, theta, footprint);
              if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) { 
                  collision = true;
                  break;
              }
              if (cost >= 0) {
                 obs_cost_sum += cost;
              }

              geometry_msgs::msg::PoseStamped p;
              p.header = base_plan.header;
              p.pose.position.x = x;
              p.pose.position.y = y;
              tf2::Quaternion q;
              q.setRPY(0, 0, theta);
              p.pose.orientation.x = q.x();
              p.pose.orientation.y = q.y();
              p.pose.orientation.z = q.z();
              p.pose.orientation.w = q.w();
              candidate_path.poses.push_back(p);
          }

          if (collision) {
              candidates.push_back({candidate_path, std::numeric_limits<double>::max(), true, lat, 0.0, 0.0, 0.0});
              continue;
          }

          for (size_t i = 1; i < candidate_path.poses.size(); ++i) {
              double th1 = tf2::getYaw(candidate_path.poses[i-1].pose.orientation);
              double th2 = tf2::getYaw(candidate_path.poses[i].pose.orientation);
              double dth = th2 - th1;
              while (dth > M_PI) dth -= 2.0 * M_PI;
              while (dth < -M_PI) dth += 2.0 * M_PI;
              
              double dx_pt = candidate_path.poses[i].pose.position.x - candidate_path.poses[i-1].pose.position.x;
              double dy_pt = candidate_path.poses[i].pose.position.y - candidate_path.poses[i-1].pose.position.y;
              double dist = std::hypot(dx_pt, dy_pt);
              
              double kappa = (dist > 1e-3) ? dth / dist : 0.0;
              smooth_cost_sum += kappa * kappa;
          }

          candidates.push_back({candidate_path, 0.0, false, lat, obs_cost_sum, smooth_cost_sum, target_s});
      }
  }

  // 4. 对所有候选曲线的各项成本进行归一化，并计算总分
  if (!candidates.empty()) {
    double max_obs_cost = std::numeric_limits<double>::epsilon();
    double max_smooth_cost = std::numeric_limits<double>::epsilon();
    double max_lat_cost = std::numeric_limits<double>::epsilon();
    double max_lon_dist = std::numeric_limits<double>::epsilon();

    for (const auto& cand : candidates) {
        if (!cand.collision) {
            max_obs_cost = std::max(max_obs_cost, cand.obs_cost);
            max_smooth_cost = std::max(max_smooth_cost, cand.smooth_cost);
            max_lat_cost = std::max(max_lat_cost, std::abs(cand.lat));
            max_lon_dist = std::max(max_lon_dist, cand.lon_dist);
        }
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        auto & cand = candidates[i];
        if (cand.collision) {
            continue;
        }

        double norm_obs = cand.obs_cost / max_obs_cost;
        double norm_smooth = cand.smooth_cost / max_smooth_cost;
        double norm_lat = std::abs(cand.lat) / max_lat_cost;
        double norm_lon = cand.lon_dist / max_lon_dist;

        cand.score = weight_obs_ * norm_obs +
                     weight_smooth_ * norm_smooth +
                     weight_lat_ * norm_lat -
                     1.0 * norm_lon;
        
        if (cand.score < best_score) {
            best_score = cand.score;
            best_plan = cand.path;
            best_idx = i;
        }
    }
  }

  // 5. 发布所有采样的 Lattice 候选曲线进行可视化
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  for (size_t i = 0; i < candidates.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = base_plan.header;
      marker.ns = "lattice_candidates";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.02; // 线宽

      if (candidates[i].collision) {
          marker.color.r = 1.0; marker.color.a = 0.3; // 红色，半透明（碰撞）
      } else if (static_cast<int>(i) == best_idx) {
          marker.color.g = 1.0; marker.color.a = 1.0; // 绿色，不透明（最优轨迹）
          marker.scale.x = 0.06; // 加粗
      } else {
          marker.color.b = 1.0; marker.color.g = 1.0; marker.color.a = 0.5; // 青色，半透明（安全但非最优）
      }

      for (const auto& p : candidates[i].path.poses) {
          marker.points.push_back(p.pose.position);
      }
      marker_array.markers.push_back(marker);
  }
  
  if (lattice_candidates_pub_) {
      lattice_candidates_pub_->publish(marker_array);
  }

  // 打印最佳曲线得分及其偏移量
  if (best_idx != -1) {
      RCLCPP_INFO(logger_, "Best Lattice Path: lat offset = %.2fm, score = %.3f", 
                  candidates[best_idx].lat, candidates[best_idx].score);
  } else {
      RCLCPP_WARN(logger_, "All Lattice Candidates in collision!");
  }

  return best_plan;
}

}  // namespace nav2_mpc_controller
