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

  if (total_base_s < 0.001) {
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

  // 3. 分层多前瞻距离采样与多项式拟合 (Hierarchical Lattice)
  std::vector<double> valid_s;
  std::vector<std::vector<double>> valid_lat_samples;

  for (double ratio : lattice_lon_ratios_) {
      RCLCPP_INFO(logger_,"ratio: %.2f",ratio);
      double s_i = lattice_lookahead_dist_ * ratio;
      if (s_i > total_base_s) s_i = total_base_s;
      if (s_i < 0.01) continue;
      
      // 防止局部路径过短导致重复距离 (避免矩阵奇异)
      if (!valid_s.empty() && std::abs(s_i - valid_s.back()) < 1e-3) {
          continue;
      }
      valid_s.push_back(s_i);
      
      bool is_end = (s_i >= total_base_s - 1e-3);
      std::vector<double> samples;
      // 抵达终点附近时收敛横向误差，保证准确到达
      if (is_plan_contain_goal && is_end) {
          samples.push_back(0.0);
      } else {
          for (double lat = -lattice_lat_range_; lat <= lattice_lat_range_ + 1e-5; lat += lattice_lat_step_) {
              samples.push_back(lat);
          }
      }
      valid_lat_samples.push_back(samples);
  }

  if (valid_s.empty()) {
      return best_plan;
  }

  double S_max = valid_s.back();
  int K = valid_s.size();

  // 归一化后的初始导数
  double v0 = d_prime0 * S_max;

  // --- 三次 B 样条配置开始 ---
  int p = 3; // 样条阶数 (Cubic B-Spline)
  int n_cp = K + 4; // 控制点数量: Start(1) + Vel(1) + Internal(K-1) + End(3)
  std::vector<double> U(n_cp + p + 1, 0.0); // 节点向量

  // 计算各层距离的归一化比例
  std::vector<double> layer_ratios;
  layer_ratios.push_back(0.0);
  for (int i = 0; i < K; ++i) {
      layer_ratios.push_back(valid_s[i] / S_max);
  }
  
  // 内部节点设为相邻比例层的中点，使得各层控制点的影响峰值完美贴合动态设定的纵向比例
  for (int i = 0; i <= p; ++i) U[i] = 0.0;
  for (int i = 1; i <= K; ++i) {
      U[p + i] = (layer_ratios[i - 1] + layer_ratios[i]) / 2.0;
  }
  
  for (int i = n_cp; i < n_cp + p + 1; ++i) U[i] = 1.0;

  // B 样条基函数闭包
  std::function<double(int, int, double)> bspline_basis = [&](int i, int p_deg, double u) -> double {
      if (p_deg == 0) {
          if (U[i] < U[i+1]) {
              if (u >= U[i] && u < U[i+1]) return 1.0;
              // 处理 u=1 落在最后一个有效节点区间的情况
              if (std::abs(u - 1.0) < 1e-6 && std::abs(U[i+1] - 1.0) < 1e-6) return 1.0;
          }
          return 0.0;
      }
      double left = 0.0, right = 0.0;
      if (U[i+p_deg] - U[i] > 1e-6) {
          left = (u - U[i]) / (U[i+p_deg] - U[i]) * bspline_basis(i, p_deg-1, u);
      }
      if (U[i+p_deg+1] - U[i+1] > 1e-6) {
          right = (U[i+p_deg+1] - u) / (U[i+p_deg+1] - U[i+1]) * bspline_basis(i+1, p_deg-1, u);
      }
      return left + right;
  };

  // B 样条基函数导数闭包
  std::function<double(int, int, double)> bspline_deriv = [&](int i, int p_deg, double u) -> double {
      double left = 0.0, right = 0.0;
      if (U[i+p_deg] - U[i] > 1e-6) {
          left = p_deg / (U[i+p_deg] - U[i]) * bspline_basis(i, p_deg-1, u);
      }
      if (U[i+p_deg+1] - U[i+1] > 1e-6) {
          right = p_deg / (U[i+p_deg+1] - U[i+1]) * bspline_basis(i+1, p_deg-1, u);
      }
      return left - right;
  };

  int num_samples = std::max(20, 10 * K); 
  std::vector<std::vector<double>> B(num_samples + 1, std::vector<double>(n_cp, 0.0));
  std::vector<std::vector<double>> B_prime(num_samples + 1, std::vector<double>(n_cp, 0.0));

  // 预计算所有采样点的基函数值，极大加速组合遍历
  for (int i = 0; i <= num_samples; ++i) {
      double u = static_cast<double>(i) / num_samples;
      for (int j = 0; j < n_cp; ++j) {
          B[i][j] = bspline_basis(j, p, u);
          B_prime[i][j] = bspline_deriv(j, p, u);
      }
  }
  // --- B 样条配置结束 ---

  // 递归生成所有分层的横向偏移组合
  std::vector<std::vector<double>> all_combinations;
  std::function<void(int, std::vector<double>&)> generate_combos = [&](int layer_idx, std::vector<double>& current_combo) {
      if (layer_idx == K) {
          all_combinations.push_back(current_combo);
          return;
      }
      for (double lat : valid_lat_samples[layer_idx]) {
          // 启发式剪枝：丢弃相邻层横向跳变过大的锯齿形轨迹，防止组合爆炸
          if (!current_combo.empty()) {
              if (std::abs(lat - current_combo.back()) > lattice_lat_range_ * 1.5) {
                  continue;
              }
          }
          current_combo.push_back(lat);
          generate_combos(layer_idx + 1, current_combo);
          current_combo.pop_back();
      }
  };
  std::vector<double> initial_combo;
  generate_combos(0, initial_combo);

  for (const auto& lat_combo : all_combinations) {
      // 构建 B 样条控制点
      std::vector<double> CP(n_cp, 0.0);
      CP[0] = d0;
      CP[1] = d0 + v0 * U[p + 1] / 3.0; // 满足起始导数
      
      for (int i = 0; i < K - 1; ++i) {
          CP[i + 2] = lat_combo[i];
      }
      
      double lat_end = lat_combo.back();
      CP[n_cp - 3] = lat_end; // 终点位置
      CP[n_cp - 2] = lat_end; // 终点一阶导数为 0
      CP[n_cp - 1] = lat_end; // 终点二阶导数为 0

      nav_msgs::msg::Path candidate_path;
      candidate_path.header = base_plan.header;
      
      bool collision = false;
      double obs_cost_sum = 0.0;
      double smooth_cost_sum = 0.0;

      for (int i = 0; i <= num_samples; ++i) {
          double u = static_cast<double>(i) / num_samples;
          double s_i = u * S_max;
          
          double d = 0.0;
          double d_prime_u = 0.0;
          
          for (int j = 0; j < n_cp; ++j) {
              d += CP[j] * B[i][j];
              d_prime_u += CP[j] * B_prime[i][j];
          }
          
          double d_prime = d_prime_u / S_max;
          
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
          candidates.push_back({candidate_path, std::numeric_limits<double>::max(), true, lat_combo.back(), 0.0, 0.0, 0.0});
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

      candidates.push_back({candidate_path, 0.0, false, lat_combo.back(), obs_cost_sum, smooth_cost_sum, S_max});
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
    //   RCLCPP_INFO(logger_, "Best Lattice Path: lat offset = %.2fm, score = %.3f", 
    //               candidates[best_idx].lat, candidates[best_idx].score);
  } else {
      RCLCPP_WARN(logger_, "All Lattice Candidates in collision!");
  }

  return best_plan;
}

}  // namespace nav2_mpc_controller
