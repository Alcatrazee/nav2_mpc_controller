#include "safe_corridor_generator.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_mpc_controller
{

SafeCorridorGenerator::SafeCorridorGenerator(const SafeCorridorConfig & config)
: config_(config)
{
}

void SafeCorridorGenerator::setConfig(const SafeCorridorConfig & config)
{
  config_ = config;
}

bool SafeCorridorGenerator::computeLineIntersection(
  double p1_x, double p1_y, double dir1_x, double dir1_y,
  double p2_x, double p2_y, double dir2_x, double dir2_y,
  double & lambda1, double & lambda2)
{
  // Line 1: P1 + lambda1 * Dir1
  // Line 2: P2 + lambda2 * Dir2
  // lambda1 * dir1_x - lambda2 * dir2_x = p2_x - p1_x
  // lambda1 * dir1_y - lambda2 * dir2_y = p2_y - p1_y
  double det = -dir1_x * dir2_y + dir1_y * dir2_x;
  if (std::abs(det) < 1e-6) {
    return false; // Parallel lines
  }

  double dx = p2_x - p1_x;
  double dy = p2_y - p1_y;

  lambda1 = (-dx * dir2_y + dy * dir2_x) / det;
  lambda2 = (dir1_x * dy - dir1_y * dx) / det;
  return true;
}

bool SafeCorridorGenerator::computeLineSegmentIntersection(
  double p1_x, double p1_y, double p2_x, double p2_y,
  double p3_x, double p3_y, double p4_x, double p4_y,
  double & intersect_x, double & intersect_y,
  double & t1, double & t2)
{
  double u_x = p2_x - p1_x;
  double u_y = p2_y - p1_y;
  double v_x = p4_x - p3_x;
  double v_y = p4_y - p3_y;

  double det = -u_x * v_y + u_y * v_x;
  if (std::abs(det) < 1e-7) {
    return false; // Parallel or collinear segments
  }

  double dx = p3_x - p1_x;
  double dy = p3_y - p1_y;

  t1 = (-dx * v_y + dy * v_x) / det;
  t2 = (u_x * dy - u_y * dx) / det;

  const double eps = 1e-5;
  if (t1 >= -eps && t1 <= 1.0 + eps && t2 >= -eps && t2 <= 1.0 + eps) {
    t1 = std::clamp(t1, 0.0, 1.0);
    t2 = std::clamp(t2, 0.0, 1.0);
    intersect_x = p1_x + t1 * u_x;
    intersect_y = p1_y + t1 * u_y;
    return true;
  }
  return false;
}

void SafeCorridorGenerator::initializeWithCostmap(
  SafeCorridor & corridor,
  const nav2_costmap_2d::Costmap2D * costmap,
  double current_d,
  const std::vector<double> & prev_planned_d) const
{
  if (corridor.bounds.empty()) {
    return;
  }

  if (!costmap || !config_.check_costmap) {
    for (auto & bound : corridor.bounds) {
      bound.d_max = config_.default_left_width;
      bound.d_min = -config_.default_right_width;
    }
    return;
  }

  double step_size = config_.costmap_ray_step;

  // Helper lambda to check if a lateral offset d is collision-free at a bound
  auto is_free = [&](const CorridorBound & bound, double d) -> bool {
    double norm_x = -std::sin(bound.theta);
    double norm_y = std::cos(bound.theta);
    double test_x = bound.x + d * norm_x;
    double test_y = bound.y + d * norm_y;
    unsigned int mx, my;
    if (!costmap->worldToMap(test_x, test_y, mx, my)) {
      return false;
    }
    return costmap->getCost(mx, my) < config_.costmap_cost_threshold;
  };

  // Helper lambda to expand bi-directionally from a seed d_seed on a bound
  // 严格从种子点向左(+d)和向右(-d)发射射线探测，遭遇首个障碍物即刻停止，阻断跨越障碍物的区间
  auto expand_from_seed = [&](const CorridorBound & bound, double seed) -> std::pair<double, double> {
    double d_max = seed;
    double d_min = seed;

    // 向左 (+d) 射线探测，遇障碍物即停，不构建障碍物后方不可达区域
    for (double d = seed + step_size; d <= config_.default_left_width + 1e-4; d += step_size) {
      if (is_free(bound, d)) {
        d_max = d;
      } else {
        break;
      }
    }

    // 向右 (-d) 射线探测，遇障碍物即停，不构建障碍物后方不可达区域
    for (double d = seed - step_size; d >= -config_.default_right_width - 1e-4; d -= step_size) {
      if (is_free(bound, d)) {
        d_min = d;
      } else {
        break;
      }
    }

    return {d_min, d_max};
  };

  // 拓扑分支状态：记录当前走廊所锁定的绕障分支侧 (UNKNOWN: 居中无偏好; LEFT: 锁定左侧; RIGHT: 锁定右侧)
  enum class BranchSide { UNKNOWN, LEFT, RIGHT };
  BranchSide active_branch = BranchSide::UNKNOWN;

  // 1. 优先根据上一轮规划轨迹判断已存在的绕障意图 (Temporal Branch Inheritance)
  if (!prev_planned_d.empty()) {
    double max_d = *std::max_element(prev_planned_d.begin(), prev_planned_d.end());
    double min_d = *std::min_element(prev_planned_d.begin(), prev_planned_d.end());
    if (max_d > 0.12 && std::abs(min_d) < max_d) {
      active_branch = BranchSide::LEFT;
    } else if (min_d < -0.12 && std::abs(max_d) < std::abs(min_d)) {
      active_branch = BranchSide::RIGHT;
    }
  }

  // 2. 若上一轮规划无明确偏好，根据机器人当前横向偏差初始化分支偏好
  if (active_branch == BranchSide::UNKNOWN) {
    if (current_d > 0.05) {
      active_branch = BranchSide::LEFT;
    } else if (current_d < -0.05) {
      active_branch = BranchSide::RIGHT;
    }
  }

  bool has_avoided_obstacle = false;
  double prev_min = -config_.default_right_width;
  double prev_max = config_.default_left_width;
  double prev_center = std::clamp(current_d, -config_.default_right_width, config_.default_left_width);

  for (size_t i = 0; i < corridor.bounds.size(); ++i) {
    auto & bound = corridor.bounds[i];

    if (i == 0) {
      // 截面 0：从机身当前匹配的点位 target_seed 开始进行连通空间构建
      double target_seed = std::clamp(current_d, -config_.default_right_width, config_.default_left_width);
      bool seed_found = false;

      if (is_free(bound, target_seed)) {
        auto interval = expand_from_seed(bound, target_seed);
        if (interval.second - interval.first >= config_.min_corridor_width) {
          bound.d_min = interval.first;
          bound.d_max = interval.second;
          seed_found = true;
        }
      }

      if (!seed_found) {
        // 若机器人当前位置落于障碍物或膨胀层内，从 target_seed 沿截面向两侧就近搜索首个自由种子
        for (double offset = step_size; offset <= config_.default_left_width + config_.default_right_width; offset += step_size) {
          double d1 = (active_branch == BranchSide::RIGHT) ? (target_seed - offset) : (target_seed + offset);
          double d2 = (active_branch == BranchSide::RIGHT) ? (target_seed + offset) : (target_seed - offset);

          if (d1 >= -config_.default_right_width && d1 <= config_.default_left_width && is_free(bound, d1)) {
            auto interval = expand_from_seed(bound, d1);
            if (interval.second - interval.first >= config_.min_corridor_width) {
              bound.d_min = interval.first;
              bound.d_max = interval.second;
              seed_found = true;
              break;
            }
          }
          if (d2 >= -config_.default_right_width && d2 <= config_.default_left_width && is_free(bound, d2)) {
            auto interval = expand_from_seed(bound, d2);
            if (interval.second - interval.first >= config_.min_corridor_width) {
              bound.d_min = interval.first;
              bound.d_max = interval.second;
              seed_found = true;
              break;
            }
          }
        }

        if (!seed_found) {
          // 整个截面均无有效通行空间，维持 target_seed 附近的窄带容错
          bound.d_min = std::clamp(target_seed - 0.05, -config_.default_right_width, config_.default_left_width);
          bound.d_max = std::clamp(target_seed + 0.05, -config_.default_right_width, config_.default_left_width);
        }
      }

      // 如果截面 0 已处于单侧避障收缩状态，锁定分支
      if (bound.d_min > 0.02) {
        active_branch = BranchSide::LEFT;
        has_avoided_obstacle = true;
      } else if (bound.d_max < -0.02) {
        active_branch = BranchSide::RIGHT;
        has_avoided_obstacle = true;
      }
    } else {
      // 截面 i > 0：沿着与上一截面物理连通的方向严格单侧延伸，直至截取路线的最远处
      double ds = std::max(0.01, bound.s - corridor.bounds[i - 1].s);
      double reach_margin = config_.max_lateral_rate * ds + step_size;
      double search_min = std::max(-config_.default_right_width, prev_min - reach_margin);
      double search_max = std::min(config_.default_left_width, prev_max + reach_margin);

      bool resolved = false;

      // 1. 优先测试上一轮规划点在当前截面的连通性 (Temporal Seed Expansion)
      if (i < prev_planned_d.size()) {
        double plan_seed = std::clamp(prev_planned_d[i], -config_.default_right_width, config_.default_left_width);
        if (is_free(bound, plan_seed)) {
          auto interval = expand_from_seed(bound, plan_seed);
          if (interval.second - interval.first >= config_.min_corridor_width) {
            if (interval.second >= search_min && interval.first <= search_max) {
              bound.d_min = interval.first;
              bound.d_max = interval.second;
              resolved = true;
            }
          }
        }
      }

      // 2. 若上一轮规划点受阻或未提供，测试上一截面中心在当前截面的连通性 (Spatial Continuity)
      if (!resolved && is_free(bound, prev_center)) {
        auto interval = expand_from_seed(bound, prev_center);
        if (interval.second - interval.first >= config_.min_corridor_width) {
          // 检查此区间是否与上一截面走廊物理重叠连通
          if (interval.second >= search_min && interval.first <= search_max) {
            bound.d_min = interval.first;
            bound.d_max = interval.second;
            resolved = true;
          }
        }
      }

      // 3. 若上一截面中心与上一轮规划点均受阻，在已锁定的单侧分支内搜索延伸
      if (!resolved) {
        // 分别在左侧和右侧搜索自由连通区间
        bool found_left = false;
        std::pair<double, double> left_interval;
        for (double d = std::max(-config_.default_right_width, prev_center + step_size); d <= config_.default_left_width + 1e-4; d += step_size) {
          if (is_free(bound, d)) {
            left_interval = expand_from_seed(bound, d);
            if (left_interval.second - left_interval.first >= config_.min_corridor_width) {
              found_left = true;
              break;
            }
          }
        }

        bool found_right = false;
        std::pair<double, double> right_interval;
        for (double d = std::min(config_.default_left_width, prev_center - step_size); d >= -config_.default_right_width - 1e-4; d -= step_size) {
          if (is_free(bound, d)) {
            right_interval = expand_from_seed(bound, d);
            if (right_interval.second - right_interval.first >= config_.min_corridor_width) {
              found_right = true;
              break;
            }
          }
        }

        if (found_left && found_right) {
          // 关键决断：两侧均有通行空间时，严禁因宽度比较而左右横跳！
          // 严格根据已锁定的分支方向 (active_branch) 或当前机器人位置选择
          if (active_branch == BranchSide::LEFT) {
            bound.d_min = left_interval.first;
            bound.d_max = left_interval.second;
          } else if (active_branch == BranchSide::RIGHT) {
            bound.d_min = right_interval.first;
            bound.d_max = right_interval.second;
          } else {
            // active_branch 为 UNKNOWN（首次分叉点）
            if (current_d > 0.02) {
              bound.d_min = left_interval.first;
              bound.d_max = left_interval.second;
              active_branch = BranchSide::LEFT;
            } else if (current_d < -0.02) {
              bound.d_min = right_interval.first;
              bound.d_max = right_interval.second;
              active_branch = BranchSide::RIGHT;
            } else {
              // 处于中线且无偏好时，选择更宽的一侧，并立即锁定该分支侧
              double width_left = left_interval.second - left_interval.first;
              double width_right = right_interval.second - right_interval.first;
              if (width_left >= width_right) {
                bound.d_min = left_interval.first;
                bound.d_max = left_interval.second;
                active_branch = BranchSide::LEFT;
              } else {
                bound.d_min = right_interval.first;
                bound.d_max = right_interval.second;
                active_branch = BranchSide::RIGHT;
              }
            }
          }
          resolved = true;
        } else if (found_left) {
          // 仅左侧连通：
          if (active_branch == BranchSide::RIGHT) {
            // 右侧受阻，保持上一截面中心窄带，严禁跨越障碍物
            bound.d_min = std::clamp(prev_center - 0.05, -config_.default_right_width, config_.default_left_width);
            bound.d_max = std::clamp(prev_center + 0.05, -config_.default_right_width, config_.default_left_width);
          } else {
            bound.d_min = left_interval.first;
            bound.d_max = left_interval.second;
            if (active_branch == BranchSide::UNKNOWN) {
              active_branch = BranchSide::LEFT;
            }
          }
          resolved = true;
        } else if (found_right) {
          // 仅右侧连通：
          if (active_branch == BranchSide::LEFT) {
            // 左侧受阻，保持上一截面中心窄带，严禁跨越障碍物
            bound.d_min = std::clamp(prev_center - 0.05, -config_.default_right_width, config_.default_left_width);
            bound.d_max = std::clamp(prev_center + 0.05, -config_.default_right_width, config_.default_left_width);
          } else {
            bound.d_min = right_interval.first;
            bound.d_max = right_interval.second;
            if (active_branch == BranchSide::UNKNOWN) {
              active_branch = BranchSide::RIGHT;
            }
          }
          resolved = true;
        } else {
          // 当前截面可达范围内被完全阻断：维持上一截面中心的窄带，严禁跨越障碍物
          bound.d_min = std::clamp(prev_center - 0.05, -config_.default_right_width, config_.default_left_width);
          bound.d_max = std::clamp(prev_center + 0.05, -config_.default_right_width, config_.default_left_width);
        }
      }

      // 3. 记录避障收缩状态与绕障完成检测
      if (bound.d_min > 0.02 || bound.d_max < -0.02) {
        has_avoided_obstacle = true;
      } else if (has_avoided_obstacle && bound.d_min <= -0.15 && bound.d_max >= 0.15) {
        // 障碍物已结束，走廊已重新包络参考中心线，重置分支状态
        active_branch = BranchSide::UNKNOWN;
        has_avoided_obstacle = false;
      }
    }

    prev_min = bound.d_min;
    prev_max = bound.d_max;
    prev_center = 0.5 * (bound.d_min + bound.d_max);
  }
}

void SafeCorridorGenerator::applyCurvatureLimit(SafeCorridor & corridor) const
{
  for (auto & bound : corridor.bounds) {
    if (std::abs(bound.kappa) < 1e-4) {
      continue;
    }

    double radius = 1.0 / std::abs(bound.kappa);
    double max_safe_inner_d = config_.curvature_safety_factor * radius;

    if (bound.kappa > 0.0) {
      // Left turn: curvature center is on the left (d > 0)
      bound.d_max = std::min(bound.d_max, max_safe_inner_d);
    } else {
      // Right turn: curvature center is on the right (d < 0)
      bound.d_min = std::max(bound.d_min, -max_safe_inner_d);
    }
  }
}

void SafeCorridorGenerator::preventAdjacentRibIntersection(SafeCorridor & corridor) const
{
  if (corridor.bounds.size() < 2) {
    return;
  }

  for (size_t i = 0; i + 1 < corridor.bounds.size(); ++i) {
    auto & b1 = corridor.bounds[i];
    auto & b2 = corridor.bounds[i + 1];

    double n1_x = -std::sin(b1.theta);
    double n1_y = std::cos(b1.theta);
    double n2_x = -std::sin(b2.theta);
    double n2_y = std::cos(b2.theta);

    double lambda1 = 0.0, lambda2 = 0.0;
    if (computeLineIntersection(b1.x, b1.y, n1_x, n1_y, b2.x, b2.y, n2_x, n2_y, lambda1, lambda2)) {
      if (lambda1 > 0.0 && lambda2 > 0.0) {
        // Intersection is on the left side
        double safe_limit = std::min(lambda1, lambda2) - config_.rib_safety_margin;
        if (safe_limit > 0.0) {
          b1.d_max = std::min(b1.d_max, safe_limit);
          b2.d_max = std::min(b2.d_max, safe_limit);
        }
      } else if (lambda1 < 0.0 && lambda2 < 0.0) {
        // Intersection is on the right side
        double safe_limit = std::max(lambda1, lambda2) + config_.rib_safety_margin;
        if (safe_limit < 0.0) {
          b1.d_min = std::max(b1.d_min, safe_limit);
          b2.d_min = std::max(b2.d_min, safe_limit);
        }
      }
    }
  }
}

void SafeCorridorGenerator::preventHairpinSelfIntersection(SafeCorridor & corridor) const
{
  if (!config_.avoid_hairpin_self_intersection || corridor.bounds.size() < 4) {
    return;
  }

  size_t N = corridor.bounds.size();

  for (int iter = 0; iter < config_.max_intersection_iters; ++iter) {
    updateCartesianBoundaries(corridor);
    bool modified = false;

    // Check non-adjacent cross-section ribs (j >= i + 2)
    for (size_t i = 0; i < N; ++i) {
      auto & bi = corridor.bounds[i];
      for (size_t j = i + 2; j < N; ++j) {
        auto & bj = corridor.bounds[j];

        // Fast bounding sphere rejection
        double center_dist_sq = std::pow(bi.x - bj.x, 2) + std::pow(bi.y - bj.y, 2);
        double max_reach_i = std::max(std::abs(bi.d_min), bi.d_max);
        double max_reach_j = std::max(std::abs(bj.d_min), bj.d_max);
        double max_reach = max_reach_i + max_reach_j + 0.1;
        if (center_dist_sq > max_reach * max_reach) {
          continue;
        }

        // Test intersection of cross-section line segments [Right_i, Left_i] and [Right_j, Left_j]
        double q_x = 0.0, q_y = 0.0, t1 = 0.0, t2 = 0.0;
        if (computeLineSegmentIntersection(
              bi.right_x, bi.right_y, bi.left_x, bi.left_y,
              bj.right_x, bj.right_y, bj.left_x, bj.left_y,
              q_x, q_y, t1, t2))
        {
          // Calculate signed lateral offsets of the intersection point Q with respect to both reference points
          double ni_x = -std::sin(bi.theta);
          double ni_y = std::cos(bi.theta);
          double nj_x = -std::sin(bj.theta);
          double nj_y = std::cos(bj.theta);

          double d_qi = (q_x - bi.x) * ni_x + (q_y - bi.y) * ni_y;
          double d_qj = (q_x - bj.x) * nj_x + (q_y - bj.y) * nj_y;

          // Shrink bound i
          if (d_qi > 0.0) {
            double new_d = std::max(config_.min_corridor_width, d_qi - config_.rib_safety_margin);
            if (new_d < bi.d_max) {
              bi.d_max = new_d;
              modified = true;
            }
          } else if (d_qi < 0.0) {
            double new_d = std::min(-config_.min_corridor_width, d_qi + config_.rib_safety_margin);
            if (new_d > bi.d_min) {
              bi.d_min = new_d;
              modified = true;
            }
          }

          // Shrink bound j
          if (d_qj > 0.0) {
            double new_d = std::max(config_.min_corridor_width, d_qj - config_.rib_safety_margin);
            if (new_d < bj.d_max) {
              bj.d_max = new_d;
              modified = true;
            }
          } else if (d_qj < 0.0) {
            double new_d = std::min(-config_.min_corridor_width, d_qj + config_.rib_safety_margin);
            if (new_d > bj.d_min) {
              bj.d_min = new_d;
              modified = true;
            }
          }
        }

        // Check left boundary self-intersection [L_i, L_{i+1}] with [L_j, L_{j+1}]
        if (i + 1 < N && j + 1 < N && j > i + 1) {
          const auto & bi_next = corridor.bounds[i + 1];
          const auto & bj_next = corridor.bounds[j + 1];
          if (computeLineSegmentIntersection(
                bi.left_x, bi.left_y, bi_next.left_x, bi_next.left_y,
                bj.left_x, bj.left_y, bj_next.left_x, bj_next.left_y,
                q_x, q_y, t1, t2))
          {
            bi.d_max = std::max(config_.min_corridor_width, bi.d_max * 0.8);
            corridor.bounds[i + 1].d_max = std::max(config_.min_corridor_width, corridor.bounds[i + 1].d_max * 0.8);
            bj.d_max = std::max(config_.min_corridor_width, bj.d_max * 0.8);
            corridor.bounds[j + 1].d_max = std::max(config_.min_corridor_width, corridor.bounds[j + 1].d_max * 0.8);
            modified = true;
          }

          // Check right boundary self-intersection [R_i, R_{i+1}] with [R_j, R_{j+1}]
          if (computeLineSegmentIntersection(
                bi.right_x, bi.right_y, bi_next.right_x, bi_next.right_y,
                bj.right_x, bj.right_y, bj_next.right_x, bj_next.right_y,
                q_x, q_y, t1, t2))
          {
            bi.d_min = std::min(-config_.min_corridor_width, bi.d_min * 0.8);
            corridor.bounds[i + 1].d_min = std::min(-config_.min_corridor_width, corridor.bounds[i + 1].d_min * 0.8);
            bj.d_min = std::min(-config_.min_corridor_width, bj.d_min * 0.8);
            corridor.bounds[j + 1].d_min = std::min(-config_.min_corridor_width, corridor.bounds[j + 1].d_min * 0.8);
            modified = true;
          }

          // Check left boundary with right boundary cross-intersection [L_i, L_{i+1}] with [R_j, R_{j+1}]
          if (computeLineSegmentIntersection(
                bi.left_x, bi.left_y, bi_next.left_x, bi_next.left_y,
                bj.right_x, bj.right_y, bj_next.right_x, bj_next.right_y,
                q_x, q_y, t1, t2))
          {
            bi.d_max = std::max(config_.min_corridor_width, bi.d_max * 0.8);
            corridor.bounds[i + 1].d_max = std::max(config_.min_corridor_width, corridor.bounds[i + 1].d_max * 0.8);
            bj.d_min = std::min(-config_.min_corridor_width, bj.d_min * 0.8);
            corridor.bounds[j + 1].d_min = std::min(-config_.min_corridor_width, corridor.bounds[j + 1].d_min * 0.8);
            modified = true;
          }
        }
      }
    }

    if (!modified) {
      break;
    }
  }
}

void SafeCorridorGenerator::smoothCorridorBounds(SafeCorridor & corridor) const
{
  if (corridor.bounds.size() < 2) {
    return;
  }

  size_t N = corridor.bounds.size();

  // Forward filter: restrict max rate of change of d along s
  for (size_t i = 1; i < N; ++i) {
    double ds = std::max(0.01, corridor.bounds[i].s - corridor.bounds[i - 1].s);
    double max_delta = config_.max_lateral_rate * ds;

    corridor.bounds[i].d_max = std::min(corridor.bounds[i].d_max, corridor.bounds[i - 1].d_max + max_delta);
    corridor.bounds[i].d_min = std::max(corridor.bounds[i].d_min, corridor.bounds[i - 1].d_min - max_delta);
  }

  // Backward filter: backward consistency
  for (int i = static_cast<int>(N) - 2; i >= 0; --i) {
    double ds = std::max(0.01, corridor.bounds[i + 1].s - corridor.bounds[i].s);
    double max_delta = config_.max_lateral_rate * ds;

    corridor.bounds[i].d_max = std::min(corridor.bounds[i].d_max, corridor.bounds[i + 1].d_max + max_delta);
    corridor.bounds[i].d_min = std::max(corridor.bounds[i].d_min, corridor.bounds[i + 1].d_min - max_delta);
  }

  // 保证走廊两边界关系合法 (d_max >= d_min + 0.05)
  for (auto & bound : corridor.bounds) {
    if (bound.d_max < bound.d_min + 0.05) {
      double mid = 0.5 * (bound.d_max + bound.d_min);
      bound.d_max = mid + 0.025;
      bound.d_min = mid - 0.025;
    }
  }
}

void SafeCorridorGenerator::updateCartesianBoundaries(SafeCorridor & corridor) const
{
  for (auto & bound : corridor.bounds) {
    double norm_x = -std::sin(bound.theta);
    double norm_y = std::cos(bound.theta);

    bound.left_x = bound.x + bound.d_max * norm_x;
    bound.left_y = bound.y + bound.d_max * norm_y;
    bound.right_x = bound.x + bound.d_min * norm_x;
    bound.right_y = bound.y + bound.d_min * norm_y;
  }
}

SafeCorridor SafeCorridorGenerator::generateCorridor(
  const std::vector<TrajectoryPoint> & ref_points,
  const nav2_costmap_2d::Costmap2D * costmap,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  double s_to_goal,
  double current_d,
  const std::vector<double> & prev_planned_d)
{
  SafeCorridor corridor;
  corridor.frame_id = frame_id;
  corridor.stamp = stamp;

  if (ref_points.empty()) {
    return corridor;
  }

  corridor.bounds.resize(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & pt = ref_points[i];
    auto & bound = corridor.bounds[i];
    bound.s = pt.s;
    bound.x = pt.x;
    bound.y = pt.y;
    bound.theta = pt.theta;
    bound.kappa = pt.kappa;
    bound.v = pt.v;
    bound.t = pt.t;
  }

  // 1. Initial raycasting against costmap obstacles with reachable connectivity expansion & temporal guidance
  initializeWithCostmap(corridor, costmap, current_d, prev_planned_d);

  // 2. Prevent curvature center singularity (1 - kappa * d > 0)
  applyCurvatureLimit(corridor);

  // 3. Prevent adjacent rib intersection
  preventAdjacentRibIntersection(corridor);

  // 4. Prevent hairpin / U-turn self-intersection
  preventHairpinSelfIntersection(corridor);

  // 5. Smooth corridor bounds along s
  smoothCorridorBounds(corridor);

  // 6. 终点高精几何吸附漏斗 (Goal Approach Funnel Snapping)
  // 仅当机器人明确进入终点进近阶段 (s_to_goal >= 0 且 s_to_goal < config_.goal_approach_dist) 时，
  // 将走廊平滑漏斗化收拢到终点目标容差 (terminal_d_tol)；未到终点时保持两边平行状态。
  if (!corridor.bounds.empty() && s_to_goal >= 0.0 && s_to_goal < config_.goal_approach_dist) {
    double approach_dist = config_.goal_approach_dist;
    double terminal_d_tol = config_.terminal_d_tol;

    for (auto & bound : corridor.bounds) {
      double s_from_start = bound.s - corridor.bounds.front().s;
      double s_rem = s_to_goal - s_from_start;
      if (s_rem < 0.0) {
        s_rem = 0.0;
      }
      if (s_rem < approach_dist) {
        double r = s_rem / approach_dist; // 0.0 at goal, 1.0 at approach entry
        double funnel_max = terminal_d_tol + r * (config_.default_left_width - terminal_d_tol);
        double funnel_min = -terminal_d_tol + r * (-config_.default_right_width + terminal_d_tol);
        bound.d_max = std::min(bound.d_max, funnel_max);
        bound.d_min = std::max(bound.d_min, funnel_min);
        if (bound.d_max < bound.d_min + 0.03) {
          double mid = 0.5 * (bound.d_max + bound.d_min);
          bound.d_max = mid + 0.015;
          bound.d_min = mid - 0.015;
        }
      }
    }
  }

  // 7. Compute final Cartesian boundary coordinates
  updateCartesianBoundaries(corridor);

  return corridor;
}

visualization_msgs::msg::MarkerArray SafeCorridorGenerator::createVisualizationMarkers(
  const SafeCorridor & corridor) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 1. Delete all previous markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (corridor.bounds.size() < 2) {
    return marker_array;
  }

  std_msgs::msg::Header header;
  header.frame_id = corridor.frame_id;
  header.stamp = corridor.stamp;

  // 2. Corridor 3D Mesh Polygon Fill (TRIANGLE_LIST)
  visualization_msgs::msg::Marker mesh_marker;
  mesh_marker.header = header;
  mesh_marker.ns = "safe_corridor_mesh";
  mesh_marker.id = 0;
  mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  mesh_marker.action = visualization_msgs::msg::Marker::ADD;
  mesh_marker.scale.x = 1.0;
  mesh_marker.scale.y = 1.0;
  mesh_marker.scale.z = 1.0;
  mesh_marker.color.r = 0.0;
  mesh_marker.color.g = 0.75;
  mesh_marker.color.b = 0.95;
  mesh_marker.color.a = 0.35; // Translucent cyan

  // 3. Left Boundary Line (LINE_STRIP)
  visualization_msgs::msg::Marker left_line_marker;
  left_line_marker.header = header;
  left_line_marker.ns = "safe_corridor_left_boundary";
  left_line_marker.id = 1;
  left_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_line_marker.action = visualization_msgs::msg::Marker::ADD;
  left_line_marker.scale.x = 0.025; // 2.5cm line width
  left_line_marker.color.r = 0.1;
  left_line_marker.color.g = 0.95;
  left_line_marker.color.b = 0.25;
  left_line_marker.color.a = 0.9; // Bright green

  // 4. Right Boundary Line (LINE_STRIP)
  visualization_msgs::msg::Marker right_line_marker;
  right_line_marker.header = header;
  right_line_marker.ns = "safe_corridor_right_boundary";
  right_line_marker.id = 2;
  right_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  right_line_marker.action = visualization_msgs::msg::Marker::ADD;
  right_line_marker.scale.x = 0.025;
  right_line_marker.color.r = 1.0;
  right_line_marker.color.g = 0.55;
  right_line_marker.color.b = 0.0;
  right_line_marker.color.a = 0.9; // Bright orange

  // 5. Cross-Section Ribs (LINE_LIST)
  visualization_msgs::msg::Marker ribs_marker;
  ribs_marker.header = header;
  ribs_marker.ns = "safe_corridor_ribs";
  ribs_marker.id = 3;
  ribs_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  ribs_marker.action = visualization_msgs::msg::Marker::ADD;
  ribs_marker.scale.x = 0.012;
  ribs_marker.color.r = 1.0;
  ribs_marker.color.g = 1.0;
  ribs_marker.color.b = 1.0;
  ribs_marker.color.a = 0.6; // Subtle white ribs

  // 6. Center Reference Line (LINE_STRIP)
  visualization_msgs::msg::Marker center_line_marker;
  center_line_marker.header = header;
  center_line_marker.ns = "safe_corridor_center_line";
  center_line_marker.id = 4;
  center_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  center_line_marker.action = visualization_msgs::msg::Marker::ADD;
  center_line_marker.scale.x = 0.015;
  center_line_marker.color.r = 1.0;
  center_line_marker.color.g = 0.95;
  center_line_marker.color.b = 0.1;
  center_line_marker.color.a = 0.8; // Bright yellow

  size_t N = corridor.bounds.size();
  for (size_t i = 0; i < N; ++i) {
    const auto & b = corridor.bounds[i];

    geometry_msgs::msg::Point p_left, p_right, p_center;
    p_left.x = b.left_x;
    p_left.y = b.left_y;
    p_left.z = 0.01;

    p_right.x = b.right_x;
    p_right.y = b.right_y;
    p_right.z = 0.01;

    p_center.x = b.x;
    p_center.y = b.y;
    p_center.z = 0.012;

    left_line_marker.points.push_back(p_left);
    right_line_marker.points.push_back(p_right);
    center_line_marker.points.push_back(p_center);

    // Cross-section rib
    ribs_marker.points.push_back(p_right);
    ribs_marker.points.push_back(p_left);

    // Triangles for corridor surface mesh
    if (i + 1 < N) {
      const auto & b_next = corridor.bounds[i + 1];
      geometry_msgs::msg::Point p_next_left, p_next_right;
      p_next_left.x = b_next.left_x;
      p_next_left.y = b_next.left_y;
      p_next_left.z = 0.005;

      p_next_right.x = b_next.right_x;
      p_next_right.y = b_next.right_y;
      p_next_right.z = 0.005;

      // Triangle 1: (p_left, p_right, p_next_left)
      mesh_marker.points.push_back(p_left);
      mesh_marker.points.push_back(p_right);
      mesh_marker.points.push_back(p_next_left);

      // Triangle 2: (p_right, p_next_right, p_next_left)
      mesh_marker.points.push_back(p_right);
      mesh_marker.points.push_back(p_next_right);
      mesh_marker.points.push_back(p_next_left);
    }
  }

  marker_array.markers.push_back(mesh_marker);
  marker_array.markers.push_back(left_line_marker);
  marker_array.markers.push_back(right_line_marker);
  marker_array.markers.push_back(ribs_marker);
  marker_array.markers.push_back(center_line_marker);

  return marker_array;
}

}  // namespace nav2_mpc_controller
