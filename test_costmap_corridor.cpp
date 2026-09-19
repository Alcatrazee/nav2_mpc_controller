#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <cassert>
#include "safe_corridor_generator.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace nav2_mpc_controller;

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "Testing Unconstrained SafeCorridor (Full Cross-Section)" << std::endl;
    std::cout << "========================================================" << std::endl;

    // 1. 创建一个 10m x 6m 的真实 Costmap2D
    unsigned int size_x = 200; // 10.0m
    unsigned int size_y = 120; // 6.0m
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = -3.0;
    nav2_costmap_2d::Costmap2D costmap(size_x, size_y, resolution, origin_x, origin_y, nav2_costmap_2d::FREE_SPACE);

    // 2. 放置障碍物：
    // 障碍物 1: 直接穿透阻挡原参考线 y=0 (x in [3.0, 5.5], y in [-0.60, 0.15])
    // 障碍物 2: 偏向左侧 (x in [7.0, 9.0], y in [-0.10, 0.60])
    for (unsigned int mx = 0; mx < size_x; ++mx) {
        for (unsigned int my = 0; my < size_y; ++my) {
            double wx, wy;
            costmap.mapToWorld(mx, my, wx, wy);

            // 障碍物 1: 挡住 y=0, 右侧全堵, 通行区在 y in [0.20, 0.80]
            if (wx >= 3.0 && wx <= 5.5 && wy >= -0.60 && wy <= 0.15) {
                costmap.setCost(mx, my, 253);
            }

            // 障碍物 2: 挡住 y=0, 左侧全堵, 通行区在 y in [-0.80, -0.15]
            if (wx >= 7.0 && wx <= 9.0 && wy >= -0.10 && wy <= 0.60) {
                costmap.setCost(mx, my, 253);
            }
        }
    }

    // 3. 构建一条沿 x 轴向前直线行驶的参考轨迹 (y=0, 长度 9.5m)
    std::vector<TrajectoryPoint> ref_traj;
    double s = 0.0;
    double v = 0.5;
    for (double x = 0.5; x <= 9.5; x += 0.1) {
        TrajectoryPoint p;
        p.x = x;
        p.y = 0.0;
        p.theta = 0.0;
        p.kappa = 0.0;
        p.s = s;
        p.v = v;
        p.t = s / v;
        ref_traj.push_back(p);
        s += 0.1;
    }

    // 4. 配置 SafeCorridorGenerator
    SafeCorridorConfig config;
    config.default_left_width = 0.80;
    config.default_right_width = 0.80;
    config.min_corridor_width = 0.15;
    config.costmap_ray_step = 0.02;         // 2cm 采样
    config.costmap_cost_threshold = 253;    // 响应 253 代价
    config.check_costmap = true;
    config.max_lateral_rate = 0.40;

    SafeCorridorGenerator generator(config);
    SafeCorridor corridor = generator.generateCorridor(ref_traj, &costmap, "map");

    std::cout << "Corridor generated with " << corridor.size() << " bounds." << std::endl;

    // 5. 验证跨参考线自适应压缩与平移
    bool shifted_to_left = false;
    bool shifted_to_right = false;

    // 导出走廊数据
    std::ofstream out("/home/michael/turtlebot_ws/src/nav2_controller_template/corridor_costmap_data.csv");
    out << "s,x,y,theta,d_min,d_max,left_x,left_y,right_x,right_y\n";

    for (const auto & b : corridor.bounds) {
        out << b.s << "," << b.x << "," << b.y << "," << b.theta << ","
            << b.d_min << "," << b.d_max << ","
            << b.left_x << "," << b.left_y << ","
            << b.right_x << "," << b.right_y << "\n";

        // 在障碍物 1 区域，走廊右边界 d_min 应该被推到正数 (d_min > 0)
        if (b.x >= 3.5 && b.x <= 5.0) {
            if (b.d_min > 0.05 && b.d_max >= 0.70) {
                shifted_to_left = true;
            }
        }
        // 在障碍物 2 区域，走廊左边界 d_max 应该被推到负数 (d_max < 0)
        if (b.x >= 7.5 && b.x <= 8.5) {
            if (b.d_max < -0.05 && b.d_min <= -0.70) {
                shifted_to_right = true;
            }
        }
    }
    out.close();

    std::cout << "Obstacle 1 Zone: Shifted & Compressed to Left (d_min > 0): " 
              << (shifted_to_left ? "PASSED (d_min > 0)" : "FAILED") << std::endl;
    std::cout << "Obstacle 2 Zone: Shifted & Compressed to Right (d_max < 0): " 
              << (shifted_to_right ? "PASSED (d_max < 0)" : "FAILED") << std::endl;

    assert(shifted_to_left && shifted_to_right);

    // ========================================================
    // 6. Test Case 2: 验证障碍物隔断时，严禁构建不可达一侧
    // 在 x in [1.0, 3.0], y in [-0.15, 0.15] 放置中央隔离障碍物
    // 机器人位于右侧 (current_d = -0.40) 时，只构建右侧走廊，绝不构建不可达的左侧 [0.15, 0.80]
    // 机器人位于左侧 (current_d = +0.40) 时，只构建左侧走廊，绝不构建不可达的右侧 [-0.80, -0.15]
    // ========================================================
    std::cout << "\n========================================================" << std::endl;
    std::cout << "Testing Reachability: Do NOT Construct Unreachable Side" << std::endl;
    std::cout << "========================================================" << std::endl;

    nav2_costmap_2d::Costmap2D split_costmap(100, 100, 0.05, 0.0, -2.5, nav2_costmap_2d::FREE_SPACE);
    for (unsigned int mx = 0; mx < 100; ++mx) {
        for (unsigned int my = 0; my < 100; ++my) {
            double wx, wy;
            split_costmap.mapToWorld(mx, my, wx, wy);
            // 中央隔离障碍物: y in [-0.15, 0.15], x in [1.0, 3.0]
            if (wx >= 1.0 && wx <= 3.0 && wy >= -0.15 && wy <= 0.15) {
                split_costmap.setCost(mx, my, 253);
            }
        }
    }

    std::vector<TrajectoryPoint> short_traj;
    s = 0.0;
    for (double x = 0.0; x <= 4.0; x += 0.1) {
        TrajectoryPoint p;
        p.x = x;
        p.y = 0.0;
        p.theta = 0.0;
        p.kappa = 0.0;
        p.s = s;
        p.v = 0.5;
        p.t = s / 0.5;
        short_traj.push_back(p);
        s += 0.1;
    }

    // 6.1 机器人位于右侧 (current_d = -0.40)
    SafeCorridor corridor_right = generator.generateCorridor(
        short_traj, &split_costmap, "map", rclcpp::Time(), -1.0, -0.40);

    bool right_case_ok = true;
    for (const auto & b : corridor_right.bounds) {
        if (b.x >= 1.2 && b.x <= 2.8) {
            // 应该只在右侧: d_max <= -0.10, d_min <= -0.70
            // 绝不能构建左侧 (d_max 绝不能 > 0.10)
            if (b.d_max > -0.05 || b.d_min > -0.70) {
                right_case_ok = false;
                std::cerr << "Right case failed at x=" << b.x << ": d_min=" << b.d_min << ", d_max=" << b.d_max << std::endl;
            }
        }
    }
    std::cout << "Robot at Right (current_d = -0.40): Only Right constructed: "
              << (right_case_ok ? "PASSED" : "FAILED") << std::endl;
    assert(right_case_ok);

    // 6.2 机器人位于左侧 (current_d = +0.40)
    SafeCorridor corridor_left = generator.generateCorridor(
        short_traj, &split_costmap, "map", rclcpp::Time(), -1.0, 0.40);

    bool left_case_ok = true;
    for (const auto & b : corridor_left.bounds) {
        if (b.x >= 1.2 && b.x <= 2.8) {
            // 应该只在左侧: d_min >= 0.10, d_max >= 0.70
            // 绝不能构建右侧 (d_min 绝不能 < -0.10)
            if (b.d_min < 0.05 || b.d_max < 0.70) {
                left_case_ok = false;
                std::cerr << "Left case failed at x=" << b.x << ": d_min=" << b.d_min << ", d_max=" << b.d_max << std::endl;
            }
        }
    }
    std::cout << "Robot at Left (current_d = +0.40): Only Left constructed: "
              << (left_case_ok ? "PASSED" : "FAILED") << std::endl;
    assert(left_case_ok);

    std::cout << "\n>>> ALL UNCONSTRAINED & REACHABLE CORRIDOR TESTS PASSED! <<<" << std::endl;
    return 0;
}
