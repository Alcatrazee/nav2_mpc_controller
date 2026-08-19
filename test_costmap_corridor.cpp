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
    std::cout << "\n>>> ALL UNCONSTRAINED CORRIDOR TESTS PASSED! <<<" << std::endl;
    return 0;
}
