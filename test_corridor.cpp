#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "safe_corridor_generator.hpp"

using namespace nav2_mpc_controller;

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "Testing SafeCorridorGenerator on Hairpin / U-turn" << std::endl;
    std::cout << "==================================================" << std::endl;

    // 1. 构造一个 180 度掉头弯测试轨迹
    // 直线 (0,0) -> (4,0), 半圆 R=1.0 从 (4,0) 转到 (4,2), 直线 (4,2) -> (0,2)
    std::vector<TrajectoryPoint> test_traj;
    double s = 0.0;
    double v = 0.5;

    // 前段直线 (4m)
    for (double x = 0.0; x <= 4.0; x += 0.1) {
        TrajectoryPoint p;
        p.x = x;
        p.y = 0.0;
        p.theta = 0.0;
        p.kappa = 0.0;
        p.s = s;
        p.v = v;
        p.t = s / v;
        test_traj.push_back(p);
        s += 0.1;
    }

    // 掉头半圆 R=1.0 (中心在 (4, 1.0), 角度从 -pi/2 到 pi/2)
    double R = 1.0;
    double kappa = 1.0 / R; // 左转弯，kappa > 0
    for (double phi = -M_PI / 2.0 + 0.1; phi <= M_PI / 2.0; phi += 0.1) {
        TrajectoryPoint p;
        p.x = 4.0 + R * std::cos(phi);
        p.y = 1.0 + R * std::sin(phi);
        p.theta = phi + M_PI / 2.0; // 切线方向
        p.kappa = kappa;
        p.s = s;
        p.v = v;
        p.t = s / v;
        test_traj.push_back(p);
        s += R * 0.1;
    }

    // 后段反向直线 (4m)
    for (double x = 4.0; x >= 0.0; x -= 0.1) {
        TrajectoryPoint p;
        p.x = x;
        p.y = 2.0;
        p.theta = M_PI;
        p.kappa = 0.0;
        p.s = s;
        p.v = v;
        p.t = s / v;
        test_traj.push_back(p);
        s += 0.1;
    }

    std::cout << "Generated U-turn reference path with " << test_traj.size() << " points." << std::endl;

    // 2. 配置 SafeCorridorGenerator (设置默认单侧宽度为 1.0m，大于掉头间隙 2.0m 的一半)
    SafeCorridorConfig config;
    config.default_left_width = 1.0;
    config.default_right_width = 1.0;
    config.min_corridor_width = 0.15;
    config.curvature_safety_factor = 0.8;
    config.rib_safety_margin = 0.05;
    config.avoid_hairpin_self_intersection = true;
    config.check_costmap = false;

    SafeCorridorGenerator generator(config);
    SafeCorridor corridor = generator.generateCorridor(test_traj, nullptr, "map");

    std::cout << "Constructed Safe Corridor with " << corridor.size() << " bounds." << std::endl;

    // 3. 验证关键性质
    bool singularity_free = true;
    bool valid_bounds = true;

    for (size_t i = 0; i < corridor.bounds.size(); ++i) {
        const auto & b = corridor.bounds[i];
        // 检查 Frenet 坐标系奇异性条件 1 - kappa * d > 0
        if (b.kappa > 0 && (1.0 - b.kappa * b.d_max) <= 0.0) {
            std::cerr << "Singularity violation at index " << i << ": kappa=" << b.kappa << ", d_max=" << b.d_max << std::endl;
            singularity_free = false;
        }
        if (b.d_min > 0.0 || b.d_max < 0.0 || b.d_max < b.d_min) {
            std::cerr << "Invalid bound order at index " << i << ": d_min=" << b.d_min << ", d_max=" << b.d_max << std::endl;
            valid_bounds = false;
        }
    }

    std::cout << "Singularity Check: " << (singularity_free ? "PASSED" : "FAILED") << std::endl;
    std::cout << "Valid Bounds Check: " << (valid_bounds ? "PASSED" : "FAILED") << std::endl;

    // 4. 统计掉头弯处的宽度缩减情况
    std::cout << "\nSample Corridor Bounds in Hairpin curve:" << std::endl;
    for (size_t i = 0; i < corridor.bounds.size(); i += 8) {
        const auto & b = corridor.bounds[i];
        std::cout << "  idx=" << i 
                  << " | s=" << b.s 
                  << " | (x, y)=(" << b.x << ", " << b.y << ")"
                  << " | kappa=" << b.kappa
                  << " | d in [" << b.d_min << ", " << b.d_max << "]"
                  << " | Left=(" << b.left_x << ", " << b.left_y << ")"
                  << " | Right=(" << b.right_x << ", " << b.right_y << ")"
                  << std::endl;
    }

    assert(singularity_free && valid_bounds);
    std::cout << "\nALL SAFE CORRIDOR TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
