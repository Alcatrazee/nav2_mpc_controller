# Nav2 MPC Controller

这是一个基于 **Frenet 坐标系**、**CasADi C++ Function 预编译** 和 **IPOPT** 高速求解的模型预测控制 (Frenet-MPC) 局部规划器插件，适用于差速驱动机器人。

## 核心特性与路径处理流水线

本控制器在接收到全局路径后，执行以下高度优化的处理流水线：

1. **坐标系转换与局部路径截取 (Local Plan Extraction)**：
   将全局路径从全局坐标系转换至控制器的局部坐标系。计算距离机器人当前位姿最近的路径点，截取从该点开始的前方路径，并自动裁剪掉超出当前局部代价地图 (Costmap) 范围的路径。

2. **C2 自然三次样条平滑与解析采样 (C2 Cubic B-Spline Smoothing & Analytical Sampling)**：
   提取去重后的路径点并计算累积弧长 $s$。使用一维 **C2 自然三次样条曲线 ($x(s), y(s)$)** 构建连续路径，以 `0.05m` 高分辨率计算高阶导数，解析计算切线姿态 $\theta = \text{atan2}(y', x')$ 以及精确曲率 $\kappa = \frac{x' y'' - y' x''}{(x'^2 + y'^2)^{3/2}}$，彻底消除高阶多项式拟合带来的龙格震荡。

3. **时间参数化与速度规划 (Time-Parameterized Velocity Profiler)**：
   根据设定的加速度约束（最大线加速度 $a_{\max}$、最小线加速度 $a_{\min}$）与速度上限 $v_{\max}$，进行前向加速与后向减速扫描，生成带时间戳 $t$ 与目标速度 $v$ 的平滑参考轨迹。

4. **Frenet 状态转换与等时参考点采样 (Frenet State Transformation & Sampling)**：
   将机器人当前 Cartesian 位姿 $(x, y, \theta)$ 精确投影到参考轨迹上，提取 Frenet 状态量 $(s, d, e_\psi, v)$。根据预测时域 $N$ 与步长 $dt$，按 $t = k \times dt$ 从规划轨迹中采样出未来的参考弧长 $s_{\text{ref}}$、参考速度 $v_{\text{ref}}$ 和参考曲率 $\kappa_{\text{ref}}$。

5. **CasADi / IPOPT 高速 Function 预编译求解 (High-Speed CasADi Function Solver)**：
   建立 Frenet 差分运动学模型：
   $$\dot{s} = \frac{v \cos(e_\psi)}{1 - \kappa_r d}, \quad \dot{d} = v \sin(e_\psi), \quad \dot{e}_\psi = \omega - \kappa_r \dot{s}, \quad \dot{v} = a$$
   通过 `casadi::Opti::to_function()` 在初始化阶段将图求解预编译为 C++ Function 计算图，彻底消除运行时每帧重新构造优化问题的开销。求解器使用 IPOPT 的精确海森矩阵 (`exact` Hessian)，实现 3 步二次收敛（微秒级求解时间，调度频率达 $\ge 20\text{Hz}$）。

6. **输出平滑与角速度斜率限制 (Slew-Rate Limiting)**：
   对输出角速度使用斜率限制器 (Slew-Rate Limiter, 最大变化量 `0.35 rad` / `0.1s step`)，彻底消除了出弯及高频扰动引发的方向盘打抖，保证机器人平滑顺畅运行。

## 依赖项 (Dependencies)

- **ROS 2** (Nav2, tf2, geometry_msgs, nav_msgs, visualization_msgs, std_msgs)
- **CasADi** (含 IPOPT 求解器)
- **Eigen3**

## 参数配置 (nav2_params.yaml)

请将以下参数复制到你的 `nav2_params.yaml` 文件中 `controller_server` 节点下的具体控制器配置块内：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mpc_controller::MPCController"
      
      # --- MPC 预测与时间步长参数 ---
      N: 10            # MPC 预测时域步数 (默认: 10)
      dt: 0.1          # MPC 预测离散步长，单位：s (默认: 0.1)
      
      # --- 速度与加速度约束参数 ---
      v_max: 0.5       # 最大前向线速度，单位：m/s (默认: 0.5)
      v_min: 0.0       # 最小前向线速度，单位：m/s (默认: 0.0)
      w_max: 1.0       # 最大角速度，单位：rad/s (默认: 1.0)
      w_min: -1.0      # 最小角速度，单位：rad/s (默认: -1.0)
      a_max: 1.0       # 最大线加速度，单位：m/s² (默认: 1.0)
      a_min: -1.0      # 最小线加速度 / 最大减速度，单位：m/s² (默认: -1.0)
      
      # --- Frenet 状态与控制代价权重参数 ---
      q_s: 2.0         # Frenet 纵向弧长 s 误差惩罚权重
      q_d: 20.0        # Frenet 横向偏移 d 误差惩罚权重 (高权重优先保证贴线)
      q_e_psi: 5.0     # Frenet 航向角误差 e_psi 惩罚权重
      r_v: 1.0         # 线速度追踪误差惩罚权重
      r_w: 0.5         # 角速度控制量惩罚权重
      r_a: 0.2         # 线加速度控制量惩罚权重
```

### 参数说明表

| 参数名 | 类型 | 默认值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| `N` | `int` | `10` | - | MPC 预测步数 |
| `dt` | `double` | `0.1` | `s` | MPC 预测离散单步时长 |
| `v_max` | `double` | `0.5` | `m/s` | 允许的最大线速度 |
| `v_min` | `double` | `0.0` | `m/s` | 允许的最小线速度 |
| `w_max` | `double` | `1.0` | `rad/s` | 允许的最大正向角速度 |
| `w_min` | `double` | `-1.0` | `rad/s` | 允许的最大反向角速度 |
| `a_max` | `double` | `1.0` | `m/s²` | 允许的最大线加速度 |
| `a_min` | `double` | `-1.0` | `m/s²` | 允许的最大线减速度 |
| `q_s` | `double` | `2.0` | - | Frenet 纵向位置误差权重 |
| `q_d` | `double` | `20.0` | - | Frenet 横向偏移误差权重 |
| `q_e_psi` | `double` | `5.0` | - | Frenet 航向姿态误差权重 |
| `r_v` | `double` | `1.0` | - | 速度跟踪偏差权重 |
| `r_w` | `double` | `0.5` | - | 角速度输入代价权重 |
| `r_a` | `double` | `0.2` | - | 加速度输入代价权重 |

## 话题接口 (Topics)

### 发布话题 (Publishers)
- `predict_trajectory` (`nav_msgs/msg/Path`): MPC 预测轨迹（由 Frenet 状态解转换回笛卡尔坐标发布）
- `transformed_global_plan` (`nav_msgs/msg/Path`): 转换到控制器坐标系下的全局路径
- `transformed_local_plan` (`nav_msgs/msg/Path`): 用于局部追踪截取的局部路径
- `local_plan` (`nav_msgs/msg/Path`): 时间参数化与等时采样后的局部参考路径
- `local_plan_markers` (`visualization_msgs/msg/MarkerArray`): 速度向量彩色箭头（绿->红代表速度大小）
- `lateral_error` (`std_msgs/msg/Float64`): 机器人实时的横向偏移误差 $d$（单位：米）

