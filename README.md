# Nav2 MPC Controller

这是一个基于 **Frenet 坐标系**、**LTV-MPC 凸优化**、**非均匀时间步长 (Variable dt)** 与 **CasADi + OSQP 求解器** 极速求解的高性能模型预测控制局部规划器插件，适用于差速及阿克曼驱动机器人。即使在 **$40\text{Hz}$ 超高控制频率** 与 **$2.5\text{s}$ 超长前瞻视野** 下，单帧求解时间依然稳定在 **$1 \sim 3\text{ms}$**（远低于 25ms 控制周期预算），并配备了时移热启动（Shifted Warm Start）与防横跳单侧拓扑安全走廊！

---

## 核心特性与路径处理流水线

本控制器在接收到全局路径后，执行以下高度优化的处理流水线：

1. **坐标系转换与局部路径截取 (Local Plan Extraction)**：
   将全局路径从全局坐标系转换至控制器的局部坐标系。计算距离机器人当前位姿最近的路径点，截取从该点开始的前方路径，并自动裁剪掉超出当前局部代价地图 (Costmap) 范围的路径。

2. **C2 自然三次样条平滑与解析采样 (C2 Cubic B-Spline Smoothing & Analytical Sampling)**：
   提取去重后的路径点并计算累积弧长 $s$。使用一维 **C2 自然三次样条曲线 ($x(s), y(s)$)** 构建连续路径，以 `0.05m` 高分辨率计算高阶导数，解析计算切线姿态 $\theta = \text{atan2}(y', x')$ 以及精确曲率 $\kappa = \frac{x' y'' - y' x''}{(x'^2 + y'^2)^{3/2}}$，彻底消除高阶多项式拟合带来的龙格震荡。

3. **时间参数化与速度规划 (Time-Parameterized Velocity Profiler)**：
   根据设定的加速度约束（最大线加速度 $a_{\max}$、最小线加速度 $a_{\min}$）与速度上限 $v_{\max}$，进行前向加速与后向减速扫描，生成带时间戳 $t$ 与目标速度 $v$ 的平滑参考轨迹。

4. **非均匀时间步长与等时参考点采样 (Variable dt Sampling)**：
   支持非均匀时间步长（近密远疏：近端高精采样保证即时控制精度，远端大步长覆盖长达 $2.5\text{s}$ 前瞻视野以提早感知障碍物）。将机器人当前 Cartesian 位姿 $(x, y, \theta)$ 精确投影到参考轨迹上，提取 Frenet 状态量 $(s, d, e_\psi, v)$，按累计时间戳 $t_k$ 从规划轨迹中采样出未来的参考弧长 $s_{\text{ref}}$、参考速度 $v_{\text{ref}}$ 和参考曲率 $\kappa_{\text{ref}}$。

5. **时变安全走廊与单侧拓扑防横跳管道 (Connected Tube Safe Corridor)**：
   - **机身锚定与意图继承**：走廊从机身当前匹配位姿与上一轮规划点展开，若处于绕障状态自动锁定拓扑分支（`LEFT` 或 `RIGHT`）。
   - **单侧严格延伸**：截面 $i > 0$ 沿连通通道单侧生长，废除贪婪宽度比较，严禁因对侧更宽而跨障碍物左右横跳。
   - **上一轮规划引导 (Temporal Guidance)**：以时移上一轮规划点 $d_{i+1}^{*(t-1)}$ 优先作为种子展开，走廊与规划轨迹平滑贴合。
   - **全工况自相交消除**：通过几何解析法彻底消除 180° 掉头弯与急弯处相邻截面肋线及非相邻边界的自相交。

6. **LTV-MPC 凸优化与极速求解 (LTV-MPC with Shifted Warm Start)**：
   - 采用线性时变 Frenet 运动学离散模型，保证动力学约束严格线性，Hessian 矩阵常数正定：
     $$s_{k+1} = s_k + v_k \cdot dt_k, \quad d_{k+1} = d_k + v_{\text{eff}, k} \cdot e_{\psi, k} \cdot dt_k, \quad e_{\psi, k+1} = e_{\psi, k} + (\omega_k - \kappa_k v_k) \cdot dt_k, \quad v_{k+1} = v_k + a_k \cdot dt_k$$
   - 引入**帧间轨迹一致性惩罚项 (Inter-sample Trajectory Consistency)**：$q_{\text{prev\_d}} \cdot (X(1, k+1) - d_{\text{prev}}(k))^2$，抑制帧间数值微抖。
   - 配合 **OSQP 凸二次规划器** 与 **时移热启动 (Shifted Warm Start)**，单帧求解时间仅需 **$1 \sim 3\text{ms}$**！

7. **真实速度硬约束与输出平滑 (Strict Velocity Hard Constraint & Slew-Rate Limiting)**：
   - **真实速度硬约束**：若 `v_min >= 0.0`，底层前置关停倒车模式，输出速度硬截断在 $[0.0, v_{\max}]$，严格禁绝任何后退；若 `v_min < 0.0`，以 $v_{\min}$ 作为真实的倒车速度上限。
   - **角速度斜率限制 (Slew-Rate Limiter)**：限制角速度单步变化率（最大 `0.35 rad` / `0.1s`），彻底消除出弯及高频扰动引发的方向盘抖动。

---

## 依赖项 (Dependencies)

- **ROS 2** (Nav2, tf2, geometry_msgs, nav_msgs, visualization_msgs, std_msgs)
- **CasADi** (带有 OSQP / Ipopt 接口)
- **OSQP**
- **Eigen3**

---

## 参数配置 (nav2_params.yaml)

请将以下参数复制到你的 `nav2_params.yaml` 文件中 `controller_server` 节点下的具体控制器配置块内：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mpc_controller::MPCController"
      
      # 1. 预测时域与离散步长（40步非均匀采样提供长前瞻视野）
      N: 40                            # 预测步数 (Prediction Horizon Steps)；兼顾局部绕障视野与 40Hz 实时性
      dt: 0.025                        # 首段预测基准步长 (40Hz 控制周期下为 0.025s)
      use_variable_dt: true            # 启用非均匀时间步长 (近密远疏，覆盖长前瞻视野)
      solver_type: "osqp"              # 求解器类型: "osqp" (极速 LTV-MPC ~1-3ms) 或 "ipopt" (~8ms)

      # 2. 机器人运动学与动力学物理极限
      v_max: 0.50                      # 最大前进线速度 (m/s)
      v_min: -0.20                     # 最小线速度 (m/s, 0.0 表示严格禁止后退; 负值如 -0.20 允许倒车且限制倒车速度)
      w_max: 2.20                      # 最大角速度 (rad/s)
      w_min: -2.20                     # 最小角速度 (rad/s)
      a_max: 0.80                      # 最大线加速度 (m/s²)
      a_min: -0.80                     # 最大线减速度 (m/s²)

      # 3. Frenet 状态误差跟踪与帧间平滑代价权重
      q_s: 2.0                         # 纵向弧长 s 跟踪权重
      q_d: 20.0                        # 横向偏差 d 跟踪权重 (开阔区严格贴合全局参考线)
      q_e_psi: 8.0                     # 航向角误差 e_psi 阻尼权重
      q_prev_d: 12.0                   # 与上一帧横向轨迹的一致性权重 (约束相邻周期规划点差异，赋予轨迹动量，抑制微抖)

      # 4. 控制量与平滑度代价权重
      r_v: 1.0                         # 线速度 v 跟踪偏差权重
      r_w: 1.0                         # 角速度 w 控制权重
      r_a: 0.20                        # 线加速度 a 控制量惩罚权重

      # 5. 基于参考路线的安全走廊与防自交叉配置
      enable_safe_corridor: true       # 安全走廊总开关 (true: 走廊绕障; false: 纯跟线)
      corridor_default_left_width: 0.80    # 走廊默认最大左侧宽度 [m]
      corridor_default_right_width: 0.80   # 走廊默认最大右侧宽度 [m]
      corridor_min_width: 0.15             # 走廊最小有效通行宽度 [m] (低于此宽度视为不可通行)
      corridor_curvature_safety_factor: 0.85  # 曲率中心奇异点防护 (alpha / kappa)
      corridor_rib_safety_margin: 0.05     # 相邻截面肋线收缩保护裕量 [m]
      corridor_max_lateral_rate: 0.40      # 走廊横向最大坡度 (双向平滑滤波，限制走廊收缩/扩张剧烈程度)
      corridor_avoid_hairpin: true         # 180°掉头弯自交叉全局消除开关
      corridor_check_costmap: true         # 是否从 Costmap 提取障碍物收缩走廊
      corridor_costmap_cost_threshold: 253 # Costmap 障碍物代价阈值 (>=253 视为实际障碍)

      # 6. 平底锅式边缘死区二次代价与窄通道自适应居中势场
      corridor_buffer_margin: 0.15     # 走廊边缘死区缓冲裕量 [m] (开阔区代价为0，免疫点云噪点)
      q_corridor_center: 30.0          # 窄通道宽度自适应居中势场权重 (通道越窄居中推力平方级放大)
      q_corridor_bound: 35.0           # 走廊边缘二次平滑排斥势场权重
      w_corridor_slack: 1000.0         # 走廊软约束越界松弛惩罚 (保证 100% 具备可行解)

      # 7. 终点位姿物理连续漏斗约束与终端代价 (终点平滑精准停靠)
      enable_terminal_constraint: true # 终点位姿硬约束与漏斗吸附总开关
      goal_approach_dist: 1.00         # 终点进近模式距离阈值 [m] (未进入时不收拢，两边保持平行；进入后漏斗收拢)
      terminal_s_tol: 0.01             # 终点纵向位置容差 [m]
      terminal_d_tol: 0.03             # 终点横向偏差容差 [m] (强制厘米级对齐)
      terminal_epsi_tol: 0.0174        # 终点航向角误差容差 [rad] (约 1.0°)
      terminal_v_tol: 0.01             # 终点末端速度容差 [m/s]
      q_s_terminal: 10.0               # 终端纵向代价权重
      q_d_terminal: 50.0               # 终端横向代价权重 (强拉引横向归零)
      q_epsi_terminal: 20.0            # 终端航向代价权重 (强拉引对齐目标朝向)
      r_v_terminal: 10.0               # 终端速度代价权重 (强拉引平稳减速停稳)
```

---

### 参数说明表

#### 1. MPC 预测时域与离散步长 (Horizon & Discretization)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `N` | `int` | `10` | `40` | - | MPC 预测步数 (Prediction Horizon Steps) |
| `dt` | `double` | `0.1` | `0.025` | `s` | MPC 预测基准步长 (40Hz 设为 0.025s，20Hz 设为 0.05s) |
| `use_variable_dt` | `bool` | `true` | `true` | - | 是否启用非均匀时间步长 (近端 $1\times dt$、中端 $3\times dt$、远端 $6\times dt$，覆盖长前瞻视野) |
| `solver_type` | `string` | `"osqp"` | `"osqp"` | - | 优化求解器类型 (`"osqp"`: LTV-MPC+OSQP 极速求解 ~1-3ms; `"ipopt"`: 内置 Ipopt 求解 ~8ms) |

#### 2. 机器人运动学与动力学物理极限 (Kinematic & Dynamic Bounds)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `v_max` | `double` | `0.5` | `0.50` | `m/s` | 允许的最大前进线速度 |
| `v_min` | `double` | `0.0` | `-0.20` | `m/s` | 允许的最小线速度（`0.0` 严格禁绝后退；若为负值如 `-0.20` 则表示允许倒车且限制倒车最高速度为 $|v_{\min}|$） |
| `w_max` | `double` | `1.0` | `2.20` | `rad/s` | 允许的最大正向角速度 |
| `w_min` | `double` | `-1.0` | `-2.20` | `rad/s` | 允许的最大反向角速度 |
| `a_max` | `double` | `1.0` | `0.80` | `m/s²` | 允许的最大线加速度 |
| `a_min` | `double` | `-1.0` | `-0.80` | `m/s²` | 允许的最大线减速度 |

#### 3. Frenet 状态误差跟踪与帧间一致性代价 (Frenet Tracking & Consistency)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `q_s` | `double` | `2.0` | `2.0` | - | Frenet 纵向位置跟踪权重 $(s - s_{\text{ref}})^2$ |
| `q_d` | `double` | `20.0` | `20.0` | - | Frenet 横向偏移误差权重 $(d - 0)^2$ (开阔区贴合全局参考线) |
| `q_e_psi` | `double` | `5.0` | `8.0` | - | Frenet 航向姿态误差权重 $(e_\psi - 0)^2$ (抑制车头摆动) |
| `q_prev_d` | `double` | `8.0` | `12.0` | - | 帧间轨迹一致性惩罚权重 $(d_k^{(t)} - d_{k+1}^{*(t-1)})^2$（抑制规划轨迹颤抖与突变） |

#### 4. 控制量与平滑度代价权重 (Control Effort & Smoothness Costs)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `r_v` | `double` | `1.0` | `1.0` | - | 速度跟踪偏差权重 $(v - v_{\text{ref}})^2$ |
| `r_w` | `double` | `0.5` | `1.0` | - | 角速度输入代价权重 $(w - w_{\text{ref}})^2$ |
| `r_a` | `double` | `0.2` | `0.20` | - | 加速度输入代价权重 $a^2$ |

#### 5. 基于参考路线的安全走廊配置 (Safe Corridor Settings)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `enable_safe_corridor` | `bool` | `true` | `true` | - | 安全走廊总开关 (`true`: 走廊自适应绕障; `false`: 纯跟线) |
| `corridor_default_left_width` | `double` | `0.8` | `0.80` | `m` | 走廊默认最大左侧半宽 |
| `corridor_default_right_width` | `double` | `0.8` | `0.80` | `m` | 走廊默认最大右侧半宽 |
| `corridor_min_width` | `double` | `0.15` | `0.15` | `m` | 走廊最小通行宽度 (低于此宽度视为不可通行) |
| `corridor_curvature_safety_factor` | `double` | `0.85` | `0.85` | - | 曲率中心奇异点防护因子 ($\alpha / \|\kappa\|$, 防止内侧跨越曲率中心) |
| `corridor_rib_safety_margin` | `double` | `0.05` | `0.05` | `m` | 相邻截面肋线收缩保护裕量 |
| `corridor_max_lateral_rate` | `double` | `0.4` | `0.40` | - | 走廊横向最大坡度 $\tan\phi$ (双向滤波，抑制走廊剧烈收缩/扩张) |
| `corridor_avoid_hairpin` | `bool` | `true` | `true` | - | 180°掉头弯/急转弯自相交全局消除开关 |
| `corridor_check_costmap` | `bool` | `true` | `true` | - | 是否从 Costmap 代价地图提取障碍物实时收缩走廊 |
| `corridor_costmap_cost_threshold` | `int` | `100` | `253` | - | Costmap 障碍代价阈值 ($\ge$ 此值视为障碍，253 为真实障碍物边界) |

#### 6. 走廊边缘排斥势场与窄通道自适应居中 (Barrier & Centering Costs)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `corridor_buffer_margin` | `double` | `0.1` | `0.15` | `m` | 走廊边缘死区缓冲裕量 (死区内排斥力为0，免疫点云微小噪点) |
| `q_corridor_center` | `double` | `30.0` | `30.0` | - | 窄通道宽度自适应居中势场权重 (通道越窄居中推力平方级放大) |
| `q_corridor_bound` | `double` | `35.0` | `35.0` | - | 走廊边缘二次平滑排斥势场权重 (靠近边缘提供柔和回弹斥力) |
| `w_corridor_slack` | `double` | `1000.0` | `1000.0` | - | 走廊软约束越界松弛惩罚权重 (保证优化问题 100% 可行解) |

#### 7. 终点位姿连续漏斗约束与终端代价 (Terminal Goal Constraints & Costs)

| 参数名 | 类型 | 默认值 | 推荐值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `enable_terminal_constraint` | `bool` | `true` | `true` | - | 终点位姿硬约束与连续漏斗总开关 |
| `goal_approach_dist` | `double` | `0.6` | `1.00` | `m` | 终点进近模式距离阈值 (未到达时走廊保持平行，进入后漏斗收拢) |
| `terminal_s_tol` | `double` | `0.05` | `0.01` | `m` | 终点纵向位置硬约束容差 |
| `terminal_d_tol` | `double` | `0.03` | `0.03` | `m` | 终点横向偏差硬约束容差 (强制厘米级对齐终点) |
| `terminal_epsi_tol` | `double` | `0.05` | `0.0174` | `rad` | 终点航向角误差硬约束容差 (约 1.0°) |
| `terminal_v_tol` | `double` | `0.01` | `0.01` | `m/s` | 终点末端速度硬约束容差 |
| `q_s_terminal` | `double` | `10.0` | `10.0` | - | 终端纵向位置代价权重 |
| `q_d_terminal` | `double` | `50.0` | `50.0` | - | 终端横向偏差代价权重 (强拉引横向偏差归零) |
| `q_epsi_terminal` | `double` | `20.0` | `20.0` | - | 终端航向角误差代价权重 (强拉引对齐目标朝向) |
| `r_v_terminal` | `double` | `10.0` | `10.0` | - | 终端速度代价权重 (强拉引平稳减速停稳) |

---

## 话题接口 (Topics)

### 发布话题 (Publishers)
- `predict_trajectory` (`nav_msgs/msg/Path`): MPC 预测轨迹（由 Frenet 状态解转换回笛卡尔坐标发布）
- `transformed_global_plan` (`nav_msgs/msg/Path`): 转换到控制器坐标系下的全局路径
- `transformed_local_plan` (`nav_msgs/msg/Path`): 用于局部追踪截取的局部路径
- `local_plan` (`nav_msgs/msg/Path`): 时间参数化与等时采样后的局部参考路径
- `local_plan_markers` (`visualization_msgs/msg/MarkerArray`): 局部路径点速度向量彩色箭头（绿->红代表速度大小）
- `safe_corridor_markers` (`visualization_msgs/msg/MarkerArray`): 安全走廊 3D 网格、左右边界线及各截面肋线可视化 MarkerArray
- `lateral_error` (`std_msgs/msg/Float64`): 机器人实时的横向偏移误差 $d$（单位：米）
