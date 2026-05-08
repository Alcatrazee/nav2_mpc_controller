# Nav2 MPC Controller

这是一个基于 CasADi 和 IPOPT 求解的简易模型预测控制 (MPC) 局部规划器插件，适用于差速驱动机器人。

## 核心特性与路径处理流水线

本控制器在接收到全局路径后，执行了以下高度优化的处理流水线：

1. **坐标系转换与局部路径截取 (Local Plan Extraction)**：
   将全局路径从全局坐标系转换至控制器的局部坐标系。随后计算距离机器人当前位姿最近的路径点，截取从该点开始的前方路径，并自动裁剪掉超出当前局部代价地图 (Costmap) 范围的不可见区域。

2. **横向采样避障与轨迹评价 (Lattice Planner)**：
   基于截取到的基准路径，在设定的前瞻距离（`lattice_lookahead_dist`）处寻找目标点，并进行左右横向采样。使用闭式五次多项式（Quintic Spline）生成从机器人当前位姿到各候选点的簇状轨迹（Candidate Paths）。利用机器人的实际轮廓（Footprint）结合代价地图进行精确碰撞检测，综合评估**障碍物代价**、**曲率平滑度代价**和**横向偏离代价**，进而选出一条安全且最优的局部轨迹。

3. **多项式平滑与极高密度重采样 (Polynomial Smoothing & Resampling)**：
   针对提取出的局部路径（可能由于分辨率或网格原因存在折线），利用基于 SVD 的最小二乘法进行最高 5 次多项式拟合。随后以 `0.001m` 的极高空间分辨率对曲线进行重采样，计算出极其平滑的一阶导数和二阶导数，从而获取无噪音的路径曲率 ($\kappa$)。

4. **时间参数化与速度规划 (Time-Parameterized Velocity Profiler)**：
   根据给定的动力学约束（最大线速度、最大横向加速度、最大纵向加/减速度），结合曲率限制自动计算出每个点的安全上限速度。随后经过“前向运动学加速”和“后向运动学减速”两次扫描，利用梯形积分生成具有相对时间戳 ($t$) 和目标速度 ($v$) 的平滑轨迹。

5. **MPC 等时参考点采样 (Time-based Reference Extraction)**：
   根据 MPC 设定的预测步长 $dt$ 和预测步数 $N$，利用 $O(\log N)$ 时间复杂度的二分查找算法 (`std::lower_bound`)，从时间参数化轨迹中精确插值出未来 $t = k \times dt$ 时的目标参考状态集合 ($x, y, \theta, v$)。

6. **CasADi / IPOPT 非线性优化求解 (Nonlinear MPC Optimization)**：
   利用差速运动学模型（欧拉前向离散化）构建预测模型，设立初始状态约束和控制边界约束。目标函数不仅惩罚位置和航向追踪误差，还对控制指令的平滑度进行约束。最终使用 IPOPT 求解最优控制序列，并下发第一步的线速度和角速度。

## 依赖项 (Dependencies)

- **ROS 2** (Nav2, tf2, geometry_msgs, nav_msgs, visualization_msgs)
- **CasADi** (含 IPOPT 求解器)
- **Eigen3** (用于 SVD 最小二乘法曲线拟合)

## 参数配置 (nav2_params.yaml)

请将以下参数复制到你的 `nav2_params.yaml` 文件中 `controller_server` 节点下的具体控制器配置块内。

注意：下方的 `FollowPath` 是在 `controller_plugins` 列表中定义的控制器名称，如果你使用的名称不同，请做相应的替换。

```yaml
controller_server:
  ros__parameters:
    # ... [其他配置如 controller_plugins 等] ...
    FollowPath:
      plugin: "nav2_mpc_controller::MPCController"
      
      # --- MPC 预测与边界参数 ---
      N: 15            # MPC预测时域步数 (默认: 15)
      dt: 0.1          # MPC预测离散步长，单位：s (默认: 0.1)
      v_max: 0.5       # 机器人最大前向线速度，单位：m/s (也是Profiler的参考上限)
      v_min: 0.0       # 机器人最小线速度 (默认限制不倒车)
      w_max: 1.0       # 机器人最大角速度，单位：rad/s
      w_min: -1.0      # 机器人最小角速度，单位：rad/s
      
      # --- 代价函数权重参数 ---
      q_x: 1.0         # X坐标追踪误差惩罚权重
      q_y: 1.0         # Y坐标追踪误差惩罚权重
      q_theta: 0.1     # 航向角(Yaw)追踪误差惩罚权重
      r_v: 0.1         # 线速度控制量大小惩罚权重（防止加减速过猛）
      r_w: 0.1         # 角速度控制量大小惩罚权重（防止自旋过猛）
      
      # --- Lattice Planner 横向避障采样参数 ---
      lattice_lookahead_dist: 1.5  # 前瞻目标点距离，单位：m (默认: 1.5)
      lattice_lat_range: 0.8       # 横向采样范围，单位：m (默认: +/- 0.8)
      lattice_lat_step: 0.2        # 横向采样步长，单位：m (默认: 0.2)
      lattice_lon_ratios: [0.6, 0.8, 1.0] # 多个前向采样点的距离比例 (默认: [0.6, 0.8, 1.0])
      weight_obs: 2.0              # 障碍物代价值权重 (默认: 2.0)
      weight_lat: 1.0              # 轨迹横向偏离全局路径代价权重 (默认: 1.0)
      weight_smooth: 0.5           # 轨迹曲率平滑舒适性代价权重 (默认: 0.5)
```

## 已实现功能
1. 跟踪任意的路径（但无障碍物识别和停障或绕障功能）
2. 发布可视化的路径信息（即根据速度变化的箭头）
## 待实现功能
1. 局部绕障