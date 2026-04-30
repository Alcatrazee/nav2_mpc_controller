# Nav2 MPC Controller

这是一个基于 CasADi 和 IPOPT 求解的简易模型预测控制 (MPC) 局部规划器插件，适用于差速驱动机器人。

## 参数配置 (nav2_params.yaml)

请将以下参数复制到你的 `nav2_params.yaml` 文件中 `controller_server` 节点下的具体控制器配置块内。

注意：下方的 `FollowPath` 是在 `controller_plugins` 列表中定义的控制器名称，如果你使用的名称不同，请做相应的替换。

```yaml
controller_server:
  ros__parameters:
    # ... [其他配置如 controller_plugins 等] ...
    FollowPath:
      plugin: "nav2_mpc_controller::MPCController"
      N: 15            # 预测步数 (整数)
      dt: 0.1          # 预测步长，单位：秒
      v_max: 0.5       # 最大线速度，单位：m/s
      v_min: 0.0       # 最小线速度，单位：m/s
      w_max: 1.0       # 最大角速度，单位：rad/s
      w_min: -1.0      # 最小角速度，单位：rad/s
      
      # 代价函数权重参数
      q_x: 1.0         # X方向追踪误差权重
      q_y: 1.0         # Y方向追踪误差权重
      q_theta: 0.1     # 航向角追踪误差权重
      r_v: 0.1         # 线速度控制平滑/惩罚权重
      r_w: 0.1         # 角速度控制平滑/惩罚权重
```