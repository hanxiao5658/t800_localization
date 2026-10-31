# T800 Localization：双 Launch 部署

将 `localization.launch` 与 `localization_rviz.launch` 放到仓库 `launch/` 文件夹。

## 地图约定
唯一地图：`$(find t800_localization)/maps/hall_raw.pcd`。不使用其他 PCD。必须保证初始位姿 YAML 与这份 PCD 采用相同的原始 `map_raw` 坐标系。若地图发生变化，需要重新核准 `config/fastgicp_params.yaml` 中初始位姿及 `config/map_leveling.yaml`。

## 启动顺序

1. 在其他终端启动 MID360s 驱动及 FAST-LIO2，确保 `/Odometry`、`/cloud_registered` 正常发布。
2. 启动计算：`roslaunch t800_localization localization.launch`
3. 可选，另一个终端只启动 RViz：`roslaunch t800_localization localization_rviz.launch`
4. 点云累计至少 10 帧后手动触发：`rosservice call /my_ros1/localization/relocalize "{}"`
5. 检查：`rostopic echo /my_ros1/localization/status`，成功应显示 `ALIGNED_INITIAL_ONLY`。

## 数据输入
- `/Odometry`：FAST-LIO2 里程计
- `/cloud_registered`：FAST-LIO2 已配准到 camera_init 的点云（FastGICP 源点云）

## 定位输出
- `/my_ros1/localization/status`：GICP 状态 `std_msgs/String`，初始 `UNINITIALIZED`，匹配中 `MATCHING`，首次成功 `ALIGNED_INITIAL_ONLY`，失败时显示相应 REJECTED* 状态。
- `/my_ros1/localization/map_to_odom`：实际为 `map_raw -> camera_init`，不是 ROS 常见 `map -> odom`，沿用旧命名。
- `/my_ros1/localization/base_footprint_pose`：`map` 下名义脚部平面位姿。
- `/my_ros1/localization/body_pose`：`map` 下 body 三维位姿。
- `/my_ros1/localization/path`：地图坐标系下轨迹。
- `/my_ros1/localization/pose_status`：位姿链路状态，成功后通常 `TRACKING_FIXED_GICP`。

## TF 注意事项
`map -> map_raw` 由 `level_map_rviz.py` 发布；首次成功后 `map_raw -> camera_init` 由 `initial_gicp_tf_static.py` 发布静态 TF；`camera_init -> body` 需 FAST-LIO2 自身提供；`map -> base_footprint` 由 `base_footprint_path.py` 发布。保持 `config/fastgicp_params.yaml` 的 `publish_tf: false`，避免和静态 TF 桥重复广播。

旧 `base_footprint_visualization.launch` 名字里虽然有 visualization，但实际上只启动 ROS 数据计算和发布节点，不会打开 RViz；新总启动文件复用它。若 RViz 配置文件名不为 `t800_footprint.rviz`，启动时可用 `rviz_config:=绝对路径` 覆盖。

## 使用边界
这是一次性 GICP 对齐 + 持续 FAST-LIO2 里程计，不是连续的 PCD 匹配定位。重启 FAST-LIO2 后必须重启本定位链路，重新触发 GICP。`ALIGNED_INITIAL_ONLY` 不是后续定位置信度证明。
