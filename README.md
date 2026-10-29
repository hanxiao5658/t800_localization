# t800_localization — ROS1 Noetic

当前是 **可构建的工程骨架**；尚未实现 FastGICP 配准、地图加载或 map→odom 发布，不能用它代替已验证的 FAST-LIO2 或安全停车节点。

## 目录
- `maps/`：放置首次建图保存的 PCD（大型文件可只放挂载目录并在 YAML 配绝对路径）。
- `config/localization_params.yaml`：后续地图定位接口的预留配置。
- `config/startup_reference.yaml`：可由下述采集工具生成，记录固定开机位置在**首次建图坐标系**中的 FAST-LIO 参考位姿。
- `scripts/capture_reference_pose.py`：参考位姿采集工具，不执行 GICP。
- `launch/capture_reference_pose.launch`：启动该采集工具。

## 创建与编译
将 `t800_localization/` 放到 `/root/catkin_ws/src`，执行：
```bash
cd /root/catkin_ws
chmod +x src/t800_localization/scripts/*.py
catkin_make --pkg t800_localization
source devel/setup.bash
rospack find t800_localization
```

## 首次建图期间采集未来开机点的初值
1. 使用 MID360 + FAST-LIO2 **首次建图**；确保 PCD 将按这次建图的坐标系保存。
2. 走到固定开机地点（如走廊拐角），机器人停止、IMU/里程计稳定。
3. 在同一次 FAST-LIO 运行中执行：
```bash
roslaunch t800_localization capture_reference_pose.launch
```
4. 查看 `config/startup_reference.yaml`；这记录的是 FAST-LIO `/Odometry` 的 **child_frame 在首次建图坐标系中的位姿**，不是 `base_footprint`。如果需要固定脚部导航参考点，之后还需要做几何外参转换。
5. 继续建图完成后保存 PCD。若首次建图末尾漂移很大，之前记录的位姿也可能对地图产生偏差；需要重新校准。

**不是每次开机都需要用这工具记录。** 日常启动时需要采集当前点云用于 GICP，与首次建图保存的 PCD 进行匹配。首次建图期间记录的初值仅作为定位搜索起点；只有在大致相同的固定起点开机时适用。以后开机位置不固定，需要引入地点识别/多初值/粗配准等机制。

## 数据和坐标系注意
- `/Odometry` 的 `header.frame_id` 可能为 `camera_init`，`child_frame_id` 可能为 `body`，以实际消息为准。
- 若后续使用 `/cloud_registered`，该点云可能已在 FAST-LIO 的世界/里程计系；若使用原始 `/livox/lidar`，则是雷达局部坐标。配准的初值必须对应输入点云坐标系，**不能混用两者**。
- `map` 要明确与首次保存 PCD 的建图坐标系一致；新开机时 FAST-LIO 运行坐标系一般不同，需要算 `map→odom`。
- 地图里的 Waypoint 统一采用 `map` 的 `base_footprint` 位姿（地面参考点），不要将 FAST-LIO IMU 位姿直接当作脚尖中点位姿。
- 首次建图中的固定开机位姿可以先在 RViz/PCD 中粗略测量，再通过 GICP 校正；采集工具只是降低后续粗对准的操作量。

## 下一阶段建议
1. 先在录好的第二段 ROSBag 上验证局部扫描与 PCD 的 GICP 配准成功率、初值容忍范围及误匹配。
2. 经验证再增加 `map_matcher`（C++/PCL+fast_gicp），输出质量/收敛指标以及定位状态。
3. 使用 `map→odom` 和 FAST-LIO 里程计计算 `map→base_footprint`，发布 `/my_ros1/localization/pose`，兼容导航旧接口。
4. 定位质量不足时，导航应停车，不得把 GICP 错配强制用作位姿修正。
