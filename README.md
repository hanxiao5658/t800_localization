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



## ------------------------------------------

# T800 标准站姿 IMU 外参标定 + 水平地图轨迹可视化

本增量包针对当前公开仓库 `hanxiao5658/t800_localization` 的 ROS1 Noetic 结构设计。**不会覆盖原有 C++ FastGICP、map_leveling.yaml、PCD 地图，也不会向机器人发送速度指令。**

## 1. 将文件复制到原 ROS 包

将补丁文件夹下：
- `scripts/calibrate_body_extrinsics.py`
- `scripts/base_footprint_path.py`
- `config/body_extrinsics.yaml`
- `launch/base_footprint_visualization.launch`
- `rviz_config/t800_footprint.rviz`（可选预设配置）

分别放到 `/root/catkin_ws/src/t800_localization/` 的同名目录中。注意 **不要覆盖已有的 `scripts/level_math.py`，本补丁会复用它。**

```bash
cd /root/catkin_ws
chmod +x src/t800_localization/scripts/calibrate_body_extrinsics.py
chmod +x src/t800_localization/scripts/base_footprint_path.py
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

本次只是新增 Python 脚本/Launch，无需重新编译 FastGICP。若想使 `catkin_make install` 也安装两个脚本，可以将它们追加到 `CMakeLists.txt` 中原有的 `catkin_install_python(PROGRAMS ...)` 列表，不能创建第二个覆盖原列表的同名配置块。

## 2. 标定标准站姿下 base_footprint → body 旋转

让机器人在平整地面 **标准站立、停止步行与转向**，MID360s 与运行 FAST-LIO2 时使用的安装姿态一致，保持静止不少于 5 秒。

```bash
rosrun t800_localization calibrate_body_extrinsics.py \
  --duration 4 --skip 0.5 --height 1.5 \
  --output /root/catkin_ws/src/t800_localization/config/body_extrinsics.yaml
```

也可用标准站姿 ROSBag（不需要播放，也不需要 FAST-LIO2）：

```bash
python3 /root/catkin_ws/src/t800_localization/scripts/calibrate_body_extrinsics.py \
  --bag /path/to/standing.bag --skip 0.5 --duration 4 --height 1.5 \
  --output /root/catkin_ws/src/t800_localization/config/body_extrinsics.yaml
```

输出：

- `translation_xyz: [0,0,1.5]`：名义高度，**请后续按机械测量修改；测量确认后改 `translation_measured: true`**。
- `rotation_xyzw`：固定姿态标定，定义 `base_footprint` 到 `body` 的四元数。
- `gravity_up_in_body`：IMU 静止比力的归一化向量。
- `rpy_deg_diagnostic`：输出供人阅读的角度，不能代替 quaternion。
- `rotation_calibrated: true`：标定成功后写入。

**假设与边界：** 静止 IMU 只能测量重力倾斜（Roll/Pitch），不能单独确定 Yaw。脚本将 body +X 的水平投影约定为 base_footprint +X（机器人正前方）。若机械方向存在水平偏航偏差，可先独立测量，再通过 `--yaw-offset-deg 角度` 明确配置。测的是 IMU body 原点，而不是雷达点云原点。确认 `/livox/imu` 测量轴与 FAST-LIO `body` 实际一致（尤其 FAST-LIO 外参配置）。这份固定外参适用于名义标准站姿，不能精确代表移动中的两个真实接触脚的位置。

## 3. 启动水平地图、轨迹和坐标轴

**先做以下检查：**

```bash
rosparam get /use_sim_time
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/child_frame_id
```

如实机实时数据，`/use_sim_time` 应为 `false` 且时间戳与 ROS 时间接近；如果离线 ROSBag，请在启动节点前 `rosparam set /use_sim_time true`，并 `rosbag play --clock ...`；Gazebo 不能同时发布 `/clock`。建议将 `fastgicp_params.yaml` 的 `max_data_age` 从此前用于调试的超大数值恢复为合理值（如1.0秒），并确保 `map_frame: map_raw`，`publish_tf: false`，`auto_relocalize: false`。

依次启动：

1. **已有 FAST-LIO2**（MID360s 或 ROSBag）。应有 `/Odometry` 和 `/cloud_registered`。
2. **已有 FastGICP**：

```bash
roslaunch t800_localization fastgicp_relocalization.launch \
  pcd_file:=/root/catkin_ws/src/t800_localization/maps/实际PCD文件名.pcd
```

3. **新增水平地图与轨迹显示节点**（其中 include 了旧 `level_map_rviz.launch`，所以**不要再单独启动旧 level_map_rviz.launch**，否则 `map→map_raw` TF 重复）：

```bash
roslaunch t800_localization base_footprint_visualization.launch \
  pcd_file:=/root/catkin_ws/src/t800_localization/maps/实际PCD文件名.pcd
```

4. **执行一次 GICP 服务**，在数据持续刷新时操作：

```bash
rosservice call /my_ros1/localization/relocalize "{}"
```

此后不必重复调用 GICP。新轨迹节点仅使用第一次收到的有效 `map_raw→camera_init` 配准结果，继续消费每帧 `/Odometry`；如果需要重新计算配准，请重启轨迹节点及重新标定运行状态。

## 4. RViz 配置

可以直接用预设配置打开 RViz：

```bash
rviz -d /root/catkin_ws/src/t800_localization/rviz_config/t800_footprint.rviz
```

或者手动打开 `rviz`，Global Options → **Fixed Frame = `map`**。Add 以下显示：

| Display | Topic / Reference Frame | 作用 |
|---|---|---|
| PointCloud2 | `/my_ros1/localization/prior_map_raw` | 首次建图 PCD；通过静态 TF 在 `map` 中水平显示 |
| PointCloud2 | `/my_ros1/localization/aligned_scan` | 开机一次性配准后的局部扫描 |
| Path | `/my_ros1/localization/path` | 位姿轨迹；`header.frame_id=map` |
| Axes | **Reference Frame = `base_footprint`** | 机器人名义脚部坐标轴，**X 红、Y 绿、Z 蓝** |
| Pose | `/my_ros1/localization/base_footprint_pose` | 可选，检查位置和水平朝向 |
| Grid | （默认） | 可检查地面与水平面倾角；Z 零点需额外标定 |

如果看到 `Axes: No transform from [base_footprint] to [map]`，先检查 GICP 是否已成功；然后：

```bash
rostopic echo -n 1 /my_ros1/localization/pose_status
rostopic echo -n 1 /my_ros1/localization/base_footprint_pose
rostopic echo -n 1 /my_ros1/localization/path/header
rosrun tf tf_echo map base_footprint
```

推荐状态 `TRACKING_FIXED_GICP`。刚启动前会是 `WAITING_FOR_INITIAL_GICP`。

轨迹与 base_footprint 均位于重力对齐后的 `map`，`base_footprint` 的姿态是平面纯 Yaw，故在 RViz 中**蓝色 Z 轴应始终竖直**。其原点的 Z 坐标是基于刚性名义几何推算值，并不保证等于0；需要标定地面高度才能让 Z=0 代表地面。

## 5. 重要工程注意

- `/my_ros1/localization/map_to_odom` 实际是 `map_raw→camera_init`，虽然话题名仍叫 map_to_odom。已有 FastGICP 定期发同一个 TransformStamped，而且 publisher 是 latched。新节点只锁定第一次结果。
- `map_raw→camera_init` 是**一次开机的固定变换**，`camera_init→body` 则是 FAST-LIO2 的连续变化量。新节点本身不执行配准。
- 原始 `map_leveling.yaml` 应保持来自**首次建图 Bag**，千万不要拿第二次开机的重力数据覆盖它。机器人安装角改变时可以重新标定 `body_extrinsics.yaml`，但原先水平化 PCD 的固定地图变换不必随之改变。
- 不要再同时启动旧 `relocalization_rviz.launch` 的 PCD 发布器，防止不同节点给同一地图话题发消息。
- 本版本发布 `map→base_footprint` 单独 TF，避免和其它可能存在的 `map_raw→camera_init→body` TF 发布器冲突。它不代表所有中间边均被发布到 TF 树；中间变换由节点内部计算。
- **请检查是否有其它节点也发布 `base_footprint` TF。** 有时机器人 URDF `robot_state_publisher` 会使用同名帧；有重复则设置 `<arg name="publish_foot_tf" value="false"/>` 或改新节点 `foot_frame`。不允许同一 TF 子坐标系有两个父坐标系。
- **仅用于定位链路验证**：它不会向导航下发运动命令，亦无完整失锁停车策略。FAST-LIO2 自身漂移仍可能累积；一次 GICP 不等于实时地图约束。
- 在 ROSBag 倒带或重播时，若新的 FAST-LIO2 重置了局部原点，应该重新启动两种定位节点并重新执行 GICP；不能把上一次运行的 `map_raw→camera_init` 继续用于新的会话。
