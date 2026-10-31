# T800 Localization — MID360s + FAST-LIO2 + FastGICP（ROS1 Noetic）

> 基于 `hanxiao5658/t800_localization` 的 `main` 分支公开源码核对整理（2026-10-31）。本文记录**当前代码实际行为**，并明确标注部署前必须检查的配置。不包含机器人运动控制或失锁安全停车功能。

## 1. 系统用途和边界

此包负责把新一次开机运行的 FAST-LIO2 局部坐标系 `camera_init`，通过开机时一次 FastGICP 配准，对齐至首次建图得到的 PCD 坐标系 `map_raw`；再把 `map_raw` 按首次建图标定的重力方向转换为水平 `map`，计算 `body` 与名义 `base_footprint` 在 `map` 下的位姿。

**当前方案是一次性重定位 + 持续里程计，不是连续地图匹配。** FAST-LIO2 后续漂移不会自动被 PCD 定期修正；机器人真实行走过程中需要额外的置信度监测、动态 TF/定位策略和失锁停车机制。

## 2. 正常部署：只需要哪些启动项

ROS1 Noetic、catkin 环境；以下命令的 ROS 包路径和 PCD 文件名请按本机修改。各命令在不同终端运行（每个终端先 `source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash`）。

1. **外部启动 MID360s 驱动与 FAST-LIO2**（由使用中的 FAST-LIO2 项目提供，不由本包启动），确保 `/Odometry` 和 `/cloud_registered` 持续更新；FAST-LIO2 输入原始 Livox 数据 `/livox/lidar`、`/livox/imu`（实际名称以其配置为准）。
2. **启动本包 FastGICP 重定位节点**：

   ```bash
   roslaunch t800_localization fastgicp_relocalization.launch \
     pcd_file:=/root/catkin_ws/src/t800_localization/maps/hall.pcd
   ```

3. **启动地图水平化 + 名义脚部位姿 + 轨迹节点**：

   ```bash
   roslaunch t800_localization base_footprint_visualization.launch \
     pcd_file:=/root/catkin_ws/src/t800_localization/maps/hall.pcd
   ```

   该 Launch **已经 include** `level_map_rviz.launch`，因此**不要另外再启动** `level_map_rviz.launch`，避免重复发布 `map -> map_raw`。
4. 待 FAST-LIO2 持续提供点云和里程计后，**手动触发一次** FastGICP：

   ```bash
   rosservice call /my_ros1/localization/relocalize "{}"
   ```

   查看 `success: True` 以及配准的 fitness、overlap、修正量。若失败，检查点云数量、坐标帧和匹配阈值，再重试。
5. （可选）`rviz -d /root/catkin_ws/src/t800_localization/rviz_config/t800_footprint.rviz`；Fixed Frame 设为 `map`。

### 哪些 Python/Launch 不是日常必需

| 文件 | 用途 | 日常启动？ |
|---|---|---|
| `launch/fastgicp_relocalization.launch` | 启动 C++ FastGICP | **需要** |
| `src/fastgicp_relocalizer.cpp` | C++ 重定位程序（Launch 自动调用编译产物） | **不手动运行** |
| `launch/base_footprint_visualization.launch` | 合并水平地图、位姿和轨迹 | **需要** |
| `launch/level_map_rviz.launch` | 被上一 Launch 自动包含，加载 PCD、发布地图水平化 TF | **不单独启动** |
| `scripts/level_map_rviz.py` | 发布 `map -> map_raw`、可视化 Marker、body 水平地图位姿 | **由 Launch 启动** |
| `scripts/base_footprint_path.py` | 利用 GICP 和 Odometry 发布 body、footprint 位姿、路径与 TF | **由 Launch 启动** |
| `scripts/level_math.py` | 两个 Python 节点使用的变换数学工具 | **无需启动** |
| `scripts/calibrate_body_extrinsics.py` | 静止 IMU 标定名义脚部到 body 旋转 | **只在标定时** |
| `scripts/capture_reference_pose.py` + `launch/capture_reference_pose.launch` | 首次建图时采集固定开机点位姿 | **只在建图/重设开机点时** |
| `launch/relocalization_rviz.launch`（如仍存在） | 旧可视化流程 | **不要与当前地图可视化流程重复使用** |

## 3. 数据流：GICP 到底用了什么点云？

```text
MID360s 原始雷达/IMU
    │
    └── FAST-LIO2
          ├── /Odometry                [nav_msgs/Odometry]
          │       header.frame_id = camera_init
          │       child_frame_id  = body
          │
          └── /cloud_registered        [sensor_msgs/PointCloud2]
                  frame_id = camera_init（必须与 Odometry 的 header.frame_id 相同）
                        │
                        ▼
                  FastGICP relocalizer
                  ├── 源点云：缓存最近 10–25 帧 /cloud_registered
                  ├── 目标点云：PCD 在 map_raw 下的局部裁剪区域
                  ├── 初值：initial_imu_pose_in_map 和当前 /Odometry
                  ├── Service：/my_ros1/localization/relocalize
                  └── 结果：map_raw -> camera_init
                        │
                        ▼
                  /my_ros1/localization/map_to_odom
                        │
           map_leveling.yaml + body_extrinsics.yaml
                        │
                        ▼
                  /my_ros1/localization/base_footprint_pose
                  /my_ros1/localization/body_pose
                  /my_ros1/localization/path
```

**重要：** FastGICP 直接订阅的是 `/cloud_registered`（而非 `/livox/lidar`、`/cloud_registered_body` 或 `/cloud_effected`）；`/cloud_registered` 已经由 FAST-LIO2 转换到其运行时的 `camera_init` 系。C++ 在 `cloudCb()` 中按 `header.frame_id` 保存并缓存点云，最大保留 25 帧；发起服务时要求至少 10 帧。实际持续时间取决于 FAST-LIO2 输出频率，**不保证恰好 3 秒**。当前代码是对缓存点云直接拼接、降采样，无需再次对整组点云应用 `/Odometry`，因为它们已经在共同的 `camera_init` 系下。

`initial_imu_pose_in_map` 的参数名称容易误解：当前 `map_frame: map_raw`，所以实际输入的初始 IMU/body 位姿属于 **map_raw**，不是水平 `map`；数组为 `[x,y,z,roll,pitch,yaw]`（单位米、弧度），表示开机固定参考位置在原始 PCD 坐标系下的 body/IMU 初值。GICP 先结合当前里程计计算 `T_map_raw_camera_init` 初始猜测：

```text
T_guess = T_map_raw_body(reference) × inverse(T_camera_init_body(current))
```

并使用该初值裁剪局部 PCD 地图、执行 GICP 优化和质量检查。配准通过后输出 `T_map_raw_camera_init`。

## 4. ROS Topics / Services（当前源码）

### 输入

| Topic | 类型 | 生产者 -> 消费者 | 作用 |
|---|---|---|---|
| `/livox/lidar` | Livox 自定义消息或 PointCloud2（视驱动和 FAST-LIO 配置） | MID360s -> FAST-LIO2 | 原始雷达数据；**不是 FastGICP 直接输入** |
| `/livox/imu` | `sensor_msgs/Imu` | MID360s -> FAST-LIO2 | 激光惯性里程计所需 IMU；本包标定脚本也会离线/临时使用 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 -> FastGICP、level_map_rviz、base_footprint_path | 提供动态 `camera_init -> body` 位姿 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO2 -> FastGICP | 已处于 `camera_init` 的配准源点云 |
| `/my_ros1/localization/map_to_odom` | `geometry_msgs/TransformStamped` | FastGICP -> 两个 Python 位姿节点 | 实际为 `map_raw -> camera_init` 的固定对齐结果，名字 `map_to_odom` 仅为历史命名 |

### 输出

| Topic | 类型 | 发布节点 | 意义 |
|---|---|---|---|
| `/my_ros1/localization/status` | `std_msgs/String` | `fastgicp_relocalizer` | GICP 采集、匹配、拒绝/成功状态；成功为 `ALIGNED_INITIAL_ONLY` |
| `/my_ros1/localization/map_to_odom` | `geometry_msgs/TransformStamped` | `fastgicp_relocalizer` | `map_raw -> camera_init`，成功后按定时器重新发布同一数值，latched |
| `/my_ros1/localization/aligned_scan` | `sensor_msgs/PointCloud2` | `fastgicp_relocalizer` | 本次匹配后已经对齐到 `map_raw` 的聚合点云，不是实时跟踪点云 |
| `/my_ros1/localization/prior_map_raw` | `sensor_msgs/PointCloud2` | `pcl_ros/pcd_to_pointcloud` | 首次保存的 PCD，`frame_id=map_raw`；供 RViz 显示 |
| `/my_ros1/localization/level_markers` | `visualization_msgs/MarkerArray` | `level_map_rviz.py` | RViz 参考位置和实时方向标记 |
| `/my_ros1/localization/body_pose_level` | `geometry_msgs/PoseStamped` | `level_map_rviz.py` | 水平 `map` 下 body 位置，调试/展示输出 |
| `/my_ros1/localization/base_footprint_pose` | `geometry_msgs/PoseStamped` | `base_footprint_path.py` | **推荐下游平面导航读取的名义 base_footprint 位姿，frame_id=map** |
| `/my_ros1/localization/body_pose` | `geometry_msgs/PoseStamped` | `base_footprint_path.py` | 完整 body/IMU 在水平 `map` 下的三维姿态 |
| `/my_ros1/localization/path` | `nav_msgs/Path` | `base_footprint_path.py` | `base_footprint` 在 `map` 的路径，按距离/角度阈值追加 |
| `/my_ros1/localization/pose_status` | `std_msgs/String` | `base_footprint_path.py` | `WAITING_FOR_INITIAL_GICP` / `WAITING_FOR_ODOMETRY` / `STALE_ODOMETRY` / `TRACKING_FIXED_GICP` |

### Service

| Service | 类型 | 作用 |
|---|---|---|
| `/my_ros1/localization/relocalize` | `std_srvs/Trigger` | **手动触发一次 GICP 配准**。请求为空 `{}`；返回 `success`、包含点数和 quality 指标的 `message`。自动模式默认关闭。 |

当前已核对的 `fastgicp_relocalizer.cpp` **只注册上述一个业务 Service**。`rosservice list` 中可能出现 ROS 自带的 logger 等系统服务，并非另一个定位接口。

### 推荐给导航/上层软件的输入

- **平面导航位姿**：`/my_ros1/localization/base_footprint_pose`（PoseStamped，`map`）；或者 TF 查询 `map -> base_footprint`。
- **机体 6DoF 位姿**：`/my_ros1/localization/body_pose`（PoseStamped，`map`）。
- **路径可视化**：`/my_ros1/localization/path`，它不是控制输入。
- **运行状态**：同时观察 `/my_ros1/localization/status` 和 `/my_ros1/localization/pose_status`。`TRACKING_FIXED_GICP` 只表示已获得一次 GICP 结果并且近期收到里程计，**不代表地图匹配仍然精确**。

## 5. 五个坐标系的准确关系

记号 `T_A_B` 表示把 **B 系的坐标转换到 A 系**（ROS TF 父帧 `A`、子帧 `B`）。

```text
map                                 （水平地图/导航全局系）
 └── map_raw                         （首次 FAST-LIO2 建图/原始 PCD 系）
      └── camera_init                （本次开机 FAST-LIO2 局部里程计系）
           └── body                  （FAST-LIO2 body/IMU 系）

此外：base_footprint 为名义平面脚部参考系，计算得到 map -> base_footprint。
```

| 变换 | 获取方式 | 静态/动态 | 是否广播到 TF |
|---|---|---|---|
| `map -> map_raw` | `map_leveling.yaml`，依据**首次建图**重力标定 | 静态 | **是**，`level_map_rviz.py` 广播 `/tf_static` |
| `map_raw -> camera_init` | FastGICP 将累计扫描对齐至原始 PCD | 每次重定位时更新；当前日常只做一次 | **配置相关**：当前 YAML `publish_tf: true`，C++ 广播 `/tf`；始终发布 `map_to_odom` 话题 |
| `camera_init -> body` | FAST-LIO2 `/Odometry` | 持续动态 | **本包不广播**；其他 FAST-LIO2 节点是否广播需现场核查 |
| `base_footprint -> body` | `body_extrinsics.yaml`，名义安装平移和静止 IMU 旋转 | 名义静态外参 | **本包未单独广播**，用于内部位姿计算 |
| `map -> base_footprint` | 以上变换组合，并把姿态投影为水平 Yaw | 持续动态 | **是**，`base_footprint_path.py` 在 `publish_foot_tf=true` 时广播 `/tf` |

主要公式：

```text
T_map_body = T_map_map_raw × T_map_raw_camera_init × T_camera_init_body
T_map_foot_rigid = T_map_body × inverse(T_base_footprint_body)
```

随后 `base_footprint_path.py` 保留 `T_map_foot_rigid` 的位置，但把姿态替换成仅含水平 Yaw 的旋转矩阵，形成实际发布的 `T_map_base_footprint`。因此 `base_footprint` 是**名义平面导航系**，不是双足机器人每步实时接触的支撑脚系。

**注意 TF 树并非完全依照上面的数学链路发布**：当前本包另行直接广播 `map -> base_footprint`，不广播 `body`。因此即使能在 RViz 看到 `map -> base_footprint`，也不代表 `camera_init -> body` 已经在 `/tf` 中存在。若 RViz 提示 `No transform from body to map`，优先检查 FAST-LIO2 是否广播 `camera_init -> body`；`/Odometry` 话题有数据不等于一定存在相应 TF。

**严禁同一子帧被两个父帧广播**；特别检查 `base_footprint` 是否已经被 URDF 或其他导航节点发布。若冲突可在 `base_footprint_visualization.launch` 传 `publish_foot_tf:=false`，改由下游消费 PoseStamped。

## 6. 三个重要 YAML 配置

- `config/fastgicp_params.yaml`：`map_frame: map_raw`、`odom_topic: /Odometry`、`registered_cloud_topic: /cloud_registered`、固定开机点 `initial_imu_pose_in_map`、缓存帧数、匹配阈值、`publish_tf`、`max_data_age` 等。
- `config/map_leveling.yaml`：**首次建图时** `map_raw` 的重力方向 `gravity_up_in_map_raw`，决定 PCD 在水平 `map` 下如何旋转；不可由第二次开机的 IMU 标定结果覆盖。
- `config/body_extrinsics.yaml`：`base_footprint -> body` 名义安装旋转、平移；静止 IMU 可估算 Roll/Pitch，却不能单凭重力确定 Yaw；平移高度必须机械测量。

### 部署前必须核对的问题

1. **高度标定尚未完成**：当前 `body_extrinsics.yaml` 有 `translation_measured: false` 且 `translation_xyz: [0, 0, -1.5]`。这不应直接被当成真实的脚部到 IMU 位移！尤其要确认 **负号** 是否符合实际的 `base_footprint -> body` 坐标约定。错误会让全局 `base_footprint` 高度/位置偏移，必须机械测量后再填写和将 `translation_measured` 改为 `true`。
2. **时间新鲜度**：当前 `fastgicp_params.yaml` 的 `max_data_age` 是极大值。实机部署应改成合理正数（例如 `1.0` 秒，需考虑传感器频率和系统负载），并检查时间同步。
3. **TF 发布策略**：当前 `fastgicp_params.yaml` 实际 `publish_tf: true`，与仓库旧 README 中要求 `false` 不一致。如果现场无其他节点发布 `map_raw -> camera_init`，可保留 `true` 以使 TF 连接完整；若已有节点发布同一子帧，必须选一个权威发布者。不要仅因旧 README 文字便假设配置是 `false`。
4. **GICP 结果不是实时校正**：`base_footprint_path.py` 的 `only_first_gicp: true` 会锁定第一次有效结果。此后即使重新调用 GICP Service 且 C++ 产生新结果，脚部位姿节点也**不会接受更新**；要更新这一节点的对齐参考，按当前流程须重启相应节点并重新定位。
5. **代码安装**：当前 `CMakeLists.txt` 中 `catkin_install_python` 只安装了 `capture_reference_pose.py`，未列入 `level_map_rviz.py`、`base_footprint_path.py` 等。源码工作区中的 `roslaunch` 可依据脚本可执行权限使用，但若通过 `catkin_make install` 部署到独立 install space，需要补齐 Python 脚本安装声明并重新编译/安装。
6. **地图/点云坐标一致性**：PCD 文件必须保持在 `map_raw` 坐标系；`/cloud_registered.header.frame_id` 必须与 `/Odometry.header.frame_id` 一致（预期 `camera_init`）。GICP 的初始 pose 也必须对应 **同一个原始 PCD 地图系**。

## 7. 现场自检命令

```bash
# 原始 FAST-LIO 输出帧
rostopic echo -n 1 /Odometry/header
rostopic echo -n 1 /Odometry/child_frame_id
rostopic echo -n 1 /cloud_registered/header
rostopic hz /Odometry
rostopic hz /cloud_registered

# 确认重定位服务和定位结果
rosservice info /my_ros1/localization/relocalize
rosservice call /my_ros1/localization/relocalize "{}"
rostopic echo -n 1 /my_ros1/localization/status
rostopic echo -n 1 /my_ros1/localization/map_to_odom
rostopic echo -n 1 /my_ros1/localization/pose_status
rostopic echo -n 1 /my_ros1/localization/base_footprint_pose
rostopic echo -n 1 /my_ros1/localization/body_pose

# TF 链核查
rosrun tf tf_echo map map_raw
rosrun tf tf_echo map_raw camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo map base_footprint
rosrun tf view_frames

# 核对实际参数与发布者（避免 TF 冲突）
rosparam get /fastgicp_relocalizer/publish_tf
rosparam get /fastgicp_relocalizer/max_data_age
rostopic info /tf
rostopic info /tf_static
```

## 8. 验收条件与下一阶段

基本链路验收：`/Odometry` 和 `/cloud_registered` 实时更新 → Service 返回 `success: True` → `/my_ros1/localization/status` 为 `ALIGNED_INITIAL_ONLY` → `/my_ros1/localization/pose_status` 为 `TRACKING_FIXED_GICP` → `base_footprint_pose` 和 `body_pose` 连续更新 → RViz 中 PCD/轨迹方向一致，`base_footprint` 的 Z 轴竖直。

**仍需完成的工程化工作**：实测并修正 foot/body 平移外参；清理 TF 重复发布者；新增统一总 Launch/部署脚本；验证配准误差与失锁判定；为导航加入定位失效停车联锁；若需要在较长行走过程中持续抑制漂移，再设计周期地图匹配/重定位更新与平滑校正，不可简单把一次性配准视作长期绝对定位。
