#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 Noetic：将 waypoints_real.yaml 可视化为 RViz MarkerArray。

该节点只负责显示，不发布 TF、Odometry 或运动控制指令。

读取：t800_navigation/config/waypoints_real.yaml
      结构：frame_id: map；waypoints: [{id, x, y, yaw}, ...]
发布：/my_ros1/localization/waypoint_markers（visualization_msgs/MarkerArray）

每个 Waypoint：
  - 圆点位置 (x, y, 0)
  - 朝向箭头 (x, y, 0)，绕 map 的 Z 轴旋转 yaw（单位 rad）
  - 文本标注 ID、XY 和 Yaw（文本高度略高于地面，便于观察）

文件路径通过 localization_rviz2.launch 提供，无需修改本脚本。
修改 YAML 后重新启动该节点，标注便会更新。
"""

import math
import os

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray


FRAME_ID = 'map'
DEFAULT_TOPIC = '/my_ros1/localization/waypoint_markers'


def make_marker(ns, marker_id, marker_type, x, y, z):
    """创建以 map 为固定坐标系的 Marker。"""
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.header.stamp = rospy.Time(0)  # 静态标注；无需等待当前 TF 时间戳
    marker.ns = ns
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(0)  # 一直显示，直到节点结束或 Marker 被删除
    return marker


def make_text(ns, marker_id, x, y, z, text, color=(1.0, 1.0, 1.0), size=0.25):
    marker = make_marker(ns, marker_id, Marker.TEXT_VIEW_FACING, x, y, z)
    marker.text = text
    marker.scale.z = size
    marker.color.r, marker.color.g, marker.color.b = color
    return marker


def load_waypoints(path):
    """读取 YAML，并检查与导航控制器约定的 map 坐标和 yaw 单位。"""
    if not os.path.isfile(path):
        raise ValueError('Waypoint YAML 不存在：{}'.format(path))
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    if not isinstance(data, dict):
        raise ValueError('YAML 顶层必须是字典')
    frame = str(data.get('frame_id', '')).lstrip('/')
    if frame != FRAME_ID:
        raise ValueError('waypoints_real.yaml 的 frame_id 必须为 map，实际是 {}'.format(frame))

    waypoints = data.get('waypoints')
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError('waypoints 必须是非空列表')

    result = []
    for i, wp in enumerate(waypoints, 1):
        try:
            name = str(wp.get('id', 'wp{}'.format(i)))
            x, y, yaw = float(wp['x']), float(wp['y']), float(wp['yaw'])
        except (TypeError, ValueError, KeyError, AttributeError) as exc:
            raise ValueError('Waypoint #{} 配置无效：{}'.format(i, exc))
        if not all(math.isfinite(n) for n in (x, y, yaw)):
            raise ValueError('Waypoint #{} 包含 NaN 或无穷值'.format(i))
        result.append((name, x, y, yaw))
    return result


def make_markers(waypoints):
    output = MarkerArray()

    # 每次重启节点时删除该话题中残留的旧标记。
    reset = Marker()
    reset.action = Marker.DELETEALL
    output.markers.append(reset)

    # map 的原点和 XYZ 标签。真正的红绿蓝坐标轴由 RViz Axes Display 绘制。
    # 原点本身固定在 (0,0,0)。
    output.markers.append(
        make_text('map_origin', 0, 0, 0, 0.32, 'map origin (0, 0, 0)',
                  (1.0, 0.96, 0.58), 0.27)
    )
    output.markers.append(make_text('map_axes_labels', 1, 1.35, 0, 0, 'X', (1, 0.25, 0.25), 0.3))
    output.markers.append(make_text('map_axes_labels', 2, 0, 1.35, 0, 'Y', (0.25, 1, 0.25), 0.3))
    output.markers.append(make_text('map_axes_labels', 3, 0, 0, 1.35, 'Z', (0.3, 0.55, 1), 0.3))

    for i, (name, x, y, yaw) in enumerate(waypoints):
        # 圆点：坐标精确使用 YAML 的 XY，Z=0。
        dot = make_marker('waypoint_positions', i, Marker.SPHERE, x, y, -1.4)
        dot.scale.x = dot.scale.y = dot.scale.z = 0.18
        dot.color.r, dot.color.g, dot.color.b = (1.0, 0.75, 0.15)
        output.markers.append(dot)

        # 箭头：位置 (x,y,0)；正 X 方向经过 yaw 旋转后为导航的最终朝向。
        # yaw 必须是弧度，和 waypoints_real.yaml 的定义一致。
        arrow = make_marker('waypoint_yaw', i, Marker.ARROW, x, y, -1.4)
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x = 0.70  # 箭头长度（米）
        arrow.scale.y = 0.13  # 箭杆直径
        arrow.scale.z = 0.18  # 箭头头部直径
        arrow.color.r, arrow.color.g, arrow.color.b = (0.18, 0.90, 0.90)
        output.markers.append(arrow)

        # 文字抬高 0.38m，仅为了清楚显示；Waypoint 真实 Z 仍是 0。
        label = make_text(
            'waypoint_labels', i, x, y, -1.4+0.38,
            '{}  ({:.2f}, {:.2f})  yaw={:.2f}rad'.format(name, x, y, yaw),
            (1.0, 0.93, 0.6), 0.22
        )
        output.markers.append(label)

    return output


def main():
    rospy.init_node('t800_waypoint_markers_rviz')
    yaml_file = rospy.get_param('~waypoints_file', '')
    topic = rospy.get_param('~marker_topic', DEFAULT_TOPIC)
    if not yaml_file:
        rospy.logfatal('必须由 launch 传入 ~waypoints_file')
        return

    try:
        waypoints = load_waypoints(os.path.expanduser(yaml_file))
    except (ValueError, OSError, yaml.YAMLError) as exc:
        rospy.logfatal('无法读取 Waypoint 配置：%s', exc)
        return

    # latched=True：RViz 即使晚于本节点启动，也能收到最后一次标注。
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    rospy.sleep(0.2)
    publisher.publish(make_markers(waypoints))
    rospy.loginfo('已从 %s 加载 %d 个 Waypoint，发布到 %s', yaml_file, len(waypoints), topic)
    rospy.spin()


if __name__ == '__main__':
    main()
