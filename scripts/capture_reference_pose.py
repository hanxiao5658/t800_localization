#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""记录首次建图会话中，FAST-LIO 的 Odometry 子坐标系在地图构建坐标系中的参考位姿。

使用场景：机器人走到将来固定开机的地点，停稳，运行一次本脚本。
输出为定位算法「初始位姿候选」，不是已配准的全局定位结果。
特别注意：Pose 属于 Odometry 的 child_frame_id（通常是 IMU/body），
不是 base_footprint，也不是直接测量的 MID360 雷达光学中心。
"""
import math
import os
import tempfile
from datetime import datetime

import rospy
import yaml
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def main():
    rospy.init_node('capture_reference_pose')
    topic = rospy.get_param('~odom_topic', '/Odometry')
    path = os.path.abspath(os.path.expanduser(rospy.get_param(
        '~output_file', '~/catkin_ws/src/t800_localization/config/startup_reference.yaml')))
    timeout = float(rospy.get_param('~timeout', 15.0))

    rospy.loginfo('等待 FAST-LIO 里程计 %s。请确保机器人已到达指定固定开机位并停稳。', topic)
    try:
        msg = rospy.wait_for_message(topic, Odometry, timeout=timeout)
    except rospy.ROSException as e:
        rospy.logerr('未收到里程计：%s', str(e))
        return

    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    frame = msg.header.frame_id
    child = msg.child_frame_id
    if not frame or not child:
        rospy.logerr('Odometry 没有有效的 frame_id/child_frame_id，拒绝写入参考位姿。')
        return

    quat = [q.x, q.y, q.z, q.w]
    quat_norm = math.sqrt(sum(v*v for v in quat))
    if quat_norm < 1e-9 or not all(math.isfinite(v) for v in [p.x,p.y,p.z,*quat]):
        rospy.logerr('里程计位姿数据无效，不保存。')
        return
    quat = [v / quat_norm for v in quat]
    roll, pitch, yaw = euler_from_quaternion(quat)

    data = {
        'note': '首次建图会话：已知固定开机位置处的 FAST-LIO 参考位姿；非重定位计算结果',
        'captured_at': datetime.now().isoformat(timespec='seconds'),
        'source_topic': topic,
        'parent_frame': frame,
        'child_frame': child,
        'header_stamp_sec': msg.header.stamp.to_sec(),
        'position': {'x': float(p.x), 'y': float(p.y), 'z': float(p.z)},
        'orientation_xyzw': {'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]},
        'rpy_rad': {'roll': float(roll), 'pitch': float(pitch), 'yaw': float(yaw)},
        'yaw_deg': float(math.degrees(yaw)),
    }

    folder = os.path.dirname(path)
    os.makedirs(folder, exist_ok=True)
    tmp_name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                         prefix='.startup_reference_', suffix='.tmp',
                                         dir=folder, delete=False) as f:
            tmp_name = f.name
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp_name, path)
        rospy.loginfo('已保存参考位姿到 %s', path)
        rospy.loginfo('frame: %s -> %s | xyz=(%.3f, %.3f, %.3f) yaw=%.4f rad (%.1f deg)',
                      frame, child, p.x, p.y, p.z, yaw, math.degrees(yaw))
        rospy.logwarn('这不是脚尖中点位姿：后续需要转换为 base_footprint，且必须与 PCD 地图坐标系一致。')
    except (OSError, yaml.YAMLError) as e:
        rospy.logerr('保存失败：%s', str(e))
        if tmp_name and os.path.exists(tmp_name):
            os.unlink(tmp_name)


if __name__ == '__main__':
    main()
