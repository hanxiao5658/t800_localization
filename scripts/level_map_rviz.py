#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T800 ROS1 map leveling and RViz diagnostics (NOT a navigation controller).

Publishes static TF map -> map_raw and display-only topics.  Leaves PCD and
FastGICP untouched: PCD / aligned_scan stay in map_raw; RViz Fixed Frame map.
Map leveling is calibrated ONCE using the ORIGINAL mapping run.
"""
import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_matrix

from level_math import (rotation_from_up, rotation_from_manual, pose_matrix,
                        rpy_matrix, horizontal_yaw)


def make_marker(frame, ns, id_, type_, xyz, yaw, rgb, label=''):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time(0)
    m.ns = ns
    m.id = id_
    m.type = type_
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(c) for c in xyz]
    cy, sy = math.cos(float(yaw) / 2.0), math.sin(float(yaw) / 2.0)
    m.pose.orientation.z, m.pose.orientation.w = sy, cy
    m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
    if type_ == Marker.ARROW:
        m.scale.x, m.scale.y, m.scale.z = 0.8, 0.15, 0.18
    elif type_ == Marker.SPHERE:
        m.scale.x = m.scale.y = m.scale.z = 0.2
    else:
        m.scale.z = 0.25
        m.text = label
    m.lifetime = rospy.Duration(0)
    return m


class LevelMapViz:
    def __init__(self):
        rospy.init_node('t800_level_map_rviz')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.raw_frame = rospy.get_param('~raw_map_frame', 'map_raw')
        if self.map_frame == self.raw_frame:
            raise ValueError('map and map_raw frame names must differ')
        self.mode = rospy.get_param('~leveling_mode', 'gravity')
        self.ground_z = float(rospy.get_param('~ground_z_offset_m', 0.0))
        if self.mode == 'gravity':
            if not rospy.get_param('~calibrated', False):
                raise ValueError('Map gravity not calibrated; use the ORIGINAL mapping bag '
                                 'or launch with leveling_mode:=manual')
            self.rotation = rotation_from_up(rospy.get_param('~gravity_up_in_map_raw'))
        elif self.mode == 'manual':
            self.rotation = rotation_from_manual(
                rospy.get_param('~manual_roll_deg', 0.0),
                rospy.get_param('~manual_pitch_deg', 0.0))
            rospy.logwarn('MANUAL map leveling for visual verification ONLY. '
                          'Use original mapping IMU for accurate navigation frame.')
        else:
            raise ValueError('leveling_mode must be gravity or manual')
        self.T_map_raw = np.eye(4)
        self.T_map_raw[:3, :3] = self.rotation
        self.T_map_raw[2, 3] = self.ground_z

        self.tf_static = tf2_ros.StaticTransformBroadcaster()
        static = TransformStamped()
        static.header.stamp = rospy.Time.now()
        static.header.frame_id = self.map_frame
        static.child_frame_id = self.raw_frame
        static.transform.translation.z = self.ground_z
        quat = quaternion_from_matrix(self.T_map_raw)
        static.transform.rotation.x, static.transform.rotation.y, static.transform.rotation.z, static.transform.rotation.w = [float(q) for q in quat]
        self.tf_static.sendTransform(static)
        rospy.loginfo('Published STATIC TF %s -> %s: mode=%s z_offset=%.3fm',
                      self.map_frame, self.raw_frame, self.mode, self.ground_z)
        rospy.loginfo('R_map_map_raw = \n%s', self.rotation)

        self.lock = threading.Lock()
        self.odom = None
        self.correction = None
        self.initial = rospy.get_param('~initial_imu_pose_in_map', [])
        if len(self.initial) != 6:
            self.initial = None
            rospy.logwarn('No initial_imu_pose_in_map found; reference arrow hidden.')
        self.mark_pub = rospy.Publisher('/my_ros1/localization/level_markers', MarkerArray,
                                        latch=True, queue_size=2)
        self.pose_pub = rospy.Publisher('/my_ros1/localization/body_pose_level', PoseStamped,
                                        latch=True, queue_size=2)
        rospy.Subscriber(rospy.get_param('~odom_topic', '/Odometry'),
                         Odometry, self.odomCb, queue_size=5)
        rospy.Subscriber('/my_ros1/localization/map_to_odom', TransformStamped,
                         self.correctionCb, queue_size=5)
        rospy.Timer(rospy.Duration(0.4), self.timerCb)

    def odomCb(self, msg):
        with self.lock:
            self.odom = msg

    def correctionCb(self, msg):
        with self.lock:
            self.correction = msg

    def pose_markers(self, name, base_id, T, rgb, label):
        xyz = T[:3, 3]
        yaw = horizontal_yaw(T[:3, :3])
        if yaw is None:
            rospy.logwarn_throttle(3, 'Body +X almost vertical, yaw not observable from projection')
            return []
        return [
            make_marker(self.map_frame, name, base_id, Marker.ARROW, xyz, yaw, rgb),
            make_marker(self.map_frame, name, base_id + 1, Marker.SPHERE, xyz, 0.0, rgb),
            make_marker(self.map_frame, name, base_id + 2, Marker.TEXT_VIEW_FACING,
                        [xyz[0], xyz[1], xyz[2] + 0.65], 0.0, rgb,
                        '%s\nx=%.2f y=%.2f z=%.2f\nhorizontal heading=%.1f deg' %
                        (label, xyz[0], xyz[1], xyz[2], math.degrees(yaw)))
        ]

    def timerCb(self, _):
        with self.lock:
            odom, corr = self.odom, self.correction
        markers = MarkerArray()
        if self.initial is not None:
            try:
                T = self.T_map_raw @ rpy_matrix(self.initial)
                markers.markers.extend(self.pose_markers('map_reference', 10, T,
                                          (1.0, 0.70, 0.05), 'Mapping-run reference IMU'))
            except Exception as e:
                rospy.logwarn_throttle(5, 'Bad initial reference pose: %s', str(e))
        if odom is not None and corr is not None:
            if corr.header.frame_id != self.raw_frame or corr.child_frame_id != odom.header.frame_id:
                rospy.logwarn_throttle(5, 'Frame mismatch: expected %s -> %s, got %s -> %s',
                                       self.raw_frame, odom.header.frame_id,
                                       corr.header.frame_id, corr.child_frame_id)
            else:
                try:
                    tr = corr.transform.translation
                    ro = corr.transform.rotation
                    T_raw_odom = pose_matrix([tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w])
                    p = odom.pose.pose.position
                    q = odom.pose.pose.orientation
                    T_odom_body = pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                    T_map_body = self.T_map_raw @ T_raw_odom @ T_odom_body
                    markers.markers.extend(self.pose_markers('relocalized_body', 20,
                        T_map_body, (0.10, 0.60, 1.0), 'Current FAST-LIO body/IMU'))
                    out = PoseStamped()
                    out.header.stamp = odom.header.stamp
                    out.header.frame_id = self.map_frame
                    out.pose.position.x, out.pose.position.y, out.pose.position.z = [float(v) for v in T_map_body[:3, 3]]
                    qb = quaternion_from_matrix(T_map_body)
                    out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = [float(v) for v in qb]
                    self.pose_pub.publish(out)
                except Exception as e:
                    rospy.logwarn_throttle(5, 'Could not compose leveled IMU pose: %s', str(e))
        self.mark_pub.publish(markers)


if __name__ == '__main__':
    try:
        LevelMapViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except (ValueError, KeyError) as ex:
        rospy.logfatal('Level map launch failed: %s', ex)
