#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz-only diagnostic overlay for T800 FastGICP localization (ROS1 Noetic).

Does NOT broadcast TF, change /Odometry, or affect navigation/controller.
Shows: reference IMU pose and map<-odom * odom<-IMU after alignment.
"""
import math
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import (quaternion_from_euler, quaternion_matrix,
                                euler_from_matrix)


def pose_matrix(position, orientation):
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid quaternion in /Odometry or map_to_odom')
    q /= np.linalg.norm(q)
    mat = quaternion_matrix(q)
    mat[:3, 3] = [position.x, position.y, position.z]
    return mat


def marker(ns, mid, kind, xyz, yaw, text, rgb, scale):
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = rospy.Time(0)  # static map frame, independent of bag timestamps
    m.ns = ns
    m.id = mid
    m.type = kind
    m.action = Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in xyz]
    q = quaternion_from_euler(0.0, 0.0, float(yaw))
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.lifetime = rospy.Duration(0)
    if kind == Marker.TEXT_VIEW_FACING:
        m.text = text
    return m


class RelocalizationViz:
    def __init__(self):
        rospy.init_node('t800_relocalization_rviz')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        initial = rospy.get_param('~initial_imu_pose_in_map', [])
        self.reference = None
        if len(initial) == 6:
            self.reference = [float(v) for v in initial]
            rospy.loginfo('Reference IMU pose: x=%.3f y=%.3f z=%.3f yaw=%.2f deg',
                          self.reference[0], self.reference[1], self.reference[2],
                          math.degrees(self.reference[5]))
        else:
            rospy.logwarn('initial_imu_pose_in_map missing; reference marker disabled')
        self.lock = threading.Lock()
        self.odom = None
        self.t_map_odom = None
        self.odom_frame = None
        self.map_frame_from_alignment = None
        self.pub = rospy.Publisher('/my_ros1/localization/rviz_markers',
                                   MarkerArray, queue_size=1, latch=True)
        self.pose_pub = rospy.Publisher('/my_ros1/localization/imu_pose_viz',
                                        PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber('/my_ros1/localization/map_to_odom', TransformStamped,
                         self.transform_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.4), self.publish)
        rospy.loginfo('RViz overlay started. Needs /Odometry and a successful map_to_odom.')

    def odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def transform_callback(self, msg):
        with self.lock:
            self.t_map_odom = msg
            self.odom_frame = msg.child_frame_id
            self.map_frame_from_alignment = msg.header.frame_id

    def publish(self, _event):
        out = MarkerArray()
        with self.lock:
            odom = self.odom
            tf = self.t_map_odom
            expected_frame = self.odom_frame
            expected_map = self.map_frame_from_alignment

        if self.reference is not None:
            x, y, z, _r, _p, yaw = self.reference
            out.markers += self.make_pose_markers('startup_reference', 10,
                                                  (x, y, z), yaw,
                                                  'Reference (mapping run)',
                                                  (1.0, 0.73, 0.12))
        if odom is not None and tf is not None:
            if expected_frame != odom.header.frame_id:
                rospy.logwarn_throttle(5, 'Frame mismatch: map_to_odom child=%s, Odometry parent=%s',
                                       expected_frame, odom.header.frame_id)
            elif expected_map != self.map_frame:
                rospy.logwarn_throttle(5, 'Map frame mismatch: %s vs %s',
                                       expected_map, self.map_frame)
            else:
                try:
                    tmo = pose_matrix(tf.transform.translation, tf.transform.rotation)
                    toi = pose_matrix(odom.pose.pose.position, odom.pose.pose.orientation)
                    tmi = np.dot(tmo, toi)
                    xyz = tmi[:3, 3]
                    yaw = euler_from_matrix(tmi, axes='sxyz')[2]
                    out.markers += self.make_pose_markers('matched_imu', 20,
                                                          xyz, yaw,
                                                          'FastGICP IMU/body pose',
                                                          (0.10, 0.70, 1.0))
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = self.map_frame
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.pose.position.x = float(xyz[0])
                    pose_msg.pose.position.y = float(xyz[1])
                    pose_msg.pose.position.z = float(xyz[2])
                    # Full roll/pitch/yaw for accurate body pose, not the leveled marker arrow.
                    from tf.transformations import quaternion_from_matrix
                    q = quaternion_from_matrix(tmi)
                    pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = [float(v) for v in q]
                    self.pose_pub.publish(pose_msg)
                except Exception as ex:
                    rospy.logwarn_throttle(5, 'Cannot compose localization transform: %s', str(ex))
        for m in out.markers:
            m.header.frame_id = self.map_frame
        self.pub.publish(out)

    @staticmethod
    def make_pose_markers(ns, base, xyz, yaw, label, rgb):
        # Arrows use +X forward in the horizontal plane; origin denotes IMU/body, NOT feet.
        arrow = marker(ns, base, Marker.ARROW, xyz, yaw, '', rgb, (0.85, 0.12, 0.17))
        ball = marker(ns, base + 1, Marker.SPHERE, xyz, 0.0, '', rgb, (0.23, 0.23, 0.23))
        caption = marker(ns, base + 2, Marker.TEXT_VIEW_FACING,
                         (xyz[0], xyz[1], xyz[2] + 0.55), 0.0,
                         '%s\nx=%.2f  y=%.2f  z=%.2f\nyaw=%.1f deg' %
                         (label, xyz[0], xyz[1], xyz[2], math.degrees(yaw)),
                         rgb, (0.0, 0.0, 0.25))
        return [arrow, ball, caption]


if __name__ == '__main__':
    try:
        RelocalizationViz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
