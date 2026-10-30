#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish the FIRST accepted FastGICP map_raw -> camera_init as static TF.

ROS1 Noetic. Keeps the existing FastGICP publish_tf=false and the current
base_footprint_path.py untouched. Only ONE node may own this TF edge.

IMPORTANT: This is valid only while FAST-LIO uses the same odom origin.
Restart this bridge and the localization pipeline when FAST-LIO restarts.
"""
import copy
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class InitialGicpStaticTf:
    def __init__(self):
        rospy.init_node('t800_initial_gicp_tf_static')
        self.raw_frame = rospy.get_param('~raw_map_frame', 'map_raw').lstrip('/')
        self.odom_frame = rospy.get_param('~odom_frame', 'camera_init').lstrip('/')
        self.topic = rospy.get_param(
            '~correction_topic', '/my_ros1/localization/map_to_odom')
        if not self.raw_frame or not self.odom_frame or self.raw_frame == self.odom_frame:
            raise ValueError('Invalid TF parent/child names')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.lock = threading.Lock()
        self.saved = None
        self.sub = rospy.Subscriber(self.topic, TransformStamped, self.callback, queue_size=5)
        rospy.loginfo('Waiting for first GICP: %s -> %s from %s',
                      self.raw_frame, self.odom_frame, self.topic)

    def callback(self, msg):
        parent, child = msg.header.frame_id.lstrip('/'), msg.child_frame_id.lstrip('/')
        if (parent, child) != (self.raw_frame, self.odom_frame):
            rospy.logwarn_throttle(5, 'Reject GICP TF %s -> %s; expected %s -> %s',
                                   parent, child, self.raw_frame, self.odom_frame)
            return
        t, q = msg.transform.translation, msg.transform.rotation
        vals = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            rospy.logerr_throttle(5, 'Reject nonfinite GICP transform')
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1e-8:
            rospy.logerr_throttle(5, 'Reject invalid zero GICP quaternion')
            return
        with self.lock:
            if self.saved is not None:
                # This bridge deliberately freezes the initial alignment.
                old = self.saved.transform
                dist = math.sqrt((t.x-old.translation.x)**2 +
                                 (t.y-old.translation.y)**2 +
                                 (t.z-old.translation.z)**2)
                dot = abs((q.x*old.rotation.x + q.y*old.rotation.y +
                           q.z*old.rotation.z + q.w*old.rotation.w) / norm)
                dot = max(0.0, min(1.0, dot))
                angle = 2.0 * math.acos(dot)
                if dist > 0.01 or angle > math.radians(0.5):
                    rospy.logwarn_throttle(
                        5, 'New GICP correction differs from LOCKED static TF '
                        '(%.3f m, %.2f deg); ignoring. Restart localization '
                        'pipeline to change odom origin or alignment.',
                        dist, math.degrees(angle))
                return
            transform = copy.deepcopy(msg)
            transform.header.stamp = rospy.Time.now()  # ignored by /tf_static consumers
            transform.header.frame_id = self.raw_frame
            transform.child_frame_id = self.odom_frame
            transform.transform.rotation.x /= norm
            transform.transform.rotation.y /= norm
            transform.transform.rotation.z /= norm
            transform.transform.rotation.w /= norm
            self.saved = transform
            self.broadcaster.sendTransform(transform)
            rospy.loginfo('LOCKED STATIC TF %s -> %s, xyz=(%.3f, %.3f, %.3f). '
                          'No further GICP needed for this FAST-LIO session.',
                          self.raw_frame, self.odom_frame, t.x, t.y, t.z)


if __name__ == '__main__':
    try:
        InitialGicpStaticTf()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
