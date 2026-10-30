#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 Noetic: map-frame leveled base_footprint pose, TF axes and Path.

  T_map_body = T_map_map_raw * T_map_raw_camera_init * T_camera_init_body
  T_map_Fnominal = T_map_body * inverse(T_Fnominal_body)
  base_footprint: Fnominal translation plus horizontal projected +X heading.

The FastGICP map_to_odom result is accepted ONCE (first successful solution)
by default; later results are deliberately ignored until this node restarts.
The original GICP process may continue publishing the same transform.

No commands are published. The planar base_footprint is a virtual navigation
frame, not an exact dynamically reconstructed support-foot frame.
"""
import math
import threading
import time

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from tf.transformations import quaternion_from_matrix
from level_math import rotation_from_up, pose_matrix, horizontal_yaw


def make_pose(T, stamp, frame):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(x) for x in T[:3, 3])
    q = quaternion_from_matrix(T)
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = (float(x) for x in q)
    return msg


class FootprintPath:
    def __init__(self):
        rospy.init_node('t800_base_footprint_path')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.raw_frame = rospy.get_param('~raw_map_frame', 'map_raw')
        self.frame_foot = rospy.get_param('~foot_frame', 'base_footprint')
        self.expected_body = rospy.get_param('~body_frame', 'body')
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.correction_topic = rospy.get_param('~correction_topic', '/my_ros1/localization/map_to_odom')
        self.max_path_poses = max(10, int(rospy.get_param('~max_path_poses', 5000)))
        self.min_step = max(0., float(rospy.get_param('~min_path_step_m', .02)))
        self.min_yaw = math.radians(max(0., float(rospy.get_param('~min_path_yaw_deg', 1.0))))
        self.max_odom_wall_age = float(rospy.get_param('~max_odom_wall_age_s', 1.0))
        self.publish_tf = bool(rospy.get_param('~publish_tf', True))
        self.once = bool(rospy.get_param('~only_first_gicp', True))
        if not rospy.get_param('~calibrated', False):
            raise ValueError('Map is not calibrated: map_leveling.yaml calibrated must be true')
        up = rospy.get_param('~gravity_up_in_map_raw')
        R = rotation_from_up(up)
        self.T_map_raw = np.eye(4)
        self.T_map_raw[:3, :3] = R
        self.T_map_raw[2, 3] = float(rospy.get_param('~ground_z_offset_m', 0.))
        if not rospy.get_param('~rotation_calibrated', False):
            raise ValueError('body_extrinsics.yaml rotation_calibrated must be true')
        p_F_B = rospy.get_param('~translation_xyz')
        q_F_B = rospy.get_param('~rotation_xyzw')
        if len(p_F_B) != 3 or len(q_F_B) != 4:
            raise ValueError('body extrinsics must contain xyz(3) and xyzw(4)')
        self.T_F_B = pose_matrix(p_F_B, q_F_B)
        self.T_B_F = np.linalg.inv(self.T_F_B)
        if not rospy.get_param('~translation_measured', False):
            rospy.logwarn('F->body translation is NOMINAL, not measured; position error may remain!')
        self.lock = threading.RLock()
        self.T_raw_odom = None
        self.odom_frame = None
        self.last_msg_receipt_wall = None
        self.last_odom_stamp = None
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.path_pub = rospy.Publisher('/my_ros1/localization/path', Path, latch=True, queue_size=2)
        self.pose_pub = rospy.Publisher('/my_ros1/localization/base_footprint_pose', PoseStamped, queue_size=10)
        self.body_pub = rospy.Publisher('/my_ros1/localization/body_pose', PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/my_ros1/localization/pose_status', String, latch=True, queue_size=2)
        self.br = tf2_ros.TransformBroadcaster()
        self.sub_corr = rospy.Subscriber(self.correction_topic, TransformStamped, self.corr_cb, queue_size=2)
        self.sub_odom = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Timer(rospy.Duration(.5), self.status_timer)
        rospy.loginfo('T800 footprint path initialized. map=%s raw=%s odom=%s, first GICP only=%s',
                      self.map_frame, self.raw_frame, self.odom_topic, self.once)

    def corr_cb(self, msg):
        with self.lock:
            if msg.header.frame_id != self.raw_frame:
                rospy.logwarn_throttle(3., 'Ignore GICP: parent=%s, expected %s', msg.header.frame_id, self.raw_frame)
                return
            if not msg.child_frame_id:
                return
            if self.T_raw_odom is not None and self.once:
                return
            t = msg.transform.translation
            r = msg.transform.rotation
            try:
                T = pose_matrix([t.x,t.y,t.z],[r.x,r.y,r.z,r.w])
            except ValueError as exc:
                rospy.logerr('Reject invalid GICP transform: %s', exc)
                return
            if not np.all(np.isfinite(T)):
                return
            changed = self.T_raw_odom is not None
            self.T_raw_odom = T
            self.odom_frame = msg.child_frame_id
            if changed:
                self.path.poses = []
            rospy.loginfo('Accepted %s GICP map_raw->%s; future odometry uses fixed correction.',
                          'updated' if changed else 'INITIAL', self.odom_frame)

    def odom_cb(self, msg):
        with self.lock:
            self.last_msg_receipt_wall = time.monotonic()
            if self.T_raw_odom is None:
                return
            if msg.header.frame_id != self.odom_frame:
                rospy.logwarn_throttle(3., 'Odom parent=%s does not match GICP child=%s',
                                       msg.header.frame_id, self.odom_frame)
                return
            if msg.child_frame_id != self.expected_body:
                rospy.logwarn_throttle(3., 'Odom child=%s expected=%s; check FAST-LIO body / Livox extrinsics.',
                                       msg.child_frame_id, self.expected_body)
                return
            stamp = msg.header.stamp
            if stamp.to_sec() == 0.0:
                rospy.logwarn_throttle(3., 'Odometry timestamp is zero: cannot publish meaningful path')
                return
            if self.last_odom_stamp is not None and stamp < self.last_odom_stamp:
                rospy.logwarn('ROS time moved backwards. Clearing path; restart GICP if odometry origin changed.')
                self.path.poses = []
            self.last_odom_stamp = stamp
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            try:
                T_odom_B = pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
                T_map_B = self.T_map_raw @ self.T_raw_odom @ T_odom_B
                T_map_F_rigid = T_map_B @ self.T_B_F
            except ValueError as exc:
                rospy.logwarn_throttle(3, 'Bad Odometry pose: %s', exc)
                return
            yaw = horizontal_yaw(T_map_F_rigid[:3,:3])
            if yaw is None:
                rospy.logwarn_throttle(3., 'Footprint +X vertical: horizontal yaw undefined')
                return
            T_map_F = np.eye(4)
            T_map_F[:3, 3] = T_map_F_rigid[:3, 3]
            c, s = math.cos(yaw), math.sin(yaw)
            T_map_F[:3,:3] = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            pose = make_pose(T_map_F, stamp, self.map_frame)
            self.pose_pub.publish(pose)
            self.body_pub.publish(make_pose(T_map_B, stamp, self.map_frame))
            self._append_path(pose, yaw)
            if self.publish_tf:
                tfmsg = TransformStamped()
                tfmsg.header = pose.header
                tfmsg.child_frame_id = self.frame_foot
                tfmsg.transform.translation.x = pose.pose.position.x
                tfmsg.transform.translation.y = pose.pose.position.y
                tfmsg.transform.translation.z = pose.pose.position.z
                tfmsg.transform.rotation = pose.pose.orientation
                self.br.sendTransform(tfmsg)

    def _append_path(self, pose, yaw):
        pts = self.path.poses
        append = True
        if pts:
            prev = pts[-1]
            dx = pose.pose.position.x - prev.pose.position.x
            dy = pose.pose.position.y - prev.pose.position.y
            dz = pose.pose.position.z - prev.pose.position.z
            prev_yaw = 2 * math.atan2(prev.pose.orientation.z, prev.pose.orientation.w)
            angle = math.atan2(math.sin(yaw-prev_yaw), math.cos(yaw-prev_yaw))
            append = math.sqrt(dx*dx+dy*dy+dz*dz) >= self.min_step or abs(angle) >= self.min_yaw
        if append:
            pts.append(pose)
            if len(pts) > self.max_path_poses:
                del pts[:len(pts) - self.max_path_poses]
            self.path.header.stamp = pose.header.stamp
            self.path_pub.publish(self.path)

    def status_timer(self, _):
        with self.lock:
            if self.T_raw_odom is None:
                state = 'WAITING_FOR_INITIAL_GICP'
            elif self.last_msg_receipt_wall is None:
                state = 'WAITING_FOR_ODOMETRY'
            elif time.monotonic()-self.last_msg_receipt_wall > self.max_odom_wall_age:
                state = 'STALE_ODOMETRY'
            else:
                state = 'TRACKING_FIXED_GICP'
        self.status_pub.publish(String(data=state))


if __name__ == '__main__':
    try:
        FootprintPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except (ValueError, KeyError) as ex:
        rospy.logfatal('Could not start footprint path node: %s', ex)
