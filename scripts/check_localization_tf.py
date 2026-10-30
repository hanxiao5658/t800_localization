#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check whether all T800 localization frames are connected in ROS1 TF."""
import rospy
import tf2_ros

EDGES = [
    ('map', 'map_raw'),
    ('map_raw', 'camera_init'),
    ('camera_init', 'body'),
    ('map', 'camera_init'),
    ('map', 'body'),
    ('map', 'base_footprint'),
    ('body', 'base_footprint'),
]


def main():
    rospy.init_node('check_t800_localization_tf', anonymous=True)
    buf = tf2_ros.Buffer(cache_time=rospy.Duration(30))
    listener = tf2_ros.TransformListener(buf)  # hold reference during lookups
    rospy.sleep(1.0)
    all_ok = True
    for parent, child in EDGES:
        try:
            tf = buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(3))
            t, q = tf.transform.translation, tf.transform.rotation
            print('[OK]  {} -> {}  xyz=({:+.3f}, {:+.3f}, {:+.3f}) '
                  'quat=({:+.3f}, {:+.3f}, {:+.3f}, {:+.3f})'
                  .format(parent, child, t.x, t.y, t.z, q.x, q.y, q.z, q.w))
        except Exception as e:  # Diagnostic: report all lookup failures without crashing
            all_ok = False
            print('[FAIL] {} -> {}: {}'.format(parent, child, e))
    print('\nResult:', 'TF chain CONNECTED' if all_ok else 'TF chain INCOMPLETE')


if __name__ == '__main__':
    main()
