#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Estimate original FAST-LIO2 map tilt from the IMU at the start of the MAPPING bag.

Usage: python3 calibrate_map_level_from_bag.py ORIGINAL_MAPPING.bag \
             --output /root/catkin_ws/src/t800_localization/config/map_leveling.yaml

It assumes the original FAST-LIO2's camera_init orientation at initialization
coincides with the initial IMU body orientation (standard upstream behavior).
Do not use a bag from a DIFFERENT FAST-LIO2 run to calibrate this saved PCD.
"""
import argparse
import math
import os
import sys
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='ROS bag from the SAME mapping run that produced the PCD')
    parser.add_argument('--output', required=True, help='YAML output file')
    parser.add_argument('--topic', default='/livox/imu')
    parser.add_argument('--skip', type=float, default=0.0,
                        help='seconds to skip after first IMU message')
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--min-samples', type=int, default=200)
    parser.add_argument('--max-gyro-dps', type=float, default=8.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.skip < 0:
        parser.error('duration must be > 0 and skip must be >= 0')

    try:
        import rosbag
    except ImportError:
        sys.exit('ROS1 rosbag Python module not found. Run after sourcing /opt/ros/noetic/setup.bash')

    samples = []
    times = []
    angular = []
    with rosbag.Bag(args.bag, 'r') as bag:
        first = None
        for _, msg, bag_time in bag.read_messages(topics=[args.topic]):
            stamp = msg.header.stamp.to_sec() if not msg.header.stamp.to_sec() == 0.0 else bag_time.to_sec()
            if first is None:
                first = stamp
            elapsed = stamp - first
            if elapsed < args.skip:
                continue
            if elapsed > args.skip + args.duration:
                break
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z], dtype=float)
            w = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                          msg.angular_velocity.z], dtype=float)
            if np.all(np.isfinite(a)) and np.all(np.isfinite(w)):
                samples.append(a)
                angular.append(w)
                times.append(stamp)
    if len(samples) < args.min_samples:
        sys.exit('Only %d samples found in the beginning of bag; need >= %d. '
                 'Check topic, timing and the original mapping bag.' % (len(samples), args.min_samples))
    A = np.asarray(samples)
    W = np.asarray(angular)
    gyro_peak = np.degrees(np.linalg.norm(W, axis=1).max())
    med_a = np.median(A, axis=0)
    mag = np.linalg.norm(A, axis=1)
    med_mag = float(np.median(mag))
    sigma_mag = float(np.std(mag))
    if gyro_peak > args.max_gyro_dps:
        sys.exit('Calibration aborted: gyro peak %.2f deg/s indicates motion. '
                 'Use static first seconds of ORIGINAL mapping bag.' % gyro_peak)
    if med_mag < 1e-4 or sigma_mag / med_mag > 0.12:
        sys.exit('Calibration aborted: acceleration magnitude too unstable '
                 '(relative std %.2f%%). Choose a quiet stationary window.' % (100*sigma_mag/max(med_mag,1e-9)))
    up = med_a / np.linalg.norm(med_a)
    # Keep a good yaw gauge: map X = horizontal projection of raw X.
    from level_math import rotation_from_up
    R = rotation_from_up(up)
    pitch_est = math.degrees(math.atan2(-up[0], math.hypot(up[1], up[2])))
    roll_est = math.degrees(math.atan2(up[1], up[2]))
    params = {
        'map_frame': 'map',
        'raw_map_frame': 'map_raw',
        'leveling_mode': 'gravity',
        'calibrated': True,
        'gravity_up_in_map_raw': [float(v) for v in up],
        'ground_z_offset_m': 0.0,
        'manual_roll_deg': 0.0,
        'manual_pitch_deg': 0.0,
        'calibration_bag': os.path.basename(args.bag),
        'calibration_sample_count': int(len(samples)),
    }
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write('# Derived ONLY from the original PCD MAPPING run. Do not overwrite with a startup bag.\n')
        yaml.safe_dump(params, f, allow_unicode=True, sort_keys=False)
    print('Wrote:', args.output)
    print('IMU samples:', len(samples), 'stationary gyro peak: %.2f deg/s' % gyro_peak)
    print('Median acceleration (original units, g or m/s²):', med_a.tolist())
    print('Normalized IMU up in map_raw:', up.tolist())
    print('Estimated roll/pitch diagnostic: %.3f / %.3f deg' % (roll_est, pitch_est))
    print('R_map_map_raw (raw coordinate vectors -> leveled map vectors):\n', R)
    print('Check ground parallel to RViz Grid; Z offset still needs separate ground calibration.')


if __name__ == '__main__':
    main()
