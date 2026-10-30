#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1 Noetic: calibrate T_base_footprint_body rotation from a stationary Livox IMU.

Assumptions:
  * standard upright T800 standing pose on a level floor;
  * the body frame used in FAST-LIO2 is aligned with /livox/imu sensor axes;
  * the horizontal projection of the sensor +X agrees with robot's +X forward
    when yaw_offset_deg=0. Gravity alone cannot calibrate relative yaw.
  * body origin translation in base_footprint: [0, 0, height] by design.

Live:
    python3 calibrate_body_extrinsics.py --duration 4 --height 1.5
Bag:
    python3 calibrate_body_extrinsics.py --bag standing.bag --duration 4 --height 1.5
"""
import argparse
import math
import os
import sys
import numpy as np
import yaml

# Support rosrun and direct invocation from any working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from level_math import rotation_from_up


def gyro_norm(msg):
    w = msg.angular_velocity
    return np.array([w.x, w.y, w.z], dtype=np.float64)


def acceleration(msg):
    a = msg.linear_acceleration
    return np.array([a.x, a.y, a.z], dtype=np.float64)


def collect_bag(path, topic, skip, duration):
    import rosbag
    records = []
    t0 = None
    detected_frame = ''
    with rosbag.Bag(path, 'r') as bag:
        for _, msg, t_bag in bag.read_messages(topics=[topic]):
            stamp = msg.header.stamp.to_sec() or t_bag.to_sec()
            if t0 is None:
                t0 = stamp
            elapsed = stamp - t0
            if elapsed < skip:
                continue
            if elapsed > skip + duration:
                break
            if not detected_frame:
                detected_frame = msg.header.frame_id
            records.append((acceleration(msg), gyro_norm(msg)))
    return records, detected_frame


def collect_live(topic, skip, duration):
    import rospy
    from sensor_msgs.msg import Imu
    from threading import Lock
    rospy.init_node('calibrate_t800_body_extrinsics', anonymous=True)
    values = []
    lock = Lock()
    first_wall = [None]
    frame = ['']
    import time
    def callback(msg):
        with lock:
            current = time.monotonic()
            if first_wall[0] is None:
                first_wall[0] = current
            elapsed = current - first_wall[0]
            if elapsed < skip or elapsed > skip + duration:
                return
            if not frame[0]:
                frame[0] = msg.header.frame_id
            values.append((acceleration(msg), gyro_norm(msg)))
    subscriber = rospy.Subscriber(topic, Imu, callback, queue_size=1000)
    rospy.loginfo('Collecting IMU from %s; leave T800 standing still for %.1fs', topic, skip+duration)
    rate = rospy.Rate(25)
    timeout = time.monotonic() + max(30.0, 4.0*(skip+duration))
    while not rospy.is_shutdown():
        with lock:
            first = first_wall[0]
        if first is not None and time.monotonic() - first >= skip + duration:
            break
        if time.monotonic() > timeout:
            subscriber.unregister()
            raise RuntimeError('No IMU data or sampling stalled: ' + topic)
        rate.sleep()
    subscriber.unregister()
    with lock:
        return list(values), frame[0]


def calibrate(records, min_samples, max_gyro_dps, accel_rel_std_limit, yaw_offset_deg):
    if len(records) < min_samples:
        raise ValueError('Only %d IMU samples, need >= %d' % (len(records), min_samples))
    A = np.asarray([r[0] for r in records], dtype=float)
    W = np.asarray([r[1] for r in records], dtype=float)
    if not np.all(np.isfinite(A)) or not np.all(np.isfinite(W)):
        raise ValueError('IMU contains NaN or infinity')
    mag = np.linalg.norm(A, axis=1)
    wdeg = np.degrees(np.linalg.norm(W, axis=1))
    gyro_p99 = float(np.percentile(wdeg, 99))
    rel_std = float(np.std(mag) / max(1e-9, np.median(mag)))
    if gyro_p99 > max_gyro_dps:
        raise ValueError('IMU not stationary: 99%% gyro=%.2f deg/s > %.2f' % (gyro_p99, max_gyro_dps))
    if rel_std > accel_rel_std_limit:
        raise ValueError('IMU acceleration unstable: std / median=%.3f > %.3f' % (rel_std, accel_rel_std_limit))
    if np.median(mag) < .1:
        raise ValueError('Acceleration too small; invalid or zero messages')
    up_body = np.median(A, axis=0)
    up_body = up_body / np.linalg.norm(up_body)
    # R_F_B * up_body = [0,0,1]. F +X is projected B +X unless yaw offset specified.
    R = rotation_from_up(up_body)
    yaw = math.radians(yaw_offset_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    R = Rz @ R
    # Quaternion + ZYX roll/pitch/yaw, pure NumPy (no SciPy dependency).
    trace = float(np.trace(R))
    if trace > 0:
        S = math.sqrt(trace+1.0)*2.0
        qw = .25*S
        qx = (R[2,1]-R[1,2])/S
        qy = (R[0,2]-R[2,0])/S
        qz = (R[1,0]-R[0,1])/S
    else:
        k = int(np.argmax(np.diag(R)))
        if k == 0:
            S = math.sqrt(max(0.,1+R[0,0]-R[1,1]-R[2,2]))*2
            qw = (R[2,1]-R[1,2])/S
            qx = .25*S
            qy = (R[0,1]+R[1,0])/S
            qz = (R[0,2]+R[2,0])/S
        elif k == 1:
            S = math.sqrt(max(0.,1+R[1,1]-R[0,0]-R[2,2]))*2
            qw = (R[0,2]-R[2,0])/S
            qx = (R[0,1]+R[1,0])/S
            qy = .25*S
            qz = (R[1,2]+R[2,1])/S
        else:
            S = math.sqrt(max(0.,1+R[2,2]-R[0,0]-R[1,1]))*2
            qw = (R[1,0]-R[0,1])/S
            qx = (R[0,2]+R[2,0])/S
            qy = (R[1,2]+R[2,1])/S
            qz = .25*S
    quat = np.array([qx,qy,qz,qw])
    quat /= np.linalg.norm(quat)
    roll = math.degrees(math.atan2(R[2,1],R[2,2]))
    pitch = math.degrees(math.atan2(-R[2,0],math.hypot(R[2,1],R[2,2])))
    yaw_out = math.degrees(math.atan2(R[1,0],R[0,0]))
    return {
        'up_body': up_body.tolist(),
        'quat_xyzw': [float(v) for v in quat],
        'rpy_deg': [float(roll), float(pitch), float(yaw_out)],
        'gyro_p99_dps': gyro_p99,
        'accel_rel_std': rel_std,
        'median_accel_norm_input_units': float(np.median(mag)),
        'samples': int(len(records)),
        'R': R,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topic', default='/livox/imu')
    p.add_argument('--bag', default='', help='If supplied, read this ROS1 bag instead of subscribing live')
    p.add_argument('--skip', type=float, default=0.5)
    p.add_argument('--duration', type=float, default=4.0)
    p.add_argument('--height', type=float, default=1.5, help='Nominal F->body Z in metres; replace after measurement')
    p.add_argument('--yaw-offset-deg', type=float, default=0., help='Measured body +X horizontal offset from robot front, positive toward robot +Y')
    p.add_argument('--max-gyro-dps', type=float, default=8.)
    p.add_argument('--max-accel-rel-std', type=float, default=.08)
    p.add_argument('--min-samples', type=int, default=200)
    p.add_argument('--output', default='/root/catkin_ws/src/t800_localization/config/body_extrinsics.yaml')
    args = p.parse_args()
    if args.duration <= 0 or args.skip < 0 or args.height <= 0:
        p.error('duration and height must be >0, skip >= 0')
    if args.bag:
        records, frame = collect_bag(args.bag, args.topic, args.skip, args.duration)
    else:
        records, frame = collect_live(args.topic, args.skip, args.duration)
    result = calibrate(records, args.min_samples, args.max_gyro_dps,
                       args.max_accel_rel_std, args.yaw_offset_deg)
    data = {
      'body_frame': 'body',
      'imu_frame_checked': frame,
      'parent_frame': 'base_footprint',
      'rotation_calibrated': True,
      'translation_measured': False,
      'translation_xyz': [0.0, 0.0, float(args.height)],
      'rotation_xyzw': result['quat_xyzw'],
      'yaw_offset_deg': float(args.yaw_offset_deg),
      'gravity_up_in_body': result['up_body'],
      'rpy_deg_diagnostic': result['rpy_deg'],
      'stationary_sample_count': result['samples'],
      'median_acceleration_norm_original_units': result['median_accel_norm_input_units'],
      'gyro_p99_dps': result['gyro_p99_dps'],
      'accel_rel_std': result['accel_rel_std'],
      'calibration_source': os.path.basename(args.bag) if args.bag else args.topic,
    }
    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)
    if os.path.exists(args.output):
        print('WARNING: existing YAML will be overwritten:', args.output)
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write('# Transform from nominal base_footprint to FAST-LIO2 IMU body.\n')
        f.write('# Rotation assumes body +X horizontal projection agrees with robot +X.\n')
        f.write('# translation_measured stays false until the height is physically measured.\n')
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    print('\nCALIBRATION SAVED:', args.output)
    print('Frame of IMU messages:', frame)
    print('up vector measured in body:', result['up_body'])
    print('F->body roll/pitch/yaw (deg):', result['rpy_deg'])
    print('F->body quaternion xyzw:', result['quat_xyzw'])
    print('height (nominal, measure later):', args.height)
    print('99%% gyro=%.3f deg/s, accel rel std=%.4f, samples=%d' %
          (result['gyro_p99_dps'], result['accel_rel_std'], result['samples']))
    print('CHECK body X points forward. IMU alone cannot determine yaw offset or height.')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, ImportError) as exc:
        sys.exit('Calibration failed: %s' % exc)
