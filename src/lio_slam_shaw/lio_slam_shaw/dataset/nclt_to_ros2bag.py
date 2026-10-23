#!/usr/bin/env python3
"""
Convert NCLT sensor data (velodyne_hits.bin + ms25 IMU) to a ROS2 bag.

Usage:
    pip install rosbags numpy tqdm
    python nclt_to_ros2bag.py <sensor_data_dir> <output_bag_dir>

Example:
    python nclt_to_ros2bag.py velodyne_data/2013-04-05/ nclt_2013-04-05

Output topics:
    /points_raw   sensor_msgs/msg/PointCloud2  (fields: x,y,z,time,ring)
    /imu_raw      sensor_msgs/msg/Imu
"""

import os
import sys
import struct
import argparse

import numpy as np
from tqdm import tqdm
from pathlib import Path

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)

# Message types from typestore
Time = typestore.types['builtin_interfaces/msg/Time']
Header = typestore.types['std_msgs/msg/Header']
PointCloud2 = typestore.types['sensor_msgs/msg/PointCloud2']
PointField = typestore.types['sensor_msgs/msg/PointField']
Imu = typestore.types['sensor_msgs/msg/Imu']
Vector3 = typestore.types['geometry_msgs/msg/Vector3']
QuaternionMsg = typestore.types['geometry_msgs/msg/Quaternion']

# PointField datatype constants
PF_FLOAT32 = 7
PF_UINT16 = 4

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def usec_to_time(utime_us: int) -> Time:
    """Convert microsecond timestamp to builtin_interfaces/Time."""
    sec = int(utime_us // 1_000_000)
    nsec = int((utime_us % 1_000_000) * 1000)
    return Time(sec=sec, nanosec=nsec)


def time_to_ns(utime_us: int) -> int:
    """Convert microsecond timestamp to nanoseconds (for rosbag2 API)."""
    return int(utime_us * 1000)


def convert_vel_coords(x_s, y_s, z_s):
    """Convert raw velodyne hit to metres (NCLT convention → right-hand)."""
    scaling = 0.005  # 5 mm per count
    offset = -100.0
    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset
    return x, -y, -z


def verify_magic(data: bytes) -> bool:
    magic = 44444
    m = struct.unpack('<HHHH', data)
    return m[0] == magic and m[1] == magic and m[2] == magic and m[3] == magic


PC2_FIELDS = [
    PointField(name='x', offset=0, datatype=PF_FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PF_FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PF_FLOAT32, count=1),
    PointField(name='time', offset=12, datatype=PF_FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PF_UINT16, count=1),
]
PC2_POINT_STEP = 18  # 4+4+4+4+2
PC2_POINT_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('time', '<f4'), ('ring', '<u2'),
])


def make_pointcloud2_fast(stamp_us: int, xyz: np.ndarray, time_offsets: np.ndarray, rings: np.ndarray) -> bytes:
    """Build serialised PointCloud2 from numpy arrays. xyz: (N,3) float32, time_offsets: (N,) float32 in seconds, rings: (N,) uint16."""
    n_points = xyz.shape[0]
    buf = np.zeros(n_points, dtype=PC2_POINT_DTYPE)
    buf['x'] = xyz[:, 0]
    buf['y'] = xyz[:, 1]
    buf['z'] = xyz[:, 2]
    buf['time'] = time_offsets
    buf['ring'] = rings

    msg = PointCloud2(
        header=Header(stamp=usec_to_time(stamp_us), frame_id='velodyne'),
        height=1,
        width=n_points,
        fields=PC2_FIELDS,
        is_bigendian=False,
        point_step=PC2_POINT_STEP,
        row_step=PC2_POINT_STEP * n_points,
        data=buf.view(np.uint8),
        is_dense=True,
    )
    return typestore.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2')


def make_imu(stamp_us: int, ax, ay, az, wx, wy, wz, qx, qy, qz, qw) -> bytes:
    """Build a serialised Imu message."""
    header = Header(stamp=usec_to_time(stamp_us), frame_id='imu_link')
    msg = Imu(
        header=header,
        orientation=QuaternionMsg(x=float(qx), y=float(qy), z=float(qz), w=float(qw)),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vector3(x=float(wx), y=float(wy), z=float(wz)),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vector3(x=float(ax), y=float(ay), z=float(az)),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )
    return typestore.serialize_cdr(msg, 'sensor_msgs/msg/Imu')


# --------------------------------------------------------------------------
# Velodyne binary parser (numpy-vectorized)
# --------------------------------------------------------------------------

# Hit dtype: 3 × uint16 (x_s, y_s, z_s) + 1 × uint8 (intensity) + 1 × uint8 (layer)
HIT_DTYPE = np.dtype([('x_s', '<u2'), ('y_s', '<u2'), ('z_s', '<u2'),
                      ('intensity', 'u1'), ('layer', 'u1')])
MAGIC_BYTES = struct.pack('<HHHH', 44444, 44444, 44444, 44444)
PACKET_HEADER_SIZE = 8 + 4 + 8 + 4  # magic + num_hits + utime + padding


def write_velodyne(data_dir: str, writer, conn_points):
    """Parse velodyne_hits.bin and write PointCloud2 messages (numpy-vectorized)."""
    vel_path = os.path.join(data_dir, 'velodyne_hits.bin')
    if not os.path.exists(vel_path):
        print(f"[WARN] {vel_path} not found, skipping velodyne.")
        return

    file_size = os.path.getsize(vel_path)
    pbar = tqdm(total=file_size, desc='Velodyne', unit='B', unit_scale=True)

    scaling = np.float32(0.005)
    offset = np.float32(-100.0)

    with open(vel_path, 'rb') as f:
        # Read first packet header
        hdr = f.read(PACKET_HEADER_SIZE)
        pbar.update(PACKET_HEADER_SIZE)
        if hdr[:8] != MAGIC_BYTES:
            print("[ERROR] Invalid magic in first packet")
            return

        num_hits = struct.unpack_from('<I', hdr, 8)[0]
        last_time = struct.unpack_from('<Q', hdr, 12)[0]

        # Skip first packet's hits (just to seed last_time)
        f.seek(num_hits * 8, 1)
        pbar.update(num_hits * 8)

        scan_start_time = last_time
        last_packend_time = last_time
        # Accumulate hits for current scan
        hit_arrays = []  # list of (hits_np_array, utime, last_packend_time_before)
        total_hits_in_scan = 0

        while True:
            hdr = f.read(PACKET_HEADER_SIZE)
            if len(hdr) < PACKET_HEADER_SIZE:
                break
            pbar.update(PACKET_HEADER_SIZE)

            if hdr[:8] != MAGIC_BYTES:
                print("[WARN] Bad magic, stopping velodyne parse.")
                break

            num_hits = struct.unpack_from('<I', hdr, 8)[0]
            utime = struct.unpack_from('<Q', hdr, 12)[0]

            # Read all hits in this packet at once
            raw_hits = f.read(num_hits * 8)
            pbar.update(num_hits * 8)
            if len(raw_hits) < num_hits * 8:
                break

            hits = np.frombuffer(raw_hits, dtype=HIT_DTYPE, count=num_hits)
            hit_arrays.append((hits, utime, last_packend_time))
            total_hits_in_scan += num_hits
            last_packend_time = utime

            # Emit scan when ~100ms elapsed
            if utime - scan_start_time > 100000:
                _emit_scan(hit_arrays, total_hits_in_scan, scan_start_time,
                           scaling, offset, writer, conn_points)
                scan_start_time = utime
                hit_arrays = []
                total_hits_in_scan = 0

        # Flush remaining
        if hit_arrays:
            _emit_scan(hit_arrays, total_hits_in_scan, scan_start_time,
                       scaling, offset, writer, conn_points)

    pbar.close()
    print(f"[OK] Velodyne done.")


def _emit_scan(hit_arrays, total_hits, scan_start_time, scaling, offset, writer, conn_points):
    """Vectorized: convert accumulated packets into one PointCloud2 and write."""
    xyz_all = np.empty((total_hits, 3), dtype=np.float32)
    time_all = np.empty(total_hits, dtype=np.float32)
    ring_all = np.empty(total_hits, dtype=np.uint16)

    idx = 0
    for hits, utime, prev_packend_time in hit_arrays:
        n = len(hits)
        # Coordinate conversion (vectorized)
        x = hits['x_s'].astype(np.float32) * scaling + offset
        y = hits['y_s'].astype(np.float32) * scaling + offset
        z = hits['z_s'].astype(np.float32) * scaling + offset
        xyz_all[idx:idx+n, 0] = x
        xyz_all[idx:idx+n, 1] = -y
        xyz_all[idx:idx+n, 2] = -z

        # Time offset relative to scan start (in seconds)
        time_all[idx:idx+n] = np.float32((utime - scan_start_time) * 1e-6)
        ring_all[idx:idx+n] = hits['layer'].astype(np.uint16)
        idx += n

    serialized = make_pointcloud2_fast(scan_start_time, xyz_all, time_all, ring_all)
    writer.write(conn_points, time_to_ns(scan_start_time), serialized)


# --------------------------------------------------------------------------
# IMU (MS25) parser
# --------------------------------------------------------------------------

def write_imu(data_dir: str, writer, conn_imu):
    """Parse ms25.csv + ms25_euler.csv and write Imu messages."""
    ms25_path = os.path.join(data_dir, 'ms25.csv')
    euler_path = os.path.join(data_dir, 'ms25_euler.csv')

    if not os.path.exists(ms25_path):
        print(f"[WARN] {ms25_path} not found, skipping IMU.")
        return
    if not os.path.exists(euler_path):
        print(f"[WARN] {euler_path} not found, skipping IMU.")
        return

    ms25 = np.loadtxt(ms25_path, delimiter=',')
    ms25_euler = np.loadtxt(euler_path, delimiter=',')
    data_len = min(len(ms25), len(ms25_euler))

    # Extrinsic rotation: IMU frame → Velodyne frame
    # R_imu_to_vel = [[0,-1,0],[-1,0,0],[0,0,-1]]
    from scipy.spatial.transform import Rotation as R
    r_extR = R.from_matrix([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    r_extR_T = r_extR.inv()

    print(f"[INFO] Writing {data_len} IMU messages...")
    utime_last = 0

    for i in tqdm(range(data_len), desc='IMU'):
        utime = int(ms25[i, 0])

        # Raw IMU values
        accel_x = ms25[i, 4]
        accel_y = ms25[i, 5]
        accel_z = ms25[i, 6]
        rot_r = ms25[i, 7]
        rot_p = ms25[i, 8]
        rot_h = ms25[i, 9]

        # Euler angles for orientation
        r_angle = ms25_euler[i, 1]
        p_angle = ms25_euler[i, 2]
        h_angle = ms25_euler[i, 3]

        # Orientation in velodyne frame
        r_q = R.from_euler('xyz', [h_angle, p_angle, r_angle])
        r_lid = r_extR * r_q * r_extR_T
        q_lid = r_lid.as_quat()  # [x, y, z, w]

        # Interpolated message (midpoint between consecutive samples)
        if i > 0 and utime_last > 0:
            mid_time = (utime + utime_last) // 2
            ax_mid = (ms25[i, 4] + ms25[i - 1, 4]) * 0.5
            ay_mid = (ms25[i, 5] + ms25[i - 1, 5]) * 0.5
            az_mid = (ms25[i, 6] + ms25[i - 1, 6]) * 0.5
            wr_mid = (ms25[i, 7] + ms25[i - 1, 7]) * 0.5
            wp_mid = (ms25[i, 8] + ms25[i - 1, 8]) * 0.5
            wh_mid = (ms25[i, 9] + ms25[i - 1, 9]) * 0.5

            # Apply frame rotation (IMU → velodyne frame):
            # acc: [-ay, -ax, -az],  gyro: [-rot_p, -rot_r, -rot_h]
            serialized = make_imu(
                mid_time,
                ax=-float(ay_mid), ay=-float(ax_mid), az=-float(az_mid),
                wx=-float(wp_mid), wy=-float(wr_mid), wz=-float(wh_mid),
                qx=-q_lid[0], qy=-q_lid[1], qz=-q_lid[2], qw=-q_lid[3],
            )
            writer.write(conn_imu, time_to_ns(mid_time), serialized)

        # Original-time message
        serialized = make_imu(
            utime,
            ax=-float(accel_y), ay=-float(accel_x), az=-float(accel_z),
            wx=-float(rot_p), wy=-float(rot_r), wz=-float(rot_h),
            qx=-q_lid[0], qy=-q_lid[1], qz=-q_lid[2], qw=-q_lid[3],
        )
        writer.write(conn_imu, time_to_ns(utime), serialized)
        utime_last = utime

    print(f"[OK] IMU done ({data_len * 2} messages).")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='NCLT sensor data → ROS2 bag')
    parser.add_argument('data_dir', help='Directory with velodyne_hits.bin, ms25.csv, etc.')
    parser.add_argument('output_bag', help='Output ROS2 bag directory path')
    parser.add_argument('--no-velodyne', action='store_true', help='Skip velodyne')
    parser.add_argument('--no-imu', action='store_true', help='Skip IMU')
    args = parser.parse_args()

    output_path = Path(args.output_bag)

    with Writer(output_path, version=9) as writer:
        conn_points = writer.add_connection(
            '/points_raw', 'sensor_msgs/msg/PointCloud2',
            typestore=typestore,
        )
        conn_imu = writer.add_connection(
            '/imu_raw', 'sensor_msgs/msg/Imu',
            typestore=typestore,
        )

        if not args.no_velodyne:
            write_velodyne(args.data_dir, writer, conn_points)

        if not args.no_imu:
            write_imu(args.data_dir, writer, conn_imu)

    print(f"\n[DONE] ROS2 bag written to: {output_path}")
    print("Play with: ros2 bag play", output_path)


if __name__ == '__main__':
    main()
