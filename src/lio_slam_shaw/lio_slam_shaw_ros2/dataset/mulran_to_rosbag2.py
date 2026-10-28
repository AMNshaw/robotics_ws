#!/usr/bin/env python3
"""Convert MulRan dataset (Ouster LiDAR + Xsens IMU) to ROS2 bag.

Usage:
    python3 mulran_to_rosbag2.py <raw_sequence_dir> <output_bag_dir>

Example:
    python3 mulran_to_rosbag2.py raw/MulRan/DCC01 bag/DCC01
"""

import sys
import os
import struct
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


# ---------- Config ----------
LIDAR_TOPIC = "/points_raw"
IMU_TOPIC = "/imu_raw"
LIDAR_FRAME = "velodyne"
IMU_FRAME = "imu_link"

# Ouster OS1-64 parameters
NUM_RINGS = 64
NUM_COLUMNS = 1024
FRAME_PERIOD_SEC = 0.1  # 10 Hz


def ns_to_time_msg(ns: int) -> Time:
    t = Time()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def make_pointcloud2(points: np.ndarray, stamp_ns: int) -> PointCloud2:
    """Create PointCloud2 from Nx6 array (x,y,z,intensity,ring,time)."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = ns_to_time_msg(stamp_ns)
    msg.header.frame_id = LIDAR_FRAME

    msg.height = 1
    msg.width = len(points)
    msg.is_bigendian = False
    msg.is_dense = False

    # Fields: x, y, z, intensity, ring, time
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 22  # 4+4+4+4+2+4 = 22 bytes
    msg.row_step = msg.point_step * msg.width

    # Pack binary data
    buf = bytearray(msg.row_step)
    for i, pt in enumerate(points):
        offset = i * 22
        struct.pack_into("<fffHf", buf, offset + 0,
                         pt[0], pt[1], pt[2])  # x, y, z
        # intensity at offset 12
        struct.pack_into("<f", buf, offset + 12, pt[3])
        # ring at offset 16
        struct.pack_into("<H", buf, offset + 16, int(pt[4]))
        # time at offset 18
        struct.pack_into("<f", buf, offset + 18, pt[5])

    msg.data = bytes(buf)
    return msg


def make_pointcloud2_fast(points: np.ndarray, stamp_ns: int) -> PointCloud2:
    """Create PointCloud2 from Nx6 array (x,y,z,intensity,ring,time) using numpy."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = ns_to_time_msg(stamp_ns)
    msg.header.frame_id = LIDAR_FRAME

    n = len(points)
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.is_dense = False

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 24  # 4*4 + 2 + 2(pad) + 4 -> use 24 for alignment
    msg.row_step = msg.point_step * n

    # Build structured array
    dt = np.dtype([
        ("x", np.float32), ("y", np.float32), ("z", np.float32),
        ("intensity", np.float32), ("ring", np.uint16), ("_pad", np.uint16),
        ("time", np.float32),
    ])
    structured = np.zeros(n, dtype=dt)
    structured["x"] = points[:, 0]
    structured["y"] = points[:, 1]
    structured["z"] = points[:, 2]
    structured["intensity"] = points[:, 3]
    structured["ring"] = points[:, 4].astype(np.uint16)
    structured["time"] = points[:, 5]

    msg.data = structured.tobytes()
    return msg


def make_imu_msg(stamp_ns: int, qx, qy, qz, qw, gx, gy, gz, ax, ay, az) -> Imu:
    msg = Imu()
    msg.header = Header()
    msg.header.stamp = ns_to_time_msg(stamp_ns)
    msg.header.frame_id = IMU_FRAME
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    return msg


def load_ouster_bin(filepath: str) -> np.ndarray:
    """Load MulRan Ouster bin file. Returns Nx6 (x,y,z,intensity,ring,time)."""
    raw = np.fromfile(filepath, dtype=np.float32).reshape(-1, 4)  # x, y, z, intensity
    n = len(raw)

    # Compute ring and time offset for Ouster OS1-64
    # Storage order: ring-first within each column (64 rings per column, 1024 columns)
    indices = np.arange(n)
    rings = (indices % NUM_RINGS).astype(np.float32)
    columns = indices // NUM_RINGS
    time_offsets = (columns / NUM_COLUMNS * FRAME_PERIOD_SEC).astype(np.float32)

    # Stack: x, y, z, intensity, ring, time
    points = np.column_stack([raw, rings, time_offsets])
    return points


def load_imu_csv(filepath: str):
    """Load xsens_imu.csv. Returns list of (timestamp_ns, qx,qy,qz,qw, gx,gy,gz, ax,ay,az)."""
    # Format: timestamp, qx, qy, qz, qw, euler_x, euler_y, euler_z, gx, gy, gz, ax, ay, az, mx, my, mz
    data = np.loadtxt(filepath, delimiter=",")
    results = []
    for row in data:
        stamp_ns = int(row[0])
        qx, qy, qz, qw = row[1], row[2], row[3], row[4]
        gx, gy, gz = row[8], row[9], row[10]
        ax, ay, az = row[11], row[12], row[13]
        results.append((stamp_ns, qx, qy, qz, qw, gx, gy, gz, ax, ay, az))
    return results


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    seq_dir = Path(sys.argv[1])
    out_bag = Path(sys.argv[2])

    ouster_dir = seq_dir / "Ouster"
    imu_csv = seq_dir / "xsens_imu.csv"
    stamp_csv = seq_dir / "ouster_front_stamp.csv"

    assert ouster_dir.exists(), f"Ouster dir not found: {ouster_dir}"
    assert imu_csv.exists(), f"IMU csv not found: {imu_csv}"
    assert stamp_csv.exists(), f"Stamp csv not found: {stamp_csv}"

    # Load LiDAR timestamps
    lidar_stamps = np.loadtxt(str(stamp_csv), dtype=np.int64)
    print(f"LiDAR frames: {len(lidar_stamps)}")

    # Load IMU data
    print("Loading IMU data...")
    imu_data = load_imu_csv(str(imu_csv))
    print(f"IMU samples: {len(imu_data)}")

    # Merge and sort all messages by timestamp
    print("Building message timeline...")
    timeline = []  # (timestamp_ns, type, index)
    for i, ts in enumerate(lidar_stamps):
        timeline.append((int(ts), "lidar", i))
    for i, imu in enumerate(imu_data):
        timeline.append((imu[0], "imu", i))
    timeline.sort(key=lambda x: x[0])
    print(f"Total messages: {len(timeline)}")

    # Create bag writer
    if out_bag.exists():
        import shutil
        shutil.rmtree(out_bag)

    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=str(out_bag), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    # Create topics
    lidar_topic_info = rosbag2_py.TopicMetadata(
        name=LIDAR_TOPIC,
        type="sensor_msgs/msg/PointCloud2",
        serialization_format="cdr",
    )
    imu_topic_info = rosbag2_py.TopicMetadata(
        name=IMU_TOPIC,
        type="sensor_msgs/msg/Imu",
        serialization_format="cdr",
    )
    writer.create_topic(lidar_topic_info)
    writer.create_topic(imu_topic_info)

    # Write messages
    lidar_count = 0
    imu_count = 0
    for idx, (ts, msg_type, data_idx) in enumerate(timeline):
        if msg_type == "lidar":
            bin_file = ouster_dir / f"{lidar_stamps[data_idx]}.bin"
            if not bin_file.exists():
                continue
            points = load_ouster_bin(str(bin_file))
            # Filter out zero-range points (invalid)
            valid = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2) > 0.5
            points = points[valid]
            msg = make_pointcloud2_fast(points, ts)
            writer.write(LIDAR_TOPIC, serialize_message(msg), ts)
            lidar_count += 1
            if lidar_count % 100 == 0:
                print(f"  LiDAR: {lidar_count}/{len(lidar_stamps)}", flush=True)
        else:
            imu = imu_data[data_idx]
            msg = make_imu_msg(*imu)
            writer.write(IMU_TOPIC, serialize_message(msg), ts)
            imu_count += 1

    del writer
    print(f"\nDone! Written {lidar_count} LiDAR + {imu_count} IMU messages to {out_bag}")


if __name__ == "__main__":
    main()
