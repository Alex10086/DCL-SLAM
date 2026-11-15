#!/usr/bin/env python3
"""Inspect the genuine S3Ev1 sqlite bag, sensor layout, motion and calibration."""
import argparse
import json
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = {"Alpha": "a", "Bob": "b", "Carol": "c"}
DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def cloud_array(msg):
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": [f.name for f in msg.fields],
                      "formats": [endian + DTYPES[f.datatype] for f in msg.fields],
                      "offsets": [f.offset for f in msg.fields], "itemsize": msg.point_step})
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def calibration(root):
    output = {}
    for robot, prefix in ROBOTS.items():
        text = (root / "data/Calibration" / f"{robot.lower()}.yaml").read_text()
        def matrix(name):
            values = text.split(name + ": !!opencv-matrix", 1)[1].split("data:", 1)[1].split("[", 1)[1].split("]", 1)[0]
            return np.array([float(x) for x in values.split(",")]).reshape(4, 4)
        # Published definitions: Tic maps camera to IMU; Tlc maps camera to LiDAR.
        transform = matrix("Tlc") @ np.linalg.inv(matrix("Tic"))
        current = yaml.safe_load((root / "ws/src/DCL-SLAM/config" / f"dcl_lio_sam_vlp16_params_{prefix}.yaml").read_text())["lio_sam"]
        rot = transform[:3, :3].reshape(-1).tolist()
        translation = transform[:3, 3].tolist()
        output[prefix] = {"robot": robot, "T_lidar_imu": transform.tolist(),
                          "published_composition": "Tlc @ inverse(Tic)",
                          "lio_sam_overlay": {"extrinsicTrans": translation, "extrinsicRot": rot, "extrinsicRPY": rot},
                          "upstream_extrinsicTrans": current["extrinsicTrans"],
                          "upstream_extrinsicRot": current["extrinsicRot"],
                          "translation_difference_m": float(np.linalg.norm(transform[:3, 3] - current["extrinsicTrans"])),
                          "rotation_difference_deg": float(np.degrees(np.arccos(np.clip((np.trace(transform[:3, :3] @ np.array(current["extrinsicRot"]).reshape(3, 3).T) - 1) / 2, -1, 1))))}
    return output


def inspect(database, root):
    db = sqlite3.connect(f"file:{database}?mode=ro&immutable=1", uri=True)
    store = get_typestore(Stores.ROS2_HUMBLE)
    topics = db.execute("SELECT id,name,type,serialization_format FROM topics").fetchall()
    start, end, count = db.execute("SELECT min(timestamp),max(timestamp),count(*) FROM messages").fetchone()
    output = {"source": str(database), "source_bytes": database.stat().st_size,
              "start_ns": start, "end_ns": end, "duration_s": (end-start)/1e9,
              "all_messages": count, "topics": {}, "ground_truth": {}, "calibration": calibration(root)}
    for topic_id, name, typ, serialization in topics:
        if typ not in ("sensor_msgs/msg/Imu", "sensor_msgs/msg/PointCloud2"):
            continue
        count, first, last = db.execute("SELECT count(*),min(timestamp),max(timestamp) FROM messages WHERE topic_id=?", (topic_id,)).fetchone()
        info = {"type": typ, "serialization": serialization, "count": count,
                "first_ns": first, "last_ns": last, "average_hz": (count-1)*1e9/(last-first), "samples": []}
        for direction in ("ASC", "DESC"):
            timestamp, raw = db.execute(f"SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp {direction} LIMIT 1", (topic_id,)).fetchone()
            msg = store.deserialize_cdr(raw, typ)
            sample = {"record_ns": timestamp, "header_ns": msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                      "frame": msg.header.frame_id}
            if typ.endswith("PointCloud2"):
                arr = cloud_array(msg)
                sample.update({"width": msg.width, "height": msg.height, "point_step": msg.point_step,
                               "row_step": msg.row_step, "bigendian": msg.is_bigendian,
                               "fields": [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count} for f in msg.fields],
                               "field_ranges": {f.name: [float(np.nanmin(arr[f.name])), float(np.nanmax(arr[f.name]))] for f in msg.fields},
                               "first_values": {f.name: arr[f.name][:8].tolist() for f in msg.fields}})
            else:
                q = msg.orientation
                sample.update({"quaternion_xyzw": [q.x, q.y, q.z, q.w], "quaternion_norm": float(np.linalg.norm([q.x,q.y,q.z,q.w])),
                               "orientation_covariance": msg.orientation_covariance.tolist(),
                               "acceleration": [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
                               "angular_velocity": [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]})
            info["samples"].append(sample)
        output["topics"][name] = info
    for robot in ROBOTS:
        gt = np.loadtxt(database.parent / f"{robot.lower()}_gt.txt")
        output["ground_truth"][robot] = {"rows": len(gt), "start_s": gt[0,0], "end_s": gt[-1,0],
                                        "xyz_range_m": np.ptp(gt[:,1:4], axis=0).tolist(),
                                        "path_length_m": float(np.linalg.norm(np.diff(gt[:,1:4],axis=0),axis=1).sum())}
    db.close()
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = inspect(args.database.resolve(), ROOT)
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)
