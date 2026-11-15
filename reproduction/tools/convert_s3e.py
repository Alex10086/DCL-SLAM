#!/usr/bin/env python3
"""Convert the full authentic S3Ev1 three-robot bag for unmodified DCL-LIO-SAM.

IMU values and absolute timestamps are preserved. LiDAR point absolute firing
 times are preserved to FLOAT32/nanosecond rounding when rebased to scan start.
Only invalid XYZ returns are removed; no points or robots are synthesized.
"""
import argparse
import hashlib
import json
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

from inspect_s3e import ROOT, ROBOTS, calibration, cloud_array


def convert(database, output):
    source = json.loads((database.parent / "source.json").read_text())
    if database.stat().st_size != source["expected_bytes"]:
        raise ValueError("Full original dataset has not finished downloading")
    with database.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    if digest != source["expected_sha256"]:
        raise ValueError("Original dataset SHA256 does not match the publisher")
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing bag: {output}")
    db = sqlite3.connect(f"file:{database}?mode=ro&immutable=1", uri=True)
    store = get_typestore(Stores.ROS1_NOETIC)
    source_store = get_typestore(Stores.ROS2_HUMBLE)
    topics = {}
    for topic_id, name, typ in db.execute("SELECT id,name,type FROM topics"):
        for robot, prefix in ROBOTS.items():
            if name in (f"/{robot}/velodyne_points", f"/{robot}/imu/data"):
                topics[topic_id] = (f"/{prefix}/" + name.split("/", 2)[2], typ, prefix)
    if len(topics) != 6:
        raise ValueError("Expected one real LiDAR and IMU stream from each of three robots")
    report = {"source": str(database), "source_sha256": digest, "output": str(output),
              "partial": False, "source_revision": source["revision"], "topics": {},
              "adaptations": ["Alpha/Bob/Carol topics mapped to a/b/c without duplicating streams",
                              "LiDAR frame called <robot>/base_link: body origin is LiDAR origin, as in default LIO-SAM config",
                              "IMU frame called <robot>/imu_link; measurements and stamps unchanged",
                              "Nonfinite XYZ returns removed; finite measured returns retained",
                              "Cloud points stable-sorted by measured firing time; scan header rebased to earliest retained return",
                              "Measured FLOAT32 time converted from signed header-relative seconds to nonnegative scan-start-relative seconds",
                              "ROS1 record timestamps preserve original ROS2 record timestamps; no cross-robot time shift"],
              "calibration_derivation": {"T_lidar_imu": "Tlc @ inverse(Tic)",
                                         "extrinsicRot": "R_lidar_imu rotates measured IMU acceleration/angular velocity into LiDAR axes",
                                         "extrinsicRPY": "R_lidar_imu because imuConverter computes q_world_imu * inverse(q_extrinsicRPY) = q_world_lidar",
                                         "extrinsicTrans": "p_lidar_imu: IMU origin in LiDAR axes; lidarPose.compose(lidar2Imu) adds this displacement and imuPose.compose(imu2Lidar) subtracts it",
                                         "orientation_assumption": "Published Imu.orientation is world_from_imu, standard sensor_msgs convention; no alternate orientation axes documented"}}
    writer = Writer(output)
    # Use uncompressed ROS1 chunks: Noetic's roslz4 rejects modern LZ4 frames.
    with writer:
        connections = {topic_id: writer.add_connection(name, typ, typestore=store)
                       for topic_id, (name, typ, prefix) in topics.items()}
        for topic_id, (name, typ, prefix) in topics.items():
            report["topics"][name] = {"type": typ, "messages": 0, "first_record_ns": None,
                                      "last_record_ns": None, "frame": f"{prefix}/base_link" if typ.endswith("PointCloud2") else f"{prefix}/imu_link"}
        ids = ",".join(str(x) for x in topics)
        rows = db.execute(f"SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ({ids}) ORDER BY timestamp,id")
        for topic_id, timestamp, raw in rows:
            name, typ, prefix = topics[topic_id]
            stats = report["topics"][name]
            msg = source_store.deserialize_cdr(raw, typ)
            msg.header = store.types["std_msgs/msg/Header"](
                seq=stats["messages"], stamp=msg.header.stamp, frame_id=stats["frame"])
            if typ.endswith("PointCloud2"):
                fields = [(f.name, f.datatype) for f in msg.fields]
                if fields != [("x",7),("y",7),("z",7),("intensity",7),("ring",4),("time",7)]:
                    raise ValueError(f"Unexpected real cloud field layout: {fields}")
                arr = cloud_array(msg)
                finite = np.isfinite(arr["x"]) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
                pts = arr[finite].copy()
                if not len(pts) or not np.isfinite(pts["time"]).all() or (pts["ring"] >= 16).any():
                    raise ValueError("Cloud has no usable returns, invalid firing time or out-of-range measured ring")
                pts = pts[np.argsort(pts["time"], kind="stable")]
                original_time = pts["time"].astype(np.float64)
                minimum = float(original_time[0])
                maximum = float(original_time[-1])
                if maximum - minimum > 0.25:
                    raise ValueError("Measured firing-time range does not represent a 10 Hz scan in seconds")
                shift_ns = round(minimum * 1e9)
                original_header_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                header_ns = original_header_ns + shift_ns
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(header_ns, 10**9)
                pts["time"] = original_time - minimum
                error_ns = float(np.abs(shift_ns + (pts["time"].astype(np.float64) - original_time)*1e9).max())
                msg.height = 1
                msg.width = len(pts)
                msg.row_step = msg.width * msg.point_step
                msg.data = np.frombuffer(pts.tobytes(), dtype=np.uint8)
                msg.is_dense = True
                stats["input_points"] = stats.get("input_points", 0) + len(arr)
                stats["retained_points"] = stats.get("retained_points", 0) + len(pts)
                stats["removed_nonfinite_xyz"] = stats.get("removed_nonfinite_xyz", 0) + int((~finite).sum())
                stats["max_firing_timestamp_rounding_ns"] = max(stats.get("max_firing_timestamp_rounding_ns", 0), error_ns)
                stats["max_scan_duration_s"] = max(stats.get("max_scan_duration_s", 0), float(pts["time"][-1]))
                stats["min_input_relative_time_s"] = min(stats.get("min_input_relative_time_s", minimum), minimum)
                stats["max_input_relative_time_s"] = max(stats.get("max_input_relative_time_s", maximum), maximum)
                if stats["messages"] == 0:
                    stats["fields"] = [{"name": f.name, "offset": f.offset, "datatype": f.datatype} for f in msg.fields]
                    stats["first_header_ns"] = header_ns
                stats["last_header_ns"] = header_ns
            else:
                q = msg.orientation
                norm = float(np.linalg.norm([q.x,q.y,q.z,q.w]))
                if not np.isfinite(norm) or abs(norm - 1) > 0.01 or msg.orientation_covariance[0] < 0:
                    raise ValueError(f"{name} has unavailable or invalid source IMU orientation")
                stats["min_quaternion_norm"] = min(stats.get("min_quaternion_norm", norm), norm)
                stats["max_quaternion_norm"] = max(stats.get("max_quaternion_norm", norm), norm)
            writer.write(connections[topic_id], timestamp, store.serialize_ros1(msg, typ))
            stats["messages"] += 1
            if stats["first_record_ns"] is None:
                stats["first_record_ns"] = timestamp
            stats["last_record_ns"] = timestamp
    db.close()
    report["output_bytes"] = output.stat().st_size
    report["messages"] = sum(x["messages"] for x in report["topics"].values())
    if any(x["messages"] == 0 for x in report["topics"].values()):
        raise ValueError("One of the three genuine robot streams is empty")
    report_path = output.with_suffix(".conversion.json")
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    overlays = {prefix: {"lio_sam": values["lio_sam_overlay"]} for prefix, values in calibration(ROOT).items()}
    overlay_path = ROOT / "data/s3e_calibration.yaml"
    overlay_path.write_text("# Published S3Ev1 camera-chain calibration, derived for DCL-LIO-SAM.\n"
                            "# Load AFTER upstream single_ugv.launch parameters. All IMU values remain raw.\n"
                            + yaml.safe_dump(overlays, sort_keys=False))
    print(json.dumps({"bag": str(output), "report": str(report_path), "calibration_override": str(overlay_path),
                      "messages": report["messages"], "output_bytes": report["output_bytes"]}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    convert(args.database.resolve(), args.output.resolve())
