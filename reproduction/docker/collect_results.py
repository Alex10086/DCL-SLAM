#!/usr/bin/env python3
"""Subscribe to actual SLAM outputs; save trajectories, final maps, and counters."""
import argparse
import collections
import json
import math
from pathlib import Path
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from dcl_slam.msg import global_descriptor, loop_info, optimization_state

parser = argparse.ArgumentParser()
parser.add_argument('output', type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
lock = threading.Lock()
counts = collections.Counter()
last_maps = {}
trajectories = {r: [] for r in 'abc'}
path_sizes = {}
optimizer_states = {r: set() for r in 'abc'}
accepted_loops = set()
nonfinite = collections.Counter()
latest_odometry = {}
optimization_rounds = {}
map_timing = {}


def record_round(robot, msg, observed_at):
    optimizer_states[robot].add(msg.state)
    if not msg.round_id:
        return
    key = str(msg.round_id)
    evidence = optimization_rounds.setdefault(key, {'robots': {}, 'aborted': False})
    previous = evidence['robots'].get(robot, {})
    status = {name: getattr(msg, name) for name in
              ('round_id', 'coordinator', 'state', 'committed', 'aborted',
               'rotation_done', 'pose_done')}
    status.update(participants=sorted(msg.participants), order=list(msg.order),
                  separator_keys=list(msg.separator_keys),
                  proposal_keys=list(msg.proposal_keys), observed_at=observed_at)
    # PREPARE/READY legitimately have no order and locally proposed edges.
    # Freeze metadata only when COMMIT first supplies the common plan.
    committed_metadata = previous.get('committed_metadata')
    metadata = {name: status[name] for name in
                ('coordinator', 'participants', 'order', 'separator_keys')}
    status['metadata_mismatch'] = previous.get('metadata_mismatch', False) or (
        committed_metadata is not None and metadata != committed_metadata)
    status['committed_metadata'] = (
        committed_metadata if committed_metadata is not None else
        metadata if status['committed'] else None)
    status['aborted'] = status['aborted'] or previous.get('aborted', False)
    evidence['aborted'] = evidence['aborted'] or status['aborted']
    status['completed_state'] = previous.get('completed_state')
    status['completed_observed_at'] = previous.get('completed_observed_at')
    status['completed_rotation_done'] = previous.get('completed_rotation_done', False)
    status['completed_pose_done'] = previous.get('completed_pose_done', False)
    if msg.state == 6 and status['completed_observed_at'] is None:
        status.update(completed_state=6, completed_observed_at=observed_at,
                      completed_rotation_done=msg.rotation_done,
                      completed_pose_done=msg.pose_done)
    evidence['robots'][robot] = status
    # Walltime-generated IDs order rounds, unlike paused/replayed ROS time.
    for old in sorted(optimization_rounds, key=int)[:-32]:
        del optimization_rounds[old]


def callback(msg, key):
    robot, kind = key
    with lock:
        counts[robot + '/' + kind] += 1
        if kind == 'odometry':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            row = [msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            if not all(math.isfinite(v) for v in row):
                nonfinite[robot] += 1
            else:
                trajectories[robot].append(row)
                latest_odometry[robot] = msg
                if len(latest_odometry) == 3:
                    # Relay an actual odometry message from the slowest stream.
                    progress_publisher.publish(min(latest_odometry.values(),
                                                   key=lambda odom: odom.header.stamp.to_nsec()))
        elif kind == 'globalMap':
            last_maps[robot] = msg
            map_timing[robot] = {'map_received_at': rospy.Time.now().to_sec(),
                                 'map_header_stamp': msg.header.stamp.to_sec()}
        elif kind == 'path':
            path_sizes[robot] = len(msg.poses)
        elif kind == 'optimizationState':
            record_round(robot, msg, rospy.Time.now().to_sec())
        elif kind == 'loopInfo' and msg.robot0 != msg.robot1 and msg.noise < 888:
            accepted_loops.add((msg.robot0, msg.index0, msg.robot1, msg.index1))


def save_progress(_event):
    with lock:
        snapshot = {
            'clock': rospy.Time.now().to_sec(),
            'message_counts': dict(counts),
            'optimizer_states': {r: sorted(s) for r, s in optimizer_states.items()},
            'optimization_rounds': optimization_rounds,
            'accepted_inter_robot_loops': [list(edge) for edge in sorted(accepted_loops)],
            'nonfinite_odometry': dict(nonfinite),
            'robots': {},
        }
        for robot, rows in trajectories.items():
            cloud = last_maps.get(robot)
            stats = {'odometry_samples': len(rows),
                     'map_points': cloud.width * cloud.height if cloud is not None else 0}
            stats.update(map_timing.get(robot, {}))
            if rows:
                stats.update(first_stamp=rows[0][0], last_stamp=rows[-1][0])
            snapshot['robots'][robot] = stats
        serialized = json.dumps(snapshot)
    temporary = args.output / 'progress.json.tmp'
    temporary.write_text(serialized)
    temporary.replace(args.output / 'progress.json')


rospy.init_node('dcl_result_collector', anonymous=True)
progress_publisher = rospy.Publisher('/dcl_replay/progress', Odometry, queue_size=1)
subscriptions = []
for robot in 'abc':
    for kind, msg_type, suffix in [
        ('odometry', Odometry, 'Odometry'),
        ('globalMap', PointCloud2, 'distributedMapping/globalMap'),
        ('path', RosPath, 'distributedMapping/path'),
        ('globalDescriptors', global_descriptor, 'distributedMapping/globalDescriptors'),
        ('loopInfo', loop_info, 'distributedMapping/loopInfo'),
        ('optimizationState', optimization_state, 'distributedMapping/optimizationState'),
    ]:
        subscriptions.append(rospy.Subscriber('/' + robot + '/' + suffix, msg_type,
                                              callback, (robot, kind), queue_size=100,
                                              buff_size=32 * 1024 * 1024))
progress_timer = rospy.Timer(rospy.Duration(1.0), save_progress)
try:
    rospy.spin()
finally:
    progress_timer.shutdown()
    with lock:
        report = {'message_counts': dict(counts), 'path_sizes': path_sizes,
                  'optimizer_states': {r: sorted(s) for r, s in optimizer_states.items()},
                  'optimization_rounds': optimization_rounds,
                  'accepted_inter_robot_loops': [list(x) for x in sorted(accepted_loops)],
                  'nonfinite_odometry': dict(nonfinite), 'robots': {}}
        for robot, rows in trajectories.items():
            np.savetxt(str(args.output / (robot + '_trajectory.tum')), np.asarray(rows), fmt='%.9f')
            points = np.asarray(rows)
            info = {'odometry_samples': len(rows)}
            info.update(map_timing.get(robot, {}))
            if len(rows):
                xyz = points[:, 1:4]
                info.update(first_stamp=float(points[0, 0]), last_stamp=float(points[-1, 0]),
                            duration_s=float(points[-1, 0] - points[0, 0]),
                            travel_m=float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
                            extent_m=(xyz.max(axis=0) - xyz.min(axis=0)).tolist())
            if robot in last_maps:
                cloud = last_maps[robot]
                data = np.asarray(list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'),
                                                                 skip_nans=True)), dtype='<f4').reshape(-1, 3)
                info['map_points'] = len(data)
                with (args.output / (robot + '_map.pcd')).open('wb') as out:
                    out.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                               'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                               % (len(data), len(data))).encode())
                    out.write(data.tobytes())
            report['robots'][robot] = info
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
