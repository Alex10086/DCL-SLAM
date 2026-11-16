#!/usr/bin/env python3
"""Validate actual input coverage and a connected three-robot SLAM result."""
import argparse
import json
import math
from pathlib import Path

try:
    import rosbag
    import rospy
except ImportError:
    # Snapshot validation is also usable without a ROS installation.
    rosbag = rospy = None

ROBOTS = 'abc'


def input_bounds(bag_path):
    topics = ['/' + robot + '/velodyne_points' for robot in ROBOTS]
    bounds = {}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, _ in bag.read_messages(topics=topics):
            robot = topic.split('/')[1]
            bounds.setdefault(robot, [msg.header.stamp.to_sec(), None])
            if len(bounds) == len(ROBOTS):
                break
        tail_start = rospy.Time.from_sec(bag.get_end_time() - 10)
        for topic, msg, _ in bag.read_messages(topics=topics, start_time=tail_start):
            bounds[topic.split('/')[1]][1] = msg.header.stamp.to_sec()
    if set(bounds) != set(ROBOTS) or any(end is None for _, end in bounds.values()):
        raise RuntimeError('Input bag must contain all three LiDAR streams through the sequence end')
    return bounds


def completed_round(summary):
    """Return the newest round only; historical completions cannot mask a new attempt."""
    rounds = summary.get('optimization_rounds', {})
    if not rounds:
        raise RuntimeError('Missing same-round optimization evidence')
    round_id = max(rounds, key=int)
    evidence = rounds[round_id]
    robots = evidence.get('robots', {})
    if set(robots) != set(ROBOTS) or evidence.get('aborted'):
        raise RuntimeError('Latest optimization round is incomplete or aborted')
    reference = robots['a']
    for robot in ROBOTS:
        status = robots[robot]
        if (int(round_id) <= 0 or status.get('round_id') != int(round_id) or
                status.get('participants') != [0, 1, 2] or
                sorted(status.get('order', [])) != [0, 1, 2] or
                status.get('coordinator') not in [0, 1, 2] or
                any(status.get(key) != reference.get(key)
                    for key in ('coordinator', 'participants', 'order', 'separator_keys')) or
                not status.get('committed') or status.get('aborted') or
                status.get('metadata_mismatch') or
                status.get('completed_state') != 6 or
                not status.get('completed_rotation_done') or
                not status.get('completed_pose_done') or
                status.get('completed_observed_at') is None or
                not math.isfinite(status['completed_observed_at'])):
            raise RuntimeError('Latest optimization round lacks coherent committed completion')
    keys = reference.get('separator_keys', [])
    if not keys or len(keys) % 2:
        raise RuntimeError('Missing canonical committed separator pairs')
    adjacency = {i: set() for i in range(3)}
    for index in range(0, len(keys), 2):
        endpoints = [(key >> 56) - ord('a') for key in keys[index:index + 2]]
        if any(robot not in adjacency for robot in endpoints):
            raise RuntimeError('Committed separator contains an unknown robot')
        a, b = endpoints
        adjacency[a].add(b)
        adjacency[b].add(a)
    connected, pending = set(), [0]
    while pending:
        robot = pending.pop()
        if robot not in connected:
            connected.add(robot)
            pending.extend(adjacency[robot] - connected)
    if connected != {0, 1, 2}:
        raise RuntimeError('Committed PCM separator graph is disconnected')
    return int(round_id), max(s['completed_observed_at'] for s in robots.values())


def validate_snapshot(summary, bounds):
    """Apply the same three-robot criteria to live and final measurements."""
    if summary['nonfinite_odometry']:
        raise RuntimeError('Nonfinite odometry detected')
    for robot in ROBOTS:
        stats = summary['robots'][robot]
        if stats['odometry_samples'] < 2 or stats.get('map_points', 0) == 0:
            raise RuntimeError(robot + ': missing odometry or map')
        first, last = stats['first_stamp'], stats['last_stamp']
        start, end = bounds[robot]
        # Allow IMU initialization and at most two final 10 Hz scan intervals.
        if first > start + 1.0 or last < end - 0.2:
            raise RuntimeError('%s: incomplete input coverage (head gap %.3fs, tail gap %.3fs)' %
                               (robot, first - start, end - last))
    adjacency = {i: set() for i in range(len(ROBOTS))}
    for robot0, _, robot1, _ in summary['accepted_inter_robot_loops']:
        adjacency[robot0].add(robot1)
        adjacency[robot1].add(robot0)
    connected, pending = set(), [0]
    while pending:
        robot = pending.pop()
        if robot not in connected:
            connected.add(robot)
            pending.extend(adjacency[robot] - connected)
    if connected != set(adjacency):
        missing = ','.join(ROBOTS[i] for i in sorted(set(adjacency) - connected))
        raise RuntimeError('Three-robot loop graph is disconnected; not connected to a: ' + missing)
    _, completed_at = completed_round(summary)
    for robot in ROBOTS:
        received_at = summary['robots'][robot].get('map_received_at')
        if received_at is None or not math.isfinite(received_at) or received_at <= completed_at:
            raise RuntimeError(robot + ': waiting for fresh map after joint round completion')
    return summary


def validate_run(output, bag_path):
    output = Path(output)
    summary = json.loads((output / 'summary.json').read_text())
    launch_log = (output / 'launch.log').read_text()
    if 'process has died' in launch_log or 'Large velocity, reset IMU-preintegration' in launch_log:
        raise RuntimeError('SLAM reported a node failure or divergent IMU velocity; see launch.log')
    for robot in ROBOTS:
        with (output / (robot + '_trajectory.tum')).open() as stream:
            lines = [line for line in stream if line.strip()]
        if not lines:
            raise RuntimeError(robot + ': missing odometry')
        summary['robots'][robot]['first_stamp'] = float(lines[0].split()[0])
        summary['robots'][robot]['last_stamp'] = float(lines[-1].split()[0])
    return validate_snapshot(summary, input_bounds(bag_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    print(json.dumps(validate_run(args.output, args.bag), indent=2))
