#!/usr/bin/env python3
"""Validate actual input coverage and a connected three-robot SLAM result."""
import argparse
import json
from pathlib import Path

import rosbag
import rospy

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
    unfinished = [r for r in ROBOTS if 6 not in summary['optimizer_states'][r]]
    if unfinished:
        raise RuntimeError('Distributed optimization did not complete for: ' + ','.join(unfinished))
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
