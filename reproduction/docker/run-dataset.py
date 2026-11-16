#!/usr/bin/env python3
"""Run the three-robot upstream pipeline and preserve actual runtime evidence."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

import rosbag
from validate_results import input_bounds, validate_run, validate_snapshot

parser = argparse.ArgumentParser()
parser.add_argument('bag')
parser.add_argument('--rate', type=float, default=0.3)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--drain-timeout', type=float, default=600,
                    help='maximum wall seconds for backend completion after the last input')
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
processes = []
logs = []
socket.setdefaulttimeout(3)
expected_bounds = input_bounds(args.bag)
with rosbag.Bag(args.bag) as input_bag:
    playback_end = input_bag.get_end_time()


def start(name, command):
    stream = (args.output / (name + '.log')).open('w')
    logs.append(stream)
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    processes.append(process)
    return process


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


try:
    launch = start('launch', ['roslaunch', '/repro/s3e_headless.launch'])
    deadline = time.monotonic() + 90
    while True:
        if launch.poll() is not None:
            raise RuntimeError('roslaunch exited during startup; see launch.log')
        try:
            master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'))
            code, _, state = master.getSystemState('/dcl_reproduction')
            subscribers = dict(state[1])
            if code == 1 and all('/' + r + '/velodyne_points' in subscribers for r in 'abc'):
                break
        except (OSError, xmlrpc.client.Error):
            pass
        if time.monotonic() > deadline:
            raise RuntimeError('Three robot lidar subscribers not ready; see launch.log')
        time.sleep(0.5)
    collector = start('collector', ['python3', '/repro/collect_results.py', str(args.output)])
    recorder = start('record', ['rosbag', 'record', '--buffsize=256',
                              '-O', str(args.output / 'outputs.bag'), '-e',
                              '/[abc]/distributedMapping/.*|/[abc]/Odometry|/dcl_replay/progress|/rosout'])
    player = start('play', ['rosbag', 'play', '--clock', '--keep-alive', '--hz=100', '--delay=5',
                          '--rate=' + str(args.rate),
                          '--rate-control-topic=/dcl_replay/progress',
                          '--rate-control-max-delay=5.0', args.bag])
    drain_deadline = None
    pending_reason = 'Waiting for the last sensor input'
    while True:
        try:
            returncode = player.wait(timeout=5)
            raise RuntimeError('rosbag exited before backend completion: %s' % returncode)
        except subprocess.TimeoutExpired:
            for robot in 'abc':
                node = '/' + robot + '/laserMapping'
                code, _, uri = master.lookupNode('/dcl_reproduction', node)
                if code != 1:
                    raise RuntimeError('SLAM node disappeared: ' + node)
                try:
                    xmlrpc.client.ServerProxy(uri).getPid('/dcl_reproduction')
                except (OSError, xmlrpc.client.Error) as error:
                    raise RuntimeError('SLAM node stopped responding: ' + node) from error
        if any(process.poll() is not None for process in (launch, collector, recorder)):
            raise RuntimeError('A SLAM process exited before backend completion')
        progress_path = args.output / 'progress.json'
        if not progress_path.exists():
            continue
        progress = json.loads(progress_path.read_text())
        if progress['nonfinite_odometry']:
            raise RuntimeError('Nonfinite odometry detected')
        if progress['clock'] < playback_end + 0.05:
            continue
        # Native --keep-alive now advances only /clock, never additional sensor messages.
        if drain_deadline is None:
            drain_deadline = time.monotonic() + args.drain_timeout
        try:
            validate_snapshot(progress, expected_bounds)
        except RuntimeError as error:
            pending_reason = str(error)
        else:
            # validate_snapshot uses actual ROS-clock map reception after the
            # newest coherent completion, identically for live and final data.
            break
        if time.monotonic() > drain_deadline:
            raise RuntimeError('Backend did not finish the recorded input: ' + pending_reason)
finally:
    for process in reversed(processes):
        stop(process)
    for stream in logs:
        stream.close()

summary = validate_run(args.output, args.bag)
print(json.dumps(summary, indent=2))
