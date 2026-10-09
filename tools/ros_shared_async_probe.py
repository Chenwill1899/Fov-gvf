#!/usr/bin/env python3
"""Real ROS controller regression for asynchronous local shared-map publication.

Uses deterministic simulated time, two synthetic cameras and fixed odometry.
It checks packet provenance/timing, not navigation or multirobot flight.
"""
from array import array
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import SharedObstacles
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


def stamp(value):
    ns = round(value * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def seconds(value):
    return value.sec + value.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--domain-id', required=True, type=int)
    args = parser.parse_args()
    if not args.controller.is_absolute() or not os.access(args.controller, os.X_OK):
        parser.error('--controller must be an absolute executable')
    if args.output.exists() or args.output.with_suffix('.controller.log').exists():
        parser.error('output and controller log must not already exist')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['PYTHONNOUSERSITE'] = '1'
    prefix = '/shared_async_' + str(os.getpid())
    rclpy.init()
    node = rclpy.create_node('shared_async_probe_' + str(os.getpid()))
    packets, diagnostics, stages = [], [], []
    node.create_subscription(SharedObstacles, prefix + '/shared',
        lambda m: packets.append(dict(stamp=seconds(m.header.stamp), source=m.robot_id,
            sequence=m.sequence, observed_at=list(m.observed_at),
            centers=[[p.x, p.y, p.z] for p in m.centers], frame=m.header.frame_id,
            overloaded=m.overloaded)), 100)
    node.create_subscription(String, prefix + '/controller/paper_diagnostics',
        lambda m: diagnostics.append(json.loads(m.data)), 100)
    clock_pub = node.create_publisher(Clock, prefix + '/clock', 10)
    odom_pub = node.create_publisher(Odometry, prefix + '/odom', qos_profile_sensor_data)
    intent_pub = node.create_publisher(TwistStamped, prefix + '/intent', 10)
    peer_pub = node.create_publisher(SharedObstacles, prefix + '/shared', 10)
    infos = {v: node.create_publisher(CameraInfo, prefix + '/' + v + '/info', qos_profile_sensor_data)
             for v in ('front', 'left')}
    depths = {v: node.create_publisher(Image, prefix + '/' + v + '/depth', qos_profile_sensor_data)
              for v in ('front', 'left')}
    params = dict(use_sim_time=True, paper_enabled=True, paper_shared_obstacles=True,
        shared_robot_id='async_local', shared_map_frame='world',
        shared_obstacle_topic=prefix + '/shared', odom_topic=prefix + '/odom',
        depth_topic=prefix + '/front/depth', camera_info_topic=prefix + '/front/info',
        left_depth_topic=prefix + '/left/depth', left_camera_info_topic=prefix + '/left/info',
        back_depth_topic=prefix + '/unused_back/depth', back_camera_info_topic=prefix + '/unused_back/info',
        right_depth_topic=prefix + '/unused_right/depth', right_camera_info_topic=prefix + '/unused_right/info',
        human_intent_topic=prefix + '/intent', cmd_topic=prefix + '/command',
        horizontal_360_enabled=True, max_depth_age=.3, human_intent_timeout=.5,
        paper_depth_proposal=True, angular_width=32, angular_height=24,
        camera_offset='[-2.0, 0.0, 0.0]', body_radius=.48, safety_margin=.02,
        rollout_margin=.02, command_accel_limit=1.2, paper_observation_history=.5)
    command = [str(args.controller), '--ros-args', '-r', '__node:=controller',
               '-r', '__ns:=' + prefix, '-r', '/clock:=' + prefix + '/clock']
    for key, value in params.items():
        encoded = str(value).lower() if isinstance(value, bool) else str(value)
        command += ['-p', key + ':=' + encoded]
    process = None
    result = dict(scope='real ROS controller, synthetic asynchronous two-view sensors; no flight',
                  controller=str(args.controller), controller_sha256=hashlib.sha256(args.controller.read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  domain_id=args.domain_id, command=command, checks={}, passed=False)
    try:
        with args.output.with_suffix('.controller.log').open('x') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        def pump(duration=.08):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                if process.poll() is not None:
                    raise RuntimeError('controller exited early: ' + str(process.returncode))
                rclpy.spin_once(node, timeout_sec=.01)
        deadline = time.monotonic() + 10.
        while not all(p.get_subscription_count() for p in [odom_pub, clock_pub, *infos.values(), *depths.values()]):
            if time.monotonic() > deadline:
                raise RuntimeError('ROS discovery timeout')
            pump(.05)
        def tick(t):
            c = Clock(); c.clock = stamp(t); clock_pub.publish(c)
            pump()
        def observe(view, t, distance):
            info = CameraInfo(); info.header.stamp = stamp(t); info.width = 32; info.height = 24
            info.k = [16., 0., 15.5, 0., 18., 11.5, 0., 0., 1.]
            infos[view].publish(info)
            pump(.03)
            image = Image(); image.header.stamp = stamp(t); image.width = 32; image.height = 24
            image.encoding = '32FC1'; image.step = 128
            values = [10.] * 768
            if distance is not None:
                for y in (11, 12):
                    for x in (15, 16): values[y * 32 + x] = distance
            image.data = array('f', values).tobytes(); depths[view].publish(image)
            pump(.03)
        def phase(name, now, observations, with_peer=False):
            # Publish odometry before images so each capture has an exact pose.
            for t in sorted({now, *(s for s, _ in observations.values())}):
                o = Odometry(); o.header.stamp = stamp(t); o.header.frame_id = 'world'
                o.pose.pose.orientation.w = 1.; o.pose.pose.position.z = 3.; odom_pub.publish(o)
                pump(.02)
            for view, (t, distance) in observations.items(): observe(view, t, distance)
            intent = TwistStamped(); intent.header.stamp = stamp(now); intent.twist.linear.x = .1
            intent_pub.publish(intent)
            start = len(packets)
            tick(now)
            if with_peer:
                peer = SharedObstacles(); peer.header.stamp = stamp(now); peer.header.frame_id = 'world'
                peer.robot_id = 'remote_only'; peer.session_id = 'probe'; peer.sequence = 1
                peer.centers = [Point(x=50., y=50., z=3.)]; peer.velocities = [Vector3()]
                peer.radii = [.26]; peer.observed_at = [now]; peer.observation_counts = [2]
                peer_pub.publish(peer); pump()
            tick(now + .02); tick(now + .04)
            local = [p for p in packets[start:] if p['source'] == 'async_local']
            stages.append(dict(name=name, now=now, observations=observations, local_packets=local))
            return local
        initial = phase('synchronous_start', 100., {'front': (100., 3.), 'left': (100., 3.)})
        async_packets = phase('front_frozen_side_new', 100.12, {'left': (100.12, 4.)})
        peer_packets = phase('remote_accepted_not_forwarded', 100.20, {'left': (100.20, 4.)}, True)
        expired = phase('expired_hits_not_refreshed', 100.80, {'left': (100.80, None)})
        future = phase('future_front_side_current', 100.90, {'front': (101.10, 8.), 'left': (100.90, 4.)})
        local = [p for p in packets if p['source'] == 'async_local']
        at = lambda x, y: abs(x-y) < 1e-7
        result['checks'] = dict(
            initial_local_packet=bool(initial),
            new_side_published_while_front_still_fresh=any(at(p['stamp'], 100.12) and any(at(t, 100.12) for t in p['observed_at']) for p in async_packets),
            old_track_original_timestamp_preserved=any(any(at(t, 100.) for t in p['observed_at']) for p in async_packets),
            remote_packet_really_accepted=any(d.get('shared_packets_accepted', 0) >= 1 for d in diagnostics),
            peer_not_republished=bool(peer_packets) and not any(abs(c[0]-50.) < 1e-7 and abs(c[1]-50.) < 1e-7 for p in local for c in p['centers']),
            expired_hits_absent=bool(expired) and all(not p['observed_at'] for p in expired),
            future_front_not_published=bool(future) and all(at(p['stamp'], 100.90) and all(t <= 100.92 for t in p['observed_at']) for p in future),
            original_track_age_contract=bool(local) and all(-.0200001 <= p['stamp']-t <= .5000001 for p in local for t in p['observed_at']),
            sequence_strictly_increasing=all(a['sequence'] < b['sequence'] for a,b in zip(local, local[1:])),
            frame_unchanged=bool(local) and all(p['frame'] == 'world' for p in local))
        result['passed'] = all(result['checks'].values())
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if process is not None:
            if process.poll() is None: process.send_signal(signal.SIGINT)
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.terminate()
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=3)
            result['child_returncode'] = process.returncode
            result['child_cleaned_up'] = process.poll() is not None
        result['stages'] = stages
        result['packets'] = packets
        result['diagnostic_count'] = len(diagnostics)
        result['shared_accepted_max'] = max((d.get('shared_packets_accepted',0) for d in diagnostics),default=0)
        node.destroy_node(); rclpy.shutdown()
        with args.output.open('x') as output: json.dump(result, output, indent=2)
    print(json.dumps({k:result[k] for k in ('passed','checks','child_cleaned_up','child_returncode') if k in result}, indent=2))
    if 'error' in result: print(result['error'])
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
