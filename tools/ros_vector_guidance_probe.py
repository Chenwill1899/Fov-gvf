#!/usr/bin/env python3
"""Exercise the real controller with synthetic four-camera 3-D inputs.

Source the built ROS workspace and use an isolated ROS_DOMAIN_ID before running.
Default checks vector alignment. --paper checks strict onboard observation refusal.
--omni additionally exercises the real fused frontend and a missing front camera.
Neither mode is an Isaac navigation acceptance test.
"""
from array import array
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import PositionCommand
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def stamp_key(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--paper', action='store_true', help='exercise the paper controller')
    parser.add_argument('--omni', action='store_true', help='exercise fused 360-degree paper proposal and missing-camera refusal')
    options = parser.parse_args()
    if options.omni: options.paper = True
    executable = Path(get_package_prefix('pc_gvf')) / 'lib/pc_gvf/depth_angular_controller'
    parameters = {'body_radius': .48, 'safety_margin': .02, 'rollout_margin': .02, 'paper_pipeline_delay': .2, 'paper_enabled': options.paper, 'paper_omni_depth': options.omni, 'paper_depth_proposal': options.omni, 'horizontal_360_enabled': True, 'use_reference_line': False,
                  'max_vertical_speed': 1.0, 'max_speed': 2.0, 'speed': 2.0,
                  'max_depth_age': 0.3, 'human_intent_timeout': 0.25,
                  'field_publish_rate': 50.0, 'max_direction_rate': 1.6,
                  'command_accel_limit': 1.2, 'planning_horizon': 3.0}
    arguments = [str(executable), '--ros-args']
    for key, value in parameters.items():
        arguments += ['-p', f'{key}:={str(value).lower()}']
    with tempfile.NamedTemporaryFile(prefix='ego1p5-vector-controller-', suffix='.log', delete=False) as log:
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('vector_guidance_probe')
    odom_pub = node.create_publisher(Odometry, '/sim/odom', qos_profile_sensor_data)
    intent_pub = node.create_publisher(TwistStamped, '/human_intent', 10)
    views = ['depth', 'depth_left', 'depth_back', 'depth_right']
    info_pubs = [node.create_publisher(CameraInfo, f'/sim/{v}/camera_info', qos_profile_sensor_data) for v in views]
    depth_pubs = [node.create_publisher(Image, f'/sim/{v}/image_raw', qos_profile_sensor_data) for v in views]
    commands, rays, statuses = {}, {}, []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PositionCommand, '/position_cmd',
        lambda m: commands.update({stamp_key(m.header.stamp): (m.velocity.x,m.velocity.y,m.velocity.z)}), 100)
    node.create_subscription(String, '/depth_angular_controller/status', lambda m: statuses.append(m.data), latched)

    def field(message):
        for marker in message.markers:
            if marker.ns == 'command_ray' and len(marker.points) == 2:
                a, b = marker.points
                rays[stamp_key(marker.header.stamp)] = (b.x-a.x, b.y-a.y, b.z-a.z)
    node.create_subscription(MarkerArray, '/pc_gvf/angular_field', field, latched)

    def phase(direction, duration=1.2, depth=True, obstacle=False, intent=True, missing_front=False):
        start = time.monotonic()
        last = []
        while time.monotonic() - start < duration:
            assert process.poll() is None, f'controller exited; see {log.name}'
            stamp = node.get_clock().now().to_msg()
            odom = Odometry(); odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0; odom.pose.pose.position.z = 3.0
            odom_pub.publish(odom)
            if intent:
                request = TwistStamped(); request.header.stamp = stamp
                request.twist.linear.x, request.twist.linear.y, request.twist.linear.z = direction
                intent_pub.publish(request)
            if depth:
                for index, (info_pub, depth_pub) in enumerate(zip(info_pubs, depth_pubs)):
                    if missing_front and index == 0: continue
                    info = CameraInfo(); info.header.stamp = stamp
                    info.width = 48; info.height = 36
                    info.k = [24.0,0.0,23.5,0.0,26.7,17.5,0.0,0.0,1.0]
                    info_pub.publish(info)
                    image = Image(); image.header.stamp = stamp
                    image.width = 48; image.height = 36; image.encoding = '32FC1'; image.step = 192
                    pixels = [10.0] * (48*36)
                    if obstacle:
                        for y in range(12,24):
                            for x in range(18,30):
                                pixels[y*48+x] = 2.0
                    image.data = array('f', pixels).tobytes(); depth_pub.publish(image)
            until = time.monotonic()+0.04
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.005)
            if time.monotonic()-start > duration-0.25 and commands:
                last.append(next(reversed(commands.values())))
        assert last, 'no controller output'
        return last

    summary = {}
    try:
        phase((0.0,0.0,0.0), duration=1.5)
        for name, direction in [('front_up',(1.0,0.0,0.25)), ('left_up',(0.0,1.0,0.25)),
                                ('back_down',(-1.0,0.0,-0.25)), ('right_down',(0.0,-1.0,-0.25)),
                                ('diagonal',(1.0,0.3,0.2))]:
            outputs = phase(direction)
            v = outputs[-1]
            if options.paper:
                assert all(norm(x) < 1e-9 for x in outputs), (name, outputs)
                assert statuses[-1] == 'UNOBSERVED_BODY_ENVELOPE', (name, statuses[-1])
                summary[name] = {'velocity': v, 'status': statuses[-1]}
                continue
            assert norm(v) > 0.1, (name, v, statuses[-1:])
            cosine = sum(a*b for a,b in zip(v,direction))/norm(v)/norm(direction)
            assert cosine > 0.98, (name, cosine)
            assert v[2]*direction[2] > 0, (name, v)
            summary[name] = {'velocity': v, 'direction_cosine': cosine}
        if options.omni:
            outputs = phase((0.0,1.0,0.25), missing_front=True, duration=1.5)
            assert statuses[-1] == 'UNOBSERVED_BODY_ENVELOPE', statuses[-1]
            assert all(norm(v) < 1e-9 for v in outputs)
            assert 'OMNI_DEPTH views=3' in Path(log.name).read_text()
            summary['missing_front'] = 'three valid views reached certification; unknown body still refused'
        outputs = phase((1.0,0.0,0.0), obstacle=True, duration=2.0)
        summary['obstacle_velocity'] = outputs[-1]
        assert all(norm(v) <= 1.0+1e-6 for v in outputs), outputs
        for name, direction, opts, status in [
                ('pure_up',(0.0,0.0,1.0),{},'UNOBSERVED_BODY_ENVELOPE' if options.paper else 'DIRECTION_OUTSIDE_DEPTH_FOV'),
                ('pure_down',(0.0,0.0,-1.0),{},'UNOBSERVED_BODY_ENVELOPE' if options.paper else 'DIRECTION_OUTSIDE_DEPTH_FOV'),
                ('stale_depth',(1.0,0.0,0.2),{'depth':False},'WAITING_OMNI_DEPTH' if options.omni else 'STALE_DEPTH'),
                ('stale_intent',(1.0,0.0,0.2),{'intent':False},'STALE_INTENT'),
                ('released',(0.0,0.0,0.0),{},'ZERO_INTENT')]:
            outputs = phase(direction, **opts)
            assert all(norm(v) < 1e-9 for v in outputs), (name, outputs)
            assert statuses[-1] == status, (name, statuses[-1])
            summary[name] = status
        checked = 0
        for stamp, ray in rays.items():
            v = commands.get(stamp)
            if v is not None and norm(v) > 1e-8:
                cosine = sum(a*b for a,b in zip(v,ray))/norm(v)/norm(ray)
                assert cosine > 1.0-1e-10, ('published vector differs from planned ray', cosine)
                assert norm(v) <= 2.0+1e-8 and abs(v[2]) <= 1.0+1e-8
                checked += 1
        if options.paper:
            assert checked == 0 and all(norm(v) < 1e-9 for v in commands.values())
            summary['strict_observation_refusal'] = 'PASS; navigation acceptance remains NOT PASS'
        else:
            assert checked >= 20, checked
        summary['matched_command_rays'] = checked
        summary['controller_log'] = log.name
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()


if __name__ == '__main__':
    main()
