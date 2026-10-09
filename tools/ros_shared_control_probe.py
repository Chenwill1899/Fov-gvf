#!/usr/bin/env python3
"""Shared-map control ablation: real ROS nodes, synthetic camera/odom; no flight."""
from array import array
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
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('shared_control_probe')
    exe = str(Path(get_package_prefix('pc_gvf'))/'lib/pc_gvf/depth_angular_controller')
    processes, logs, samples = [], [], []
    phase = ['clear', 0.]
    publishers = {}
    try:
        for group in ('local', 'observer'):
            publishers[group] = (
                node.create_publisher(Odometry, f'/{group}/odom', qos_profile_sensor_data),
                node.create_publisher(CameraInfo, f'/{group}/info', qos_profile_sensor_data),
                node.create_publisher(Image, f'/{group}/depth', qos_profile_sensor_data),
                node.create_publisher(TwistStamped, f'/{group}/intent', 10))
        for robot, group, sharing in (('local_only','local',False),('shared_local','local',True),('observer','observer',True)):
            def diagnostic(message, robot=robot):
                if robot != 'observer' and time.monotonic()-phase[1] > 1.:
                    d = json.loads(message.data)
                    assert abs(d['envelope_radius']-.58)<1e-9, d
                    samples.append(dict(phase=phase[0], robot=robot, command=math.sqrt(sum(d[k]**2 for k in ('command_x','command_y','command_z'))), status=d['status'], accepted=d['accepted']))
            node.create_subscription(String, f'/{robot}/paper_diagnostics', diagnostic, 100)
            params = dict(paper_enabled=True, paper_shared_obstacles=sharing, shared_robot_id=robot, shared_map_frame='world', horizontal_360_enabled=False, max_depth_age=.3, human_intent_timeout=.5, paper_depth_proposal=True, angular_width=32, angular_height=24, camera_offset='[-2.0, 0.0, 0.0]', paper_observation_history=.5, body_radius=.48, safety_margin=.02, rollout_margin=.02, planning_horizon=3., paper_pipeline_delay=.2, command_accel_limit=1.2)
            args = [exe, '--ros-args', '-r', f'__node:={robot}', '-r', f'/position_cmd:=/{robot}/position_cmd']
            for source, target in (('/sim/odom','odom'),('/sim/depth/camera_info','info'),('/sim/depth/image_raw','depth'),('/human_intent','intent')):
                args += ['-r', f'{source}:=/{group}/{target}']
            for key, value in params.items():
                args += ['-p', f'{key}:={str(value).lower()}']
            with tempfile.NamedTemporaryFile(prefix=robot, suffix='.log', delete=False) as log:
                logs.append(log.name)
                processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT))
        for name in ('clear', 'peer_hazard', 'cleared'):
            phase[:] = [name, time.monotonic()]
            while time.monotonic()-phase[1] < 3.:
                assert all(p.poll() is None for p in processes), logs
                stamp = node.get_clock().now().to_msg()
                for group, pubs in publishers.items():
                    odom, info, depth, intent = pubs
                    o = Odometry(); o.header.stamp=stamp; o.header.frame_id='world'; o.pose.pose.orientation.w=1.; o.pose.pose.position.z=3.; o.pose.pose.position.y=1.5 if group=='observer' else 0.; odom.publish(o)
                    c = CameraInfo(); c.header.stamp=stamp; c.width=32; c.height=24; c.k=[16.,0.,15.5,0.,18.,11.5,0.,0.,1.]; info.publish(c)
                    values = [10.]*768
                    if group=='observer' and name=='peer_hazard':
                        # Observer sees a surface at world x=1, y approx 0,
                        # invisible to the local stream. The robot-centered
                        # camera is 2m aft solely for synthetic proof coverage.
                        for y in range(11,13):
                            for x in range(23,25): values[y*32+x]=3.
                    im = Image(); im.header.stamp=stamp; im.width=32; im.height=24; im.encoding='32FC1'; im.step=128; im.data=array('f', values).tobytes(); depth.publish(im)
                    q = TwistStamped(); q.header.stamp=stamp; q.twist.linear.x=.5 if group=='local' else 0.; intent.publish(q)
                until=time.monotonic()+.04
                while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.005)
        metrics = {}
        for name in ('clear','peer_hazard','cleared'):
            metrics[name] = {}
            for robot in ('local_only','shared_local'):
                rows=[r for r in samples if r['phase']==name and r['robot']==robot]
                assert rows, (name, robot, logs)
                metrics[name][robot]=dict(samples=len(rows), mean_command=sum(r['command'] for r in rows)/len(rows), max_command=max(r['command'] for r in rows), statuses=sorted(set(r['status'] for r in rows)))
        passed = all(metrics[name]['local_only']['mean_command']>.1 for name in metrics)
        passed &= metrics['clear']['shared_local']['mean_command']>.1 and metrics['cleared']['shared_local']['mean_command']>.1
        passed &= metrics['peer_hazard']['shared_local']['mean_command']<.02
        print(json.dumps(dict(passed=passed, metrics=metrics, samples=samples, body_radius=.48, certificate_radius=.58, acceleration_limit=1.2, logs=logs, scope='three real ROS controller processes; synthetic sensors and fixed poses; published-command ablation, no flight'), indent=2))
        if not passed: raise SystemExit(1)
    finally:
        for p in processes:
            if p.poll() is None: p.send_signal(signal.SIGINT)
        for p in processes:
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired: p.terminate(); p.wait(timeout=5)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
