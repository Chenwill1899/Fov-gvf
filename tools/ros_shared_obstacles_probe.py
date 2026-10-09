#!/usr/bin/env python3
"""Two real ROS processes with synthetic sensors. No Isaac or flight."""
from array import array
import argparse
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from pc_gvf_msgs.msg import SharedObstacles
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--all-features",action="store_true");options=parser.parse_args()
    exe = str(Path(get_package_prefix('pc_gvf'))/'lib/pc_gvf/depth_angular_controller')
    processes, logs, packets, diagnostics = [], [], [], {}
    rclpy.init()
    node = rclpy.create_node('shared_obstacles_probe')
    node.create_subscription(SharedObstacles, '/pc_gvf/shared_obstacles', lambda m: packets.append(m.robot_id), 100)
    for robot in ['probe_a', 'probe_b']:
        node.create_subscription(String, f'/{robot}/paper_diagnostics', lambda m, robot=robot: diagnostics.update({robot:json.loads(m.data)}),100)
        params=dict(body_radius=.48,safety_margin=.02,rollout_margin=.02,paper_pipeline_delay=.2,command_accel_limit=1.2,paper_enabled=True,paper_shared_obstacles=True,shared_robot_id=robot,shared_map_frame='world',horizontal_360_enabled=False,max_depth_age=.3,human_intent_timeout=.5,paper_depth_proposal=True,angular_width=32,angular_height=24)
        if options.all_features:params.update(paper_omni_depth=True,paper_dynamic_obstacles=True,paper_incremental_field=True,paper_spherical_memory=True,operator_assistance=True,depth_uncertainty_enabled=True)
        args=[exe,'--ros-args','-r',f'__node:={robot}','-r',f'/position_cmd:=/{robot}/position_cmd']
        for k,v in params.items():args+=['-p',f'{k}:={str(v).lower()}']
        with tempfile.NamedTemporaryFile(prefix=robot,suffix='.log',delete=False) as log:
            logs.append(log.name);processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT))
    odom_pub=node.create_publisher(Odometry,'/sim/odom',qos_profile_sensor_data)
    info_pub=node.create_publisher(CameraInfo,'/sim/depth/camera_info',qos_profile_sensor_data)
    depth_pub=node.create_publisher(Image,'/sim/depth/image_raw',qos_profile_sensor_data)
    intent_pub=node.create_publisher(TwistStamped,'/human_intent',10)
    bad_pub=node.create_publisher(SharedObstacles,'/pc_gvf/shared_obstacles',10)
    try:
        start=time.monotonic()
        while time.monotonic()-start<5.:
            assert all(p.poll() is None for p in processes),logs
            stamp=node.get_clock().now().to_msg()
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='world';o.pose.pose.orientation.w=1.;o.pose.pose.position.z=3.;odom_pub.publish(o)
            c=CameraInfo();c.header.stamp=stamp;c.width=32;c.height=24;c.k=[16.,0.,15.5,0.,18.,11.5,0.,0.,1.];info_pub.publish(c)
            im=Image();im.header.stamp=stamp;im.width=32;im.height=24;im.encoding='32FC1';im.step=128
            depth=[10.]*768
            for y in range(9,15):
                for x in range(12,20):depth[y*32+x]=3.
            im.data=array('f',depth).tobytes();depth_pub.publish(im)
            q=TwistStamped();q.header.stamp=stamp;q.twist.linear.x=.5;intent_pub.publish(q)
            bad=SharedObstacles();bad.header.stamp=stamp;bad.header.frame_id='unaligned';bad.robot_id='bad_frame';bad.session_id='test';bad.sequence=1;bad_pub.publish(bad)
            until=time.monotonic()+.05
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
        assert set(packets)>={'probe_a','probe_b'},(set(packets),logs)
        assert len(diagnostics)==2,(diagnostics,logs)
        for d in diagnostics.values():
            assert d['shared_packets_accepted']>0,d
            assert d['shared_packets_rejected']>0,d
        print(json.dumps(dict(published={k:packets.count(k) for k in ['probe_a','probe_b']},diagnostics=diagnostics,logs=logs,all_features=options.all_features,scope='two ROS processes, synthetic sensors; no flight'),indent=2))
    finally:
        for p in processes:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
