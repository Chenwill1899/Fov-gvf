"""Run from a sourced Humble shell on an isolated ROS_DOMAIN_ID, with no motion."""
import argparse
import csv
import json
import os
import subprocess
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data as qos
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image

root=Path(__file__).resolve().parents[3]
archive=root/'docs/performance/current_baseline_20261105_141800/input'
meta=json.loads((archive/'simulation.sensors.json').read_text())
raw=(archive/'last_depth.bin').read_bytes()
q0=json.loads((root/'docs/performance/step5_rolling_20261107/fixture.json').read_text())['q0']
w,h=meta['render_width'],meta['render_height']
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('binary',type=Path)
parser.add_argument('output',type=Path)
parser.add_argument('--race',action='store_true',help='exercise real receive callbacks during coordinator computation')
args=parser.parse_args()
args.output.mkdir(exist_ok=False)
rclpy.init()
node=rclpy.create_node('p3_epoch_replay')

def wait_until(predicate):
    deadline=time.monotonic()+7
    while not predicate():
        if time.monotonic()>deadline:
            raise RuntimeError("DDS discovery timeout")
        rclpy.spin_once(node,timeout_sec=.02)

class Source:
    def __init__(self, session, generation):
        self.prefix=f"/ego4p3/s_{session}/e{generation}" if session else ""
        def create(typ,topic):
            return node.create_publisher(typ,self.prefix+topic,qos)
        self.clock_pub=create(Clock,'/clock')
        self.odom=create(Odometry,'/sim/odom')
        self.intent=create(TwistStamped,'/human_intent')
        self.images=[]
        self.infos=[]
        for suffix in ('','_left','_back','_right'):
            self.images.append(create(Image,f'/sim/depth{suffix}/image_raw'))
            self.infos.append(create(CameraInfo,f'/sim/depth{suffix}/camera_info'))

    def clock(self, ns):
        msg=Clock()
        msg.clock.sec,msg.clock.nanosec=divmod(ns,10**9)
        self.clock_pub.publish(msg)

    def close(self):
        for p in [self.clock_pub,self.odom,self.intent,*self.images,*self.infos]:
            node.destroy_publisher(p)

    def wait(self):
        wait_until(lambda: all(p.get_subscription_count() for p in
                               [self.clock_pub,self.odom,self.intent,*self.images,*self.infos]))

    def send(self, ns, cameras=range(4), kin=True):
        sec,nsec=divmod(ns,10**9)
        odom=Odometry()
        odom.header.stamp.sec,odom.header.stamp.nanosec=sec,nsec
        odom.header.frame_id='world'
        odom.child_frame_id='base_link'
        odom.pose.pose.orientation.w=1.
        v=odom.twist.twist.linear
        v.x,v.y,v.z=(12*x for x in q0)
        desired=TwistStamped()
        desired.header=odom.header
        desired.twist.linear.x,desired.twist.linear.y=11.2763114494309,4.104241719908
        if kin:
            self.odom.publish(odom)
            self.intent.publish(desired)
        for i in cameras:
            cam=meta['cameras'][i]
            info=CameraInfo()
            info.header.stamp=odom.header.stamp
            info.header.frame_id=cam['frame_id']
            info.width,info.height=w,h
            info.distortion_model='plumb_bob'
            info.d=[0.]*5
            info.k=[cam['fx'],0.,cam['cx'],0.,cam['fy'],cam['cy'],0.,0.,1.]
            self.infos[i].publish(info)
            im=Image()
            im.header=info.header
            im.width,im.height,im.step=w,h,4*w
            im.encoding='32FC1'
            im.data=raw[i*w*h*4:(i+1)*w*h*4]
            self.images[i].publish(im)

def exercise(mode):
    wait_until(lambda: not any(n=='ego4p2_navigation_input' for n,_ in node.get_node_names_and_namespaces()))
    legacy=mode=='legacy'
    session='' if legacy else uuid.uuid4().hex
    old,new,foreign=Source(session,0),Source(session,1),Source(uuid.uuid4().hex,1)
    out=args.output/mode
    cmd=[str(args.binary),'--calibration',str(archive/'simulation.sensors.json'),'--output',str(out),
         '--wall-timeout','4' if legacy else '12','--correctness-normal','--expect-groups','4']
    if session:cmd+=['--source-session',session]
    if mode in ('fixed','rolling'):cmd+=['--track-'+mode]
    with (args.output/(mode+'.log')).open('w') as log:
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        try:
            old.wait()
            assert all(not node.count_publishers(t) for t in ['/sim/cmd_vel','/position_cmd'])
            wait_until(lambda: sum(n=='ego4p2_navigation_input' for n,_ in node.get_node_names_and_namespaces())==1)
            navigation=[(n,ns) for n,ns in node.get_node_names_and_namespaces() if n=='ego4p2_navigation_input']
            assert len(navigation)==1
            advertised=node.get_publisher_names_and_types_by_node(*navigation[0])
            assert set(t for t,_ in advertised)<=set(['/rosout','/parameter_events']),advertised
            old.clock(500000000);time.sleep(.08);old.send(500000000);time.sleep(.12)
            if legacy:
                old.clock(5000000);time.sleep(.06);old.send(5000000)
            else:
                wait_until(lambda:new.clock_pub.get_subscription_count()>0)
                new.clock(5000000);new.wait();time.sleep(.05)
                new.send(5000000,range(3));old.send(5000000);foreign.send(5000000)
                old.clock(500000000);foreign.clock(500000000)
                new.clock(6000000)
                # Three live cameras and a fourth from either old source cannot form a group.
                time.sleep(.09)
                new.send(5000000,[3],False);time.sleep(.12)
                new.send(5000000);new.clock(7000000);time.sleep(.10)
                for t in (10000000,15000000):
                    new.clock(t);time.sleep(.04);new.send(t);time.sleep(.12)
            code=child.wait(timeout=14)
        finally:
            if child.poll() is None:child.kill();child.wait()
    summary=json.loads((out/'summary.json').read_text())
    rows=list(csv.DictReader((out/'navigation_events.csv').open()))
    assert summary['epoch_resets']==1 and summary['motion_publishers']==0,summary
    assert any(r['epoch']=='0' and r['angular_ready']=='1' for r in rows)
    if legacy:
        assert code==3 and summary['source_quarantined'] and summary['complete_depth_groups']==1,summary
        assert all(r['angular_ready']=='0' for r in rows if r['epoch']=='1')
    else:
        assert code==0 and summary['complete_depth_groups']==4 and not summary['source_quarantined'],summary
        assert all(r['invalid']!='group_regression' for r in rows)
        assert all(int(r['sim_ns'])<500000000 for r in rows if r['epoch']=='1')
        assert any(r['epoch']=='1' and r['group']=='2' and r['angular_ready']=='1' and r['sim_ns']=='6000000' for r in rows)
        if mode!='rolling':
            assert any(r['epoch']=='1' and r['sim_ns']=='6000000' and r['group']=='0' for r in rows)
            assert any(r['epoch']=='1' and r['sim_ns']=='7000000' and r['group']=='2' and r['angular_ready']=='1' for r in rows)
    return dict(mode=mode,exit=code,advertised=advertised,**summary)

def concurrent_rewinds():
    session=uuid.uuid4().hex
    out=args.output/'race'
    cmd=[str(args.binary),'--calibration',str(archive/'simulation.sensors.json'),'--output',str(out),
         '--source-session',session,'--duration-sim','.025','--wall-timeout','90','--correctness-normal']
    with (args.output/'race.log').open('w') as log:
        child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
        try:
            active=Source(session,0)
            for generation in range(25):
                wait_until(lambda:active.clock_pub.get_subscription_count()>0)
                stamp=500000000 if generation==0 else 5000000
                active.clock(stamp);active.wait();time.sleep(.02);active.send(stamp)
                time.sleep(.10)  # allow current input and buffer diagnostic rows
                following=Source(session,generation+1)
                wait_until(lambda:following.clock_pub.get_subscription_count()>0)
                lines=(out/'navigation_events.csv').read_text().splitlines()
                rows=[r for r in csv.DictReader(lines[:-1]) if r.get('scheduled_ns')]
                if rows:
                    phase=int(rows[-1]['scheduled_ns'])
                    deadline=phase+((time.monotonic_ns()-phase)//5000000+2)*5000000+100000
                    while time.monotonic_ns()<deadline:pass
                following.clock(5000000)
                # Callback/compute overlap is an observation, never inferred from the requested phase.
                time.sleep(.03)
                active.close()
                active=following
            following.wait()
            for stamp in range(5000000,40000001,5000000):
                following.clock(stamp);time.sleep(.015);following.send(stamp);time.sleep(.035)
            assert child.wait(timeout=8)==0
        finally:
            if child.poll() is None:child.kill();child.wait()
    with (out/'epoch_events.csv').open() as stream:events=list(csv.DictReader(stream))
    drops=[r for r in events if r['event']=='discard' and int(r['commit_epoch'])>int(r['captured_epoch'])
           and int(r['group'])>0]
    assert drops, "no observed overlap in the fixed replay window"
    summary=json.loads((out/'summary.json').read_text())
    assert summary['epoch_resets']==25 and summary['discarded_results']>=len(drops)
    return dict(mode='race',observed_cross_epoch_discards=len(drops),**summary)

try:
    results=[]
    if args.race:
        results.append(concurrent_rewinds())
    for mode in (() if args.race else ('default','fixed','rolling','legacy')):
        result=exercise(mode)
        results.append(result)
        print(mode,result,flush=True)
    (args.output/'assertions.json').write_text(json.dumps(results,indent=2)+'\n')
finally:
    node.destroy_node()
    rclpy.shutdown()
