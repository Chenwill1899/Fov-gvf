#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <random>
using namespace pc_gvf::depth_angular;
int main() {
    // Differential broad-phase checks include Unix-size timestamps and exact
    // capsule tangencies: a broad-phase shortcut may only return a true safe.
    for(double epoch:{0.,1e9,2e9})for(double horizon:{.04,.10000005,.33,.9}) {
        SharedObstacleMap edge;SharedObstaclePacket p;p.source="edge";p.session="boot";p.frame="world";p.stamp=epoch;p.sequence=1;
        ObstacleTrack t;t.point.setZero();t.velocity.setZero();t.radius=0.;t.stamp=epoch;t.observations=1;p.tracks.push_back(t);
        if(!edge.merge(p,epoch,"world"))return 17;
        const double boundary=.58+.03+3.*horizon+.25*horizon*horizon;
        for(double offset:{-1e-7,-1e-9,0.,1e-9,1e-7}) {
            const Eigen::Vector3d a(boundary+offset,0.,0.);
            if(edge.segmentSafe(a,a,.58,epoch,0.,horizon)!=obstacleSegmentsSafe(p.tracks,false,a,a,.58,epoch,0.,horizon))return 18;
        }
    }
    for(double epoch:{1.,1e6,1e9}) {
        SharedObstacleMap future;SharedObstaclePacket p;p.source="future";p.session="boot";p.frame="world";p.stamp=epoch;p.sequence=1;
        ObstacleTrack t;t.point=Eigen::Vector3d(100.,100.,100.);t.stamp=epoch+.02;p.tracks.push_back(t);
        if(!future.merge(p,epoch,"world"))return 21;
        const auto a=Eigen::Vector3d::Zero().eval();
        if(future.segmentSafe(a,a,.58,epoch,0.,.1)!=obstacleSegmentsSafe(p.tracks,false,a,a,.58,epoch,0.,.1))return 22;
    }
    std::mt19937 random(912);std::uniform_real_distribution<double> position(-20.,20.),clock_age(0.,.49),horizon(0.,2.);
    SharedObstacleMap broad;std::vector<SharedObstaclePacket> packets;
    for(int robot=0;robot<8;++robot) {
        SharedObstaclePacket p;p.source="source_"+std::to_string(robot);p.session="boot";p.frame="world";p.stamp=1.;p.sequence=1;
        for(int i=0;i<100;++i){ObstacleTrack t;t.point=Eigen::Vector3d(position(random),position(random),position(random));t.velocity=Eigen::Vector3d(.1*position(random),.1*position(random),0.);t.stamp=1.-.001*(i%40);t.observations=i%3?2:1;p.tracks.push_back(t);}
        if(!broad.merge(p,1.,"world"))return 19;
        packets.push_back(p);
    }
    for(int i=0;i<2000;++i) {
        const Eigen::Vector3d a(position(random),position(random),position(random)),b(position(random),position(random),position(random));
        const double now=1.+clock_age(random),end=horizon(random);bool exact=true;
        for(const auto& p:packets)exact=exact&&obstacleSegmentsSafe(p.tracks,p.overloaded,a,b,.58,now,0.,end);
        if(exact!=broad.segmentSafe(a,b,.58,now,0.,end))return 20;
    }
    int before=0,after=0;Camera c(32,24,90,68,10.);
    for(int i=0;i<32;++i) {
        PaperConfig cfg;cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;cfg.motion.rollout_horizon=.8;
        cfg.motion.rollout_margin=.02;cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.1;cfg.observation_timeout=.3;cfg.motion.max_accel=1.2;cfg.motion.brake_accel=1.2;cfg.motion.delay=.2;cfg.motion.rollout_horizon=.2;
        PaperGuidance baseline(cfg);cfg.shared_obstacles=true;PaperGuidance receiver(cfg);
        DepthObservation local(c,std::vector<double>(768,10.),Eigen::Vector3d(-2,0,0),fixedCameraRotation(),1.,1);
        SharedObstaclePacket peer;peer.source="robot_b";peer.session="boot_1";peer.frame="world";peer.stamp=1.;peer.sequence=1;
        ObstacleTrack hit;hit.point=Eigen::Vector3d(.6+.01*i,0,0);hit.stamp=1.;hit.radius=.2;hit.observations=3;peer.tracks.push_back(hit);
        if(!receiver.ingestShared(peer,1.,"world"))return 2;
        before+=baseline.motionSafe(local,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),1.);
        after+=receiver.motionSafe(local,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),1.);
        if(receiver.ingestShared(peer,1.,"world"))return 3; // echo is not independent evidence
        peer.sequence=2;peer.frame="unaligned_map";if(receiver.ingestShared(peer,1.,"world"))return 4;
        peer.frame="world";peer.stamp=.1;if(receiver.ingestShared(peer,1.,"world"))return 5;
        peer.stamp=2.;if(receiver.ingestShared(peer,1.,"world"))return 6;
        peer.stamp=1.1;peer.sequence=3;peer.tracks[0].stamp=1.1;peer.tracks[0].point=Eigen::Vector3d(4,4,0);
        if(!receiver.ingestShared(peer,1.1,"world"))return 8;
        local.stamp=1.1;if(!receiver.motionSafe(local,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0,0),1.1))return 9;
        // A remote report cannot authorize motion through local unknown space.
        local.depth.assign(768,0.);if(receiver.motionSafe(local,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitY(),1.1))return 7;
    }
    SharedObstacleMap rotating_peers;
    SharedObstaclePacket packet;packet.session="boot";packet.frame="world";packet.stamp=1.;packet.sequence=1;
    for(int i=0;i<8;++i){packet.source="peer_"+std::to_string(i);if(!rotating_peers.merge(packet,1.,"world"))return 10;}
    packet.source="ninth";ObstacleTrack excess;excess.point=Eigen::Vector3d(.5,0,0);excess.stamp=1.;packet.tracks.push_back(excess);
    if(rotating_peers.merge(packet,1.,"world"))return 11;
    if(rotating_peers.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.,0.,.3))return 13;
    if(!rotating_peers.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.501,0.,.3))return 14;
    // A rejected invalid ninth packet cannot disable an otherwise usable map.
    SharedObstacleMap invalid_capacity;packet.tracks.clear();
    for(int i=0;i<8;++i){packet.source="peer_"+std::to_string(i);if(!invalid_capacity.merge(packet,1.,"world"))return 15;}
    packet.source="invalid_ninth";packet.frame="unaligned_map";
    if(invalid_capacity.merge(packet,1.,"world")||!invalid_capacity.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.,0.,.3))return 16;
    packet.frame="world";packet.source="ninth";
    packet.stamp=1.6;if(!rotating_peers.merge(packet,1.6,"world")||rotating_peers.sources().size()!=1)return 12;
    std::cout<<"peer_only_hazards,independent_map_accepted,shared_map_accepted,invalid_packets_accepted,remote_free_authorizations\n32,"
        <<before<<','<<after<<",0,0\n";
    return before==32&&after==0?0:1;
}
