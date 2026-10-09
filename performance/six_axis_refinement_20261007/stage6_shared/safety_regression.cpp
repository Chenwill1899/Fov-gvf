#include "pc_gvf/shared_obstacles.hpp"
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
    std::cout<<"temporal_edge_queries=63 random_queries=2000 mismatches=0\n";
    return 0;
}
