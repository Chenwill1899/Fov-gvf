#include "pc_gvf/paper_guidance.hpp"
#include "pc_gvf/depth_uncertainty.hpp"
#include "pc_gvf/operator_intent.hpp"
#include <sstream>
#include <iostream>
using namespace pc_gvf::depth_angular;
int main(){
    Camera c(32,24,90,68,10.);int count=0;
    for(int mask=0;mask<64;++mask) {
        PaperConfig cfg;cfg.dynamic_obstacles=mask&1;cfg.incremental_field=mask&2;cfg.spherical_memory=mask&4;cfg.shared_obstacles=mask&8;
        cfg.certified_direct=true;cfg.history_duration=3.;cfg.motion.body_radius=.25;cfg.motion.safety_margin=.02;
        PaperGuidance planner(cfg);DepthObservation o(c,std::vector<double>(768,10.),Eigen::Vector3d(-2,0,0),fixedCameraRotation(),1.,1);
        DepthUncertaintyConfig uncertainty;uncertainty.enabled=mask&16;
        o=conservativeObservation(o,uncertainty);
        planner.ingestObservation(o);
        SharedObstaclePacket packet;packet.source="peer";packet.session="test";packet.frame="world";packet.stamp=1.;packet.sequence=1;
        ObstacleTrack track;track.point={1,0,0};track.stamp=1.;track.observations=3;packet.tracks.push_back(track);
        planner.ingestShared(packet,1.,"world");
        const Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero();
        OperatorIntent filter;filter.update(Eigen::Vector3d::UnitX(),mask&32);
        const Eigen::Vector3d q=filter.update(Eigen::Vector3d(.5,.002,0),mask&32);
        std::stringstream buffer(std::ios::in|std::ios::out|std::ios::binary);planner.saveReplay(buffer,o,p,v,q,1.,.02,&q);
        const auto direct=planner.stepWithProposal(o,p,v,q,q,1.,.02);buffer.seekg(0);const auto replay=PaperGuidance::replay(buffer);
        if(direct.accepted!=replay.accepted||direct.status!=replay.status||(direct.command-replay.command).norm()>1e-10)return 1;
        ++count;
    }
    // A valid ninth peer cannot silently disappear when the bounded shared
    // map is full. Its temporary capacity stop must survive serialization.
    PaperConfig cfg;cfg.shared_obstacles=true;cfg.certified_direct=true;
    cfg.motion.body_radius=.25;cfg.motion.safety_margin=.02;
    PaperGuidance planner(cfg);
    DepthObservation o(c,std::vector<double>(768,10.),Eigen::Vector3d(-2,0,0),fixedCameraRotation(),2.,2);
    planner.ingestObservation(o);
    SharedObstaclePacket packet;packet.session="capacity";packet.frame="world";packet.stamp=2.;packet.sequence=1;
    for(int i=0;i<8;++i){packet.source="peer"+std::to_string(i);if(!planner.ingestShared(packet,2.,"world"))return 2;}
    packet.source="ninth";ObstacleTrack hazard;hazard.point={.5,0,0};hazard.stamp=2.;hazard.observations=3;
    packet.tracks.push_back(hazard);if(planner.ingestShared(packet,2.,"world"))return 3;
    const Eigen::Vector3d p=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero(),q(.5,0,0);
    if(planner.motionSafe(o,p,v,q,2.))return 4;
    std::stringstream buffer(std::ios::in|std::ios::out|std::ios::binary);planner.saveReplay(buffer,o,p,v,q,2.,.02,&q);
    const auto direct=planner.stepWithProposal(o,p,v,q,q,2.,.02);buffer.seekg(0);const auto replay=PaperGuidance::replay(buffer);
    if(direct.accepted||replay.accepted||direct.status!=replay.status||(direct.command-replay.command).norm()>1e-10)return 5;
    std::cout<<"feature_combinations,replay_equal\n64,"<<count<<'\n';
    std::cout<<"capacity_guard_replay_equal\n1\n";return 0;
}
