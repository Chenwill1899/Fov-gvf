#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <iostream>
using namespace pc_gvf::depth_angular;
int main() {
    int before=0,after=0;Camera c(48,36,70,60,10.);
    for(int k=0;k<74;++k) {
        const double pitch=k<72?(k/24-1)*std::acos(-1.)/3.:(k==72?1.:-1.)*std::acos(-1.)/2.;
        const Eigen::Matrix3d rot=Eigen::AngleAxisd((k%24)*2*std::acos(-1.)/24,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
            Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()).toRotationMatrix();
        const Eigen::Vector3d q=rot*Eigen::Vector3d::UnitX(),p=3.*q;
        PaperConfig cfg;cfg.coarse_fine=false;cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;
        cfg.observation_timeout=.3;cfg.history_duration=60.;cfg.history_sample_interval=1.;cfg.history_max_observations=16;cfg.history_uncertainty_rate=.05;cfg.certified_direct=true;
        cfg.motion.rollout_margin=.02;cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.1;cfg.observation_timeout=.3;cfg.motion.max_accel=1.2;cfg.motion.brake_accel=1.2;cfg.motion.delay=.2;cfg.motion.rollout_horizon=.2;
        PaperGuidance baseline(cfg);cfg.spherical_memory=true;PaperGuidance optimized(cfg);
        DepthObservation initial(c,std::vector<double>(1728,10.),Eigen::Vector3d::Zero(),rot*Eigen::AngleAxisd(std::acos(-1.)/2,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation(),0.,1);
        baseline.ingestObservation(initial);optimized.ingestObservation(initial);
        DepthObservation old(c,std::vector<double>(1728,10.),Eigen::Vector3d::Zero(),rot*fixedCameraRotation(),.2,2);
        baseline.ingestObservation(old);optimized.ingestObservation(old);
        DepthObservation current(c,std::vector<double>(1728,10.),p,
            rot*Eigen::AngleAxisd(std::acos(-1.),Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation(),.4,3);
        baseline.ingestObservation(current);optimized.ingestObservation(current);
        before+=baseline.stepWithProposal(current,p,Eigen::Vector3d::Zero(),q,q,.4,.02).accepted;
        after+=optimized.stepWithProposal(current,p,Eigen::Vector3d::Zero(),q,q,.4,.02).accepted;
        current.stamp=6.;current.version=4;optimized.ingestObservation(current);
        if(optimized.stepWithProposal(current,p,Eigen::Vector3d::Zero(),q,q,6.,.02).accepted)return 2;
    }
    SphericalMemory cache;DepthObservation old(c,std::vector<double>(1728,10.),Eigen::Vector3d::Zero(),fixedCameraRotation(),0.,1);
    cache.ingest(old);auto borrowed=cache.evidence(0.).front();auto hit=old;hit.stamp=.1;hit.version=2;hit.depth.assign(1728,0.);
    hit.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,hit.rotation.transpose()*Eigen::Vector3d(4,0,0));cache.ingest(hit);
    if(!borrowed->revoked)return 3;
    // Enabling short-lived spherical evidence must not retroactively age the
    // already verified static history. This reproduced the first Cloud stall.
    PaperConfig static_cfg;static_cfg.spherical_memory=true;static_cfg.history_duration=60.;
    static_cfg.history_uncertainty_rate=0.;static_cfg.retain_verified_travel=true;
    static_cfg.motion.body_radius=.48;static_cfg.motion.safety_margin=.02;static_cfg.motion.rollout_margin=.02;
    static_cfg.depth_uncertainty=.03;static_cfg.uncertainty_rate=.1;static_cfg.observation_timeout=.3;
    PaperGuidance static_planner(static_cfg);
    DepthObservation static_old(c,std::vector<double>(1728,2.),Eigen::Vector3d::Zero(),fixedCameraRotation(),0.,1);
    static_planner.ingestObservation(static_old);
    auto fresh=static_old;fresh.depth.assign(1728,0.);fresh.stamp=10.;fresh.version=2;static_planner.ingestObservation(fresh);
    auto current=fresh;current.supporting_views=static_planner.retainedDepthObservations();
    if(!static_planner.envelopeKnown(current,Eigen::Vector3d(1.3,0,0),.58,10.))return 4;
    std::cout<<"blind_direction_cases,sparse_history_accepted,spherical_memory_accepted,expired_accepted,contradiction_revoked\n74,"
        <<before<<','<<after<<",0,1\n";
    return before==0&&after==74?0:1;
}
