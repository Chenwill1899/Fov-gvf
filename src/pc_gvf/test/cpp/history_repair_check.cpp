#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace pc_gvf::depth_angular;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
DepthObservation evidence(const DepthObservation& current,const PaperGuidance& planner) {
    auto result=current;result.supporting_views=planner.retainedDepthObservations();return result;
}
}
int main() {
 try {
    PaperConfig cfg;cfg.local_history_repair=true;cfg.history_duration=60.;cfg.history_uncertainty_rate=0.;
    cfg.retain_verified_travel=true;cfg.history_sample_interval=1.;cfg.history_max_observations=4;
    cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;cfg.motion.rollout_margin=.02;
    cfg.depth_uncertainty=.01;cfg.uncertainty_rate=.1;cfg.observation_timeout=.5;
    Camera camera(64,48,90,68,10.);
    DepthObservation wide(camera,std::vector<double>(64*48,10.),Eigen::Vector3d::Zero(),fixedCameraRotation(),1.,1,0);
    const Eigen::Vector3d location(3.,0.,0.),hit(4.,2.,0.);
    PaperGuidance planner(cfg);planner.ingestObservation(wide);
    const auto original=planner.retainedDepthObservations().front();
    check(planner.envelopeKnown(evidence(wide,planner),location,planner.envelopeRadius()+.1,1.),"fixture does not cover body");
    auto fresh=wide;fresh.depth.assign(64*48,0.);fresh.stamp=1.1;fresh.version=2;fresh.view=1;
    fresh.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,fresh.rotation.transpose()*(hit-fresh.origin));
    planner.ingestObservation(fresh);
    check(original->revoked,"borrowed old keyframe was not invalidated");
    check(planner.retainedObservations()==2,"local conflict erased the entire static keyframe");
    const auto corrected=planner.retainedDepthObservations().front();
    check(!corrected->revoked&&corrected.get()!=original.get(),"repaired keyframe not immutable replacement");
    for(std::size_t i=0;i<wide.depth.size();++i)check(corrected->depth[i]<=wide.depth[i],"repair invented free depth");
    auto current=evidence(fresh,planner);
    check(planner.envelopeKnown(current,location,planner.envelopeRadius()+.1,1.1),"remote conflict erased valid near-body evidence");
    check(!planner.envelopeKnown(current,hit,0.,1.1),"fresh hit remains certified as free");
    check(!planner.motionSafe(current,location,Eigen::Vector3d::Zero(),(hit-location).normalized()*2.,1.1),"motion toward conflicting hit accepted");
    // The whole uncertainty footprint, including neighboring angular cells,
    // must be removed; a single central-pixel edit would fail this test.
    for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)
        check(!planner.envelopeKnown(current,hit+.004*Eigen::Vector3d(x,y,z),0.,1.1),"uncertain hit neighborhood remained free");
    // A cleared nearby corridor must produce an actual nonzero, fully
    // certified restart after release, not merely pass a point/surface test.
    const auto stopped=planner.stepWithProposal(fresh,location,Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),1.1,1./60.);
    check(!stopped.accepted&&stopped.command.norm()==0.,"release failed to stop");
    const Eigen::Vector3d forward(.5,0.,0.);
    const auto restart=planner.stepWithProposal(fresh,location,Eigen::Vector3d::Zero(),forward,forward,1.1,1./60.);
    check(restart.accepted&&restart.command.x()>.1&&restart.command.dot(forward)>0.,"local repair did not restore a certified restart");
    check(planner.motionSafe(current,location,Eigen::Vector3d::Zero(),restart.command,1.1),"restart skipped full motion proof");
    // The same conflict with the feature disabled reproduces complete loss
    // of the corridor; this guards against an ineffective synthetic fixture.
    auto legacy_cfg=cfg;legacy_cfg.local_history_repair=false;
    PaperGuidance legacy(legacy_cfg);legacy.ingestObservation(wide);legacy.ingestObservation(fresh);
    const auto blocked=legacy.stepWithProposal(fresh,location,Eigen::Vector3d::Zero(),forward,forward,1.1,1./60.);
    check(!blocked.accepted&&blocked.command.norm()==0.,"fixture does not reproduce legacy startup blockage");
    // Additional hits between keyframe retention times must keep carving.
    const Eigen::Vector3d next_hit(3.,0.,0.);fresh.stamp=1.2;fresh.version++;
    fresh.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,fresh.rotation.transpose()*next_hit);
    planner.ingestObservation(fresh);current=evidence(fresh,planner);
    check(!planner.envelopeKnown(current,next_hit,0.,1.2),"between-sample contradiction ignored");
    // A dynamic history is revoked as before, rather than locally repaired.
    cfg.history_uncertainty_rate=.1;PaperGuidance dynamic(cfg);dynamic.ingestObservation(wide);
    fresh.stamp=1.1;fresh.version=2;dynamic.ingestObservation(fresh);
    check(dynamic.retainedObservations()==1,"non-static contradictory history survived");
    check(!dynamic.envelopeKnown(evidence(fresh,dynamic),location,dynamic.envelopeRadius(),1.1),"non-static history fabricated free space");
    std::cout<<"history_repair_check: local carving, uncertainty footprint, immutable invalidation, certified restart versus legacy blockage, between-sample hits and dynamic revocation passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
