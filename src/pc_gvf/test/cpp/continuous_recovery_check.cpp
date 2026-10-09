#include "pc_gvf/paper_guidance.hpp"
#include <cstdlib>
#include <iostream>
#include <sstream>
using namespace pc_gvf::depth_angular;
void check(bool ok,const char* why){if(!ok){std::cerr<<why<<'\n';std::exit(1);}}
void replayMatches(PaperGuidance& planner,const DepthObservation& o,const Eigen::Vector3d& p,
    const Eigen::Vector3d& v,const Eigen::Vector3d& q,double t) {
    std::stringstream snapshot(std::ios::in|std::ios::out|std::ios::binary);
    planner.saveReplay(snapshot,o,p,v,q,t,.02);
    const auto expected=planner.step(o,p,v,q,t,.02);
    snapshot.seekg(0);const auto actual=PaperGuidance::replay(snapshot);
    check(expected.status==actual.status && expected.build_reason==actual.build_reason &&
        expected.accepted==actual.accepted && (expected.command-actual.command).norm()<1e-11 &&
        std::abs(expected.best_prefix-actual.best_prefix)<1e-11 &&
        expected.free_directions==actual.free_directions,"state replay changed the decision");
}
int main() {
    Camera camera(48,36,90,68,10);
    PaperConfig cfg;cfg.retain_verified_travel=true;cfg.history_duration=60;
    cfg.restart_clearance=0;
    cfg.adaptive_lookahead=true;cfg.adaptive_grid=true;cfg.minimum_lookahead=.1;
    cfg.motion.delay=.2;cfg.motion.rollout_horizon=.2;cfg.motion.max_accel=1.2;cfg.motion.brake_accel=1.2;
    PaperGuidance corridor(cfg);const double radius=corridor.envelopeRadius();
    // A tight planar wall leaves only 12 mm extra along X, but a long Y
    // corridor. A midpoint sphere expanded by 5 cm falsely rejects it.
    DepthObservation seen(camera,std::vector<double>(48*36,3+radius+.012-.22),
        Eigen::Vector3d(.22,0,.02),fixedCameraRotation(),1,1);
    for(int i=0;i<=30;++i)check(corridor.rememberVerifiedPosition(seen,Eigen::Vector3d(3,.05*i,0),1+.002*i),
        "measured tight corridor recording failed");
    check(corridor.recordedTubes()>0,"continuous evidence missing");
    auto blind=seen;blind.depth.assign(48*36,0);blind.stamp=70;blind.version++;
    check(corridor.directionalClearance(blind,Eigen::Vector3d(3,1.5,0),-Eigen::Vector3d::UnitY(),70,1.4)>1.3,
        "proved return corridor lost when depth is absent");
    check(corridor.directionalClearance(blind,Eigen::Vector3d(3,1.5,0),Eigen::Vector3d::UnitX(),70,.2)<.02,
        "continuous evidence extended through unknown wall side");
    replayMatches(corridor,blind,Eigen::Vector3d(3,1.2,0),Eigen::Vector3d::Zero(),-Eigen::Vector3d::UnitY(),70);
    auto intrusion=blind;intrusion.stamp=71;intrusion.version++;
    intrusion.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,
        intrusion.rotation.transpose()*(Eigen::Vector3d(3,.6,0)-intrusion.origin));
    corridor.ingestObservation(intrusion);
    check(corridor.revokedTubes()>0,"native obstacle failed to revoke corridor");
    check(!corridor.envelopeKnown(intrusion,Eigen::Vector3d(3,.6,0),radius,71),"revoked corridor still authorizes obstacle");
    // Individual endpoint observations do not authorize an unobserved bridge.
    PaperGuidance gap(cfg);
    for(const auto& p:{Eigen::Vector3d(3,0,0),Eigen::Vector3d(5,0,0)}) {
        auto local=seen;local.origin=p-Eigen::Vector3d(2,0,0);local.depth.assign(48*36,2+radius+.012);
        check(gap.rememberVerifiedPosition(local,p,1),"gap endpoint fixture failed");
    }
    check(gap.recordedTubes()==0&&!gap.envelopeKnown(blind,Eigen::Vector3d(4,0,0),radius,70),"unobserved endpoint gap joined");
    PaperGuidance short_start(cfg);short_start.setVerifiedSeed(Eigen::Vector3d::Zero(),radius+.035);
    auto no_view=blind;no_view.stamp=1;no_view.version=1;
    const auto moving=short_start.step(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1,.02);
    check(moving.accepted&&moving.command.norm()>0&&moving.command.norm()<.15,"short certified prefix could not start slowly");
    check(!short_start.motionSafe(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),Eigen::Vector3d::Zero(),1),
        "adaptive prefix waived measured-velocity braking");
    replayMatches(short_start,no_view,Eigen::Vector3d::Zero(),moving.command,Eigen::Vector3d::UnitX(),1.02);
    check(short_start.reachedProofs()>0,"arrived original motion certificate was compressed away");
    PaperConfig restart_test=cfg;restart_test.restart_clearance=.03;
    PaperGuidance no_restart(restart_test);no_restart.setVerifiedSeed(Eigen::Vector3d::Zero(),radius+.01);
    check(!no_restart.motionSafe(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),1),
        "terminal restart neighborhood was not checked");
    PaperConfig stopping_only=cfg;stopping_only.restart_clearance=0;
    PaperGuidance just_stop(stopping_only);just_stop.setVerifiedSeed(Eigen::Vector3d::Zero(),radius+.01);
    check(just_stop.motionSafe(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),1),
        "terminal fixture was not physically stoppable");
    PaperGuidance blocked(cfg);
    replayMatches(blocked,no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1);
    blocked.setVerifiedSeed(Eigen::Vector3d::Zero(),radius);
    check(!blocked.step(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1,.02).accepted,
        "body-only certificate expanded into motion space");
    // Static mode preserves real depth provenance through a long stop. A
    // moving-world configuration still expires it, and a fresh hit revokes it.
    PaperConfig memory_cfg=cfg;memory_cfg.history_uncertainty_rate=0.;memory_cfg.history_max_observations=16;
    PaperGuidance memory(memory_cfg);
    DepthObservation wide=seen;wide.depth.assign(48*36,10);memory.ingestObservation(wide);
    check(memory.rememberVerifiedPosition(wide,Eigen::Vector3d(3,0,0),1),"coverage witness setup failed");
    for(int i=0;i<40;++i) {
        auto far=wide;far.origin.x()=100+2*i;far.stamp=2+i;far.version=10+i;
        memory.ingestObservation(far);
    }
    check(memory.retainedObservations()==16,"coverage retention exceeded its bound");
    memory.ingestObservation(blind);
    const auto resumed=memory.step(blind,Eigen::Vector3d(3,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),70,.02);
    check(resumed.accepted&&memory.observation()&&
        memory.envelopeKnown(*memory.observation(),Eigen::Vector3d(3,0,0),radius+.2,70),
        "unique near-body depth support was evicted or aged out in static mode");
    auto changed=intrusion;changed.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,
        changed.rotation.transpose()*(Eigen::Vector3d(3,0,0)-changed.origin));
    memory.ingestObservation(changed);
    check(!memory.envelopeKnown(changed,Eigen::Vector3d(3,0,0),radius,71),"new obstacle did not revoke old static depth");
    memory_cfg.retain_verified_travel=false;PaperGuidance expiring(memory_cfg);
    expiring.ingestObservation(wide);expiring.ingestObservation(blind);
    check(expiring.step(blind,Eigen::Vector3d(3,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),70,.02).status==
        "UNOBSERVED_BODY_ENVELOPE","nonstatic observations failed to expire");
    // A proved free volume is not a predicted vehicle pose. In static mode
    // the accepted braking corridor must survive a sensor handoff before the
    // vehicle reaches it, but cannot grow into unobserved space or ignore hits.
    auto volume_cfg=cfg;volume_cfg.history_uncertainty_rate=0.;volume_cfg.retain_certified_volume=true;
    volume_cfg.intent_guard=true;volume_cfg.smooth_speed=true;
    auto source=seen;source.depth.assign(48*36,10);source.stamp=1;
    const Eigen::Vector3d location(3,0,0),travel(1,0,0);
    PaperGuidance volume(volume_cfg);
    const auto accepted=volume.stepWithProposal(source,location,travel,travel,travel,1,.02);
    check(accepted.accepted,"volume fixture failed to certify motion");
    auto hidden=source;hidden.depth.assign(48*36,0);hidden.stamp=1.02;hidden.version++;
    check(volume.motionSafe(hidden,location,travel,Eigen::Vector3d::Zero(),1.02),
        "accepted braking volume lost before physical arrival");
    check(!volume.envelopeKnown(hidden,location+Eigen::Vector3d(2,0,0),radius,1.02),
        "unexecuted proposal fabricated farther free space");
    for(int i=0;i<8;++i) {
        hidden.stamp=1.02+.02*i;
        volume.stepWithProposal(hidden,location,travel,travel,travel,hidden.stamp,.02);
        check(!volume.envelopeKnown(hidden,location+Eigen::Vector3d(2,0,0),radius,hidden.stamp),
            "successive certificates expanded into unknown volume");
    }
    replayMatches(volume,hidden,location,travel,travel,hidden.stamp);
    auto hit=hidden;hit.stamp=2;hit.version++;
    hit.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,
        hit.rotation.transpose()*(location+Eigen::Vector3d(.3,0,0)-hit.origin));
    volume.ingestObservation(hit);
    check(!volume.envelopeKnown(hit,location+Eigen::Vector3d(.3,0,0),0.,2),
        "fresh native hit failed to revoke accepted future volume");
    volume_cfg.retain_certified_volume=false;PaperGuidance legacy(volume_cfg);
    check(legacy.stepWithProposal(source,location,travel,travel,travel,1,.02).accepted,
        "legacy fixture failed");
    check(!legacy.motionSafe(hidden,location,travel,Eigen::Vector3d::Zero(),hidden.stamp),
        "legacy mode unexpectedly enabled future volume memory");
    auto cadence_cfg=memory_cfg;cadence_cfg.retain_verified_travel=true;
    cadence_cfg.history_sample_interval=1.;cadence_cfg.history_max_observations=4;
    PaperGuidance cadence(cadence_cfg);
    for(int frame=0;frame<250;++frame)for(int view=0;view<4;++view) {
        auto current=wide;current.stamp=1.+.02*frame;current.view=view;current.version=frame+1;
        current.origin=Eigen::Vector3d(100.+frame,100.*view,0.);
        cadence.ingestObservation(current);
    }
    check(cadence.retainedObservations()<=4&&cadence.evictedViews()<=16,
        "coverage eviction bypassed one-second sampling cadence");
    bool rejected=false;try{std::stringstream broken("EGOPAPR3");PaperGuidance::replay(broken);}catch(const std::exception&){rejected=true;}
    check(rejected,"truncated replay accepted");
    std::cout<<"continuous return, unknown gap/wall, revocation, short-prefix braking, deterministic replay: PASS\n";
}
