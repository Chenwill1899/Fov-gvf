#include "pc_gvf/paper_guidance.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
using namespace pc_gvf::depth_angular;
namespace {
void check(bool condition,const char* message) {
    if(!condition){std::cerr<<message<<'\n';std::exit(1);}
}
std::string snapshot(PaperGuidance& planner,const DepthObservation& view,
    const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& command) {
    std::ostringstream out(std::ios::binary);
    planner.saveReplay(out,view,position,velocity,command,view.stamp,.02,&command);
    return out.str();
}
PaperResult replay(const std::string& bytes,std::ostream* audit=nullptr) {
    std::istringstream in(bytes,std::ios::binary);
    return PaperGuidance::replay(in,audit);
}
}
int main() {
    PaperConfig cfg;cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;
    cfg.motion.rollout_margin=.02;cfg.observation_timeout=.3;
    cfg.motion.max_accel=16.;cfg.motion.brake_accel=16.;cfg.motion.velocity_tau=.10;
    cfg.motion.delay=0.;cfg.motion.rollout_horizon=.2;
    cfg.retain_verified_travel=true;cfg.retain_certified_volume=true;
    cfg.history_uncertainty_rate=0.;cfg.free_space_floor_enabled=true;
    cfg.free_space_floor_z=0.;cfg.max_speed=4.;cfg.max_vertical_speed=4.;
    Camera camera(48,36,90,68,10.);
    DepthObservation blind(camera,std::vector<double>(48*36,0.),Eigen::Vector3d(-3,0,.3),
        fixedCameraRotation(),1.,1);
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero(),x=Eigen::Vector3d::UnitX();
    PaperGuidance floor(cfg);const double radius=floor.envelopeRadius();
    // A supplied world seed intentionally straddles the plane. It is still
    // free-space evidence only inside the configured half-space.
    check(floor.setVerifiedSeed(zero,10.),"seed setup failed");
    const Eigen::Vector3d boundary(0,0,radius),below(0,0,radius-1e-7),above(0,0,radius+.2);
    check(floor.envelopeKnown(blind,boundary,radius,1.),"exact floor tangency rejected");
    check(!floor.envelopeKnown(blind,below,radius,1.),"seed bypassed floor sphere check");
    check(floor.motionSafe(blind,boundary,zero,zero,1.),"stationary boundary body rejected");
    check(!floor.motionSafe(blind,below,zero,zero,1.),"stationary body below floor accepted");
    check(!floor.rememberVerifiedPosition(blind,below,1.),"measured footprint bypassed floor");
    check(floor.proposalPath(blind,{above,above+x},1.),"safe observed path rejected");
    check(!floor.proposalPath(blind,{below,below+x},1.),"seed segment shortcut bypassed floor");
    check(!floor.proposalPath(blind,{above,below},1.),"crossing segment accepted");
    check(floor.directionalClearance(blind,above,-Eigen::Vector3d::UnitZ(),1.,1.)<=.200001,
        "candidate clearance crossed the floor");
    check(floor.motionSafe(blind,above,zero,x,1.),"safe moving certificate rejected");
    // Both endpoints are above the plane, but the already charged chord
    // bulge extends below it. The floor applies to the full query radius.
    check(!floor.motionSafe(blind,Eigen::Vector3d(0,0,radius+.001),zero,4.*x,1.),
        "motion acceleration bulge bypassed floor");
    check(!floor.motionSafe(blind,Eigen::Vector3d(0,0,radius+.14),zero,-Eigen::Vector3d::UnitZ(),1.),
        "downward braking tail crossed floor");
    auto no_tail_cfg=cfg;no_tail_cfg.motion.rollout_horizon=0.;PaperGuidance no_tail(no_tail_cfg);
    no_tail.setVerifiedSeed(zero,10.);
    check(no_tail.motionSafe(blind,Eigen::Vector3d(0,0,radius+.14),zero,-Eigen::Vector3d::UnitZ(),1.),
        "braking-tail fixture had invalid initial body");
    // A command that reverses a descent may be safe while the independent
    // zero-command E trajectory is not. E must retain the same plane.
    const Eigen::Vector3d descending(0,0,-1.),up=Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d reverse_position(0,0,radius+.05);
    check(floor.motionSafe(blind,reverse_position,descending,up,1.),"safe reversal fixture failed");
    auto emergency_cfg=cfg;emergency_cfg.emergency_braking_certificate=true;
    PaperGuidance emergency(emergency_cfg);emergency.setVerifiedSeed(zero,10.);
    check(!emergency.motionSafe(blind,reverse_position,descending,up,1.),"E path bypassed floor");
    PaperGuidance unknown(cfg);
    check(!unknown.envelopeKnown(blind,above,radius,1.)&&!unknown.motionSafe(blind,above,zero,x,1.),
        "floor constraint fabricated unknown free space");
    auto disabled_cfg=cfg;disabled_cfg.free_space_floor_enabled=false;
    PaperGuidance disabled(disabled_cfg);disabled.setVerifiedSeed(zero,10.);
    check(disabled.motionSafe(blind,below,zero,x,1.),"disabled floor changed legacy behavior");
    auto shifted_cfg=cfg;shifted_cfg.free_space_floor_z=-.2;
    PaperGuidance shifted(shifted_cfg);shifted.setVerifiedSeed(zero,10.);
    check(shifted.motionSafe(blind,below,zero,x,1.),"configured floor height was ignored");
    for(double invalid:{std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        auto invalid_cfg=cfg;invalid_cfg.free_space_floor_z=invalid;bool threw=false;
        try {PaperGuidance invalid_floor(invalid_cfg);}catch(const std::invalid_argument&){threw=true;}
        check(threw,"nonfinite floor accepted");
    }
    // Find the one explicit enabled byte from otherwise identical archives.
    // This lets us restore legacy stored geometry under a new half-space,
    // exercising capsules/balls rather than merely fresh depth paths.
    PaperGuidance empty_off(disabled_cfg),empty_on(cfg);
    auto off=snapshot(empty_off,blind,above,zero,x),on=snapshot(empty_on,blind,above,zero,x);
    check(off.substr(0,8)=="EGOPAPRQ"&&on.size()==off.size(),"Q replay format missing");
    std::size_t enabled_offset=off.size(),differences=0;
    for(std::size_t i=0;i<off.size();++i)if(off[i]!=on[i]){enabled_offset=i;++differences;}
    check(differences==1&&off[enabled_offset]==0&&on[enabled_offset]==1,"floor replay field was not isolated");
    DepthObservation seen=blind;seen.depth.assign(48*36,10.);
    PaperGuidance retained(disabled_cfg);const Eigen::Vector3d low(0,0,.3);
    check(retained.stepWithProposal(seen,low,x,x,x,1.,.02).accepted&&retained.recordedTubes()>0,
        "stored capsule fixture failed");
    blind.stamp=1.02;++blind.version;
    const auto saved=snapshot(retained,blind,low,x,x);
    check(replay(saved).accepted,"disabled stored-volume replay changed");
    auto restricted=saved;restricted[enabled_offset]=1;
    const auto denied=replay(restricted);
    check(!denied.accepted&&denied.command.isZero(),"stored capsule/ball replay bypassed floor");
    // Q stores the value too: a shifted floor changes this same state from
    // rejected to accepted without altering any geometry or observations.
    auto shifted_bytes=restricted;const double lower_floor=-1.;
    shifted_bytes.replace(enabled_offset+1,sizeof(double),reinterpret_cast<const char*>(&lower_floor),sizeof(double));
    check(replay(shifted_bytes).accepted,"Q replay lost configured floor value");
    auto legacy=saved;legacy.erase(enabled_offset,1+sizeof(double));legacy.replace(0,8,"EGOPAPRP");
    check(replay(legacy).accepted,"old P replay did not default floor to disabled");
    bool truncated_rejected=false;
    try {replay(saved.substr(0,enabled_offset+4));}catch(const std::exception&){truncated_rejected=true;}
    check(truncated_rejected,"truncated Q floor fields accepted");
    std::ostringstream audit;replay(restricted,&audit);
    check(audit.str().find("free_space_floor 1 0")!=std::string::npos,"replay audit omitted actual floor");
    std::cout<<"floor sphere/capsule shortcuts, boundary, full tail, E, unknown, disabled/value/Q-P replay: PASS\n";
}
