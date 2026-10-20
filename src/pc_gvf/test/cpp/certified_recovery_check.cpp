#include "pc_gvf/paper_guidance.hpp"
#include <iostream>
#include <cstdlib>
#include <sstream>
using namespace pc_gvf::depth_angular;
using V=Eigen::Vector3d;
void check(bool x,const char* m){if(!x){std::cerr<<m<<'\n';std::exit(1);}}
int main(){PaperConfig c;c.execution_consistent=c.bounded_exploration=true;c.retain_verified_travel=c.retain_certified_volume=true;c.certified_direct=true;c.emergency_braking_certificate=true;c.history_uncertainty_rate=0;c.motion.max_accel=c.motion.brake_accel=16;c.motion.velocity_tau=.1;c.motion.delay=.2;c.motion.rollout_horizon=.2;c.tracking_margin=.015;c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;c.restart_clearance=.02;
PaperGuidance p(c);const double r=p.envelopeRadius();check(p.setVerifiedNeighborhood({V(.9,0,0),V::Zero(),V(-.4,0,0)},r+.045),"seed setup");
Camera cam(24,18,90,68,20);DepthObservation blind(cam,std::vector<double>(24*18,0),V::Zero(),Eigen::Matrix3d::Identity(),1);
auto& a=const_cast<ActiveNavigationState&>(p.activeNavigationState());a.recovering=true;a.anchor_valid=true;a.anchor=V(.9,0,0);a.episode_active=true;a.episode=7;a.unknown_distance=.2;a.unknown_time=.3;a.unknown_volume=.4;
check(!p.motionSafe(blind,V::Zero(),V::Zero(),V(.3,0,0),1),"direct-return fixture unexpectedly free");
check(p.motionSafe(blind,V::Zero(),V::Zero(),V(.15,0,0),1),"slow certified-return fixture missing");
std::stringstream saved(std::ios::in|std::ios::out|std::ios::binary);p.saveReplay(saved,blind,V::Zero(),V::Zero(),V::UnitX(),1,1./60.);
auto result=p.step(blind,V::Zero(),V::Zero(),V::UnitX(),1,1./60.);std::cout<<result.status<<" "<<result.command.transpose()<<'\n';
check(result.accepted&&result.authorization=="CERTIFIED"&&result.navigation_mode=="RECOVER"&&
    result.status=="ACTIVE_CERTIFIED_RECOVER"&&result.command.x()>0.&&result.command.norm()<=.15+1e-12,
    "lower-speed certified direct return missing");
// Recheck the exact pre-publication delayed target used by the certificate.
// step() has already committed the NEW command for the following interval.
const V published=p.publishedCommand();p.confirmPublishedCommand(V::Zero());
const bool safe=p.motionSafe(blind,V::Zero(),V::Zero(),result.command,1);p.confirmPublishedCommand(published);
check(safe,"return lacks whole motion proof with actual prior published target");
check(a.episode==7&&a.episode_active&&a.unknown_distance>=.2&&a.unknown_time>=.3&&a.unknown_volume>=.4,"side reposition erased budget");
saved.seekg(0);auto replay=PaperGuidance::replay(saved);check(replay.status==result.status&&(replay.command-result.command).norm()<1e-12,"replay mismatch");
// Runtime may have cancelled a slow complete fan search. Its untrusted
// nomination should be tried first, with the actual ZERO published target.
p.confirmPublishedCommand(V::Zero());blind.stamp=1.05;
std::stringstream cached(std::ios::in|std::ios::out|std::ios::binary);p.saveReplay(cached,blind,V::Zero(),V::Zero(),V::UnitX(),1.05,1./60.);
check(cached.str().substr(0,8)=="EGOPAPRY","active nomination omitted from replay");
const auto reused=p.step(blind,V::Zero(),V::Zero(),V::UnitX(),1.05,1./60.);
check(reused.status=="ACTIVE_CACHED_CERTIFIED"&&reused.accepted&&
    (reused.command-result.command).norm()<1e-12&&a.unknown_time>=.3,
    "completed active nomination was lost after watchdog or erased risk debt");
cached.seekg(0);const auto reused_replay=PaperGuidance::replay(cached);
check(reused_replay.status==reused.status&&(reused_replay.command-reused.command).norm()<1e-12,"active cached replay changed decision");
p.confirmPublishedCommand(V::Zero());blind.stamp=1.36;
const auto expired=p.step(blind,V::Zero(),V::Zero(),V::UnitX(),1.36,1./60.);
check(expired.status!="ACTIVE_CACHED_CERTIFIED","cached reuse renewed its original .3s lifetime");
// A contradictory actual native hit remains a hard veto, including this fan.
auto points=std::make_shared<std::vector<V>>();points->push_back(V(.1,0,0));blind.obstacle_points=points;
blind.stamp=1.4;const auto blocked=p.step(blind,V::Zero(),V::Zero(),V::UnitX(),1.4,1./60.);
check(!blocked.accepted&&blocked.status=="ACTIVE_KNOWN_OBSTACLE","native occupied body bypassed");
std::cout<<"certified return fan, unchanged budget, replay, hard obstacle veto: PASS\n";}
