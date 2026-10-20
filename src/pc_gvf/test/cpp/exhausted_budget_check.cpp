#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <cstdlib>
#include <iostream>
#include <sstream>
using namespace pc_gvf::depth_angular;using V=Eigen::Vector3d;
void check(bool ok,const char* m){if(!ok){std::cerr<<m<<'\n';std::exit(1);}}
PaperConfig config(){PaperConfig c;c.execution_consistent=c.assisted_vertical=c.bounded_exploration=true;c.retain_verified_travel=c.retain_certified_volume=true;c.emergency_braking_certificate=c.certified_direct=true;c.history_duration=60;c.history_uncertainty_rate=c.uncertainty_rate=0.;c.motion.max_accel=c.motion.brake_accel=16;c.motion.velocity_tau=.1;c.motion.delay=0;c.motion.rollout_horizon=.2;c.restart_clearance=.02;c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;c.tracking_margin=.015;return c;}
DepthObservation view(double side){Camera cam(24,18,90,74,20);DepthObservation o(cam,std::vector<double>(24*18,0.),V::Zero(),Eigen::Matrix3d::Identity(),1.1);
for(int i=0;i<4;++i){auto yaw=Eigen::AngleAxisd(i*std::acos(-1.)/2,V::UnitZ()).toRotationMatrix();auto v=std::make_shared<DepthObservation>(cam,std::vector<double>(24*18,20.),yaw*V(.22,0,.02),yaw*fixedCameraRotation(),1.1,11,i);auto hits=std::make_shared<std::vector<V>>();if(i==0)for(double sign:{-1.,1.})hits->push_back(v->rotation.transpose()*(V(-.2,sign*side,0)-v->origin));v->obstacle_points=hits;o.supporting_views.push_back(v);}return o;}
void prepare(PaperGuidance& p){check(p.setVerifiedSeed(V(-1,0,0),.9),"anchor seed");auto& s=const_cast<ActiveNavigationState&>(p.activeNavigationState());s.anchor=V(-1,0,0);s.anchor_valid=s.recovering=s.episode_active=s.episode_origin_valid=true;s.episode_origin=V::Zero();s.episode_stamp=1;s.episode=7;s.unknown_distance=.2;s.unknown_time=.3;s.unknown_volume=.4;}
int main(){
 auto c=config();PaperGuidance p(c);prepare(p);auto& state=const_cast<ActiveNavigationState&>(p.activeNavigationState());state.unknown_time=c.probe_time_budget+.1;
 auto o=view(10.);auto r=p.step(o,V::Zero(),V::Zero(),V::UnitX(),1.1,1./60.);
 check(!r.accepted&&r.command.isZero()&&r.status=="PROBE_BUDGET_EXHAUSTED","spent budget granted unknown motion");
 check(p.activeNavigationState().unknown_time>=c.probe_time_budget+.1,"spent budget was renewed");
 PaperGuidance known(c);check(known.setVerifiedSeed(V::Zero(),3.),"known seed");auto& ks=const_cast<ActiveNavigationState&>(known.activeNavigationState());ks.episode_active=ks.episode_origin_valid=true;ks.episode_origin=V::Zero();ks.episode_stamp=1.;ks.episode=9;ks.unknown_time=c.probe_time_budget+.1;ks.anchor=V::Zero();ks.anchor_valid=true;
 auto k=known.step(o,V::Zero(),V::Zero(),V::UnitX(),1.1,1./60.);
 check(k.accepted&&k.authorization=="CERTIFIED","spent unknown budget blocked a complete certified motion");
 check(known.activeNavigationState().unknown_time>=c.probe_time_budget+.1,"certified handoff silently renewed budget");
 std::cout<<"spent budget denies unknown, permits full certification, never renews: PASS\n";
}
