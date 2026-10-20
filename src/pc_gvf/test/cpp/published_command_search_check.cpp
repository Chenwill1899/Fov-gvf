#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
using namespace pc_gvf::depth_angular;using V=Eigen::Vector3d;
int main(){
 PaperConfig c;c.execution_consistent=true;c.emergency_braking_certificate=true;c.certified_direct=true;
 c.intent_guard=true;c.smooth_speed=false;c.response_preview=true;c.retain_verified_travel=false;c.retain_certified_volume=false;
 c.motion.max_accel=c.motion.brake_accel=16;c.motion.velocity_tau=.1;c.motion.delay=.2;c.motion.rollout_horizon=.2;
 c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;c.depth_uncertainty=.06;c.uncertainty_rate=0.;c.tracking_margin=.015;c.restart_clearance=.02;c.max_speed=10;c.max_vertical_speed=1;
 const V zero=V::Zero(),forward=10*V::UnitX();
 Camera cam(24,18,90,68,20);DepthObservation o(cam,std::vector<double>(24*18,0),zero,intentChartRotation(forward,forward),1,1);
 PaperGuidance planner(c),independent(c);planner.setVerifiedSeed(zero,6);independent.setVerifiedSeed(zero,6);planner.confirmPublishedCommand(forward);independent.confirmPublishedCommand(forward);
 if(!independent.motionSafe(o,zero,forward,zero,1)){std::cerr<<"fixture original command emergency must be proved\n";return 2;}
 auto r=planner.stepWithProposal(o,zero,forward,forward,forward,1,1./60.);
 const bool independently_safe=r.accepted&&independent.motionSafe(o,zero,forward,r.command,1);
 std::cout<<"accepted="<<r.accepted<<" status="<<r.status<<" command="<<r.command.transpose()<<" independent_actual_previous_certificate="<<independently_safe<<"\n";
 if(!independently_safe){std::cerr<<"candidate retries changed the unpublished transport-delay command\n";return 1;}
 return 0;
}
