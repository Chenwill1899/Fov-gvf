#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
using namespace pc_gvf::depth_angular;
void check(bool ok,const char* why){if(!ok){std::cerr<<why<<'\n';std::exit(1);}}
PaperConfig config(){PaperConfig c;c.intent_guard=true;c.smooth_speed=true;c.adaptive_lookahead=true;
 c.adaptive_grid=true;c.field_interval=.1;c.chart_speed=16;c.minimum_lookahead=.1;c.observation_timeout=.3;
 c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;
 c.motion.max_accel=1.2;c.motion.brake_accel=1.2;c.motion.delay=.2;c.motion.rollout_horizon=.2;return c;}
DepthObservation observation(double t,const Eigen::Vector3d& p,const Eigen::Vector3d& v,const Eigen::Vector3d& q){
 Camera camera=intentChartCamera(Camera(24,18,90,68,10));return DepthObservation(camera,std::vector<double>(24*18,0),p,intentChartRotation(v,q),t,static_cast<std::uint64_t>(t*1000));}
double error(const Eigen::Vector3d& v,const Eigen::Vector3d& q){if(v.norm()<.05)return 180;
 return std::atan2(v.cross(q).norm(),v.dot(q))*180/std::acos(-1);}
int main(){
 const auto cfg=config();const auto origin=Eigen::Vector3d::Zero().eval();
 const auto forward=Eigen::Vector3d::UnitX().eval();
 PaperGuidance wrong(cfg);wrong.setVerifiedSeed(origin,100);
 auto rear=observation(1,origin,origin,-forward);
 auto rejected=wrong.step(rear,origin,origin,forward,1,.02);
 check(!rejected.accepted&&rejected.build_reason=="NO_INTENT_PROGRESS","rear-facing chart converted forward intent into reverse flight");
 // A real frustum plane leaves only steep forward/downward directions.
 // The body is observed but +X intersects unknown space almost immediately.
 // A narrow virtual chart must not hide the certified >60deg pitch option.
 Camera depth_camera(80,60,60,60,10);Eigen::Matrix3d down=Eigen::Matrix3d::Identity();down(1,1)=-1;down(2,2)=-1;
 PaperGuidance slope(cfg);const double radius=slope.envelopeRadius();
 const double x0=2*std::tan(std::acos(-1.)/6)-(radius+.003)/std::cos(std::acos(-1.)/6);
 auto observed=std::make_shared<DepthObservation>(depth_camera,std::vector<double>(80*60,10),Eigen::Vector3d(-x0,0,2),down,1,1);
 auto steep=observation(1,origin,origin,forward);steep.supporting_views.push_back(observed);
 check(slope.envelopeKnown(steep,origin,radius,1),"steep fixture body is not observed");
 auto descend=slope.step(steep,origin,origin,forward,1,.02);
 std::cout<<"steep status="<<descend.status<<" build="<<descend.build_reason<<" cmd="<<descend.command.transpose()<<" best="<<descend.best_prefix<<std::endl;
 check(descend.accepted&&descend.command.x()>0&&descend.command.z() < -1.5*descend.command.x(),"virtual chart lost safe steep forward direction");
 check(slope.motionSafe(steep,origin,origin,descend.command,1),"steep candidate skipped motion certificate");
 std::cout<<"observed steep forward direction accepted: "<<descend.command.transpose()<<'\n';
 PaperGuidance ramp(cfg);ramp.setVerifiedSeed(origin,100);
 Eigen::Vector3d p=origin,v=origin;double previous=0,max_step=0;
 for(int i=0;i<180;++i){double t=1+.02*i;auto o=observation(t,p,v,forward);auto r=ramp.step(o,p,v,2*forward,t,.02);
  check(r.accepted&&r.command.dot(forward)>0&&r.status=="INTENT_DIRECT","clear straight intent was not recaptured");
  check(r.command.norm()<=previous+cfg.motion.max_accel*.02+1e-10,"scalar speed jumped above acceleration ramp");
  check(ramp.motionSafe(o,p,v,r.command,t),"smoothed command bypassed full braking certification");
  max_step=std::max(max_step,r.command.norm()-previous);previous=r.command.norm();
  v+=.02*clampNorm((r.command-v)/cfg.motion.velocity_tau,cfg.motion.max_accel);p+=.02*v;
 }
 check(previous>1.9,"speed ramp never reached requested speed");
 // Nonzero magnitude changes use the same scalar ramp in both directions.
 for(int i=0;i<100;++i){double t=4.6+.02*i;auto r=ramp.step(observation(t,p,v,forward),p,v,.2*forward,t,.02);
  check(r.accepted&&r.command.norm()>=previous-cfg.motion.max_accel*.02-1e-10,"normal speed reduction jumped");
  check(ramp.motionSafe(observation(t,p,v,forward),p,v,r.command,t),"deceleration skipped certification");
  previous=r.command.norm();v+=.02*clampNorm((r.command-v)/cfg.motion.velocity_tau,cfg.motion.max_accel);p+=.02*v;
 }
 check(std::abs(previous-.2)<1e-9,"normal deceleration did not reach requested magnitude");
 ramp.confirmPublishedCommand(origin);
 auto resumed=ramp.step(observation(7,p,v,forward),p,v,2*forward,7,.02);
 check(resumed.command.norm()<=.024+1e-10,"discarded watchdog output survived as next-frame speed history");
 const auto released=ramp.step(observation(7.02,p,v,forward),p,v,origin,7.02,.02);
 check(released.command.isZero()&&released.status=="ZERO_INTENT","scalar ramp delayed operator release");
 PaperGuidance capture(cfg);capture.setVerifiedSeed(origin,100);
 Eigen::Vector3d angled(std::cos(std::acos(-1.)/6),std::sin(std::acos(-1.)/6),0);
 auto cap_obs=observation(1,origin,forward,angled);auto cap=capture.step(cap_obs,origin,forward,angled,1,.02);
 check(cap.accepted&&cap.status=="INTENT_DIRECT"&&std::abs(cap.command.z())<1e-10&&
     error(cap.command.normalized(),angled)<30,"certified direct intent retained vertical avoidance drift");
 check(capture.motionSafe(cap_obs,origin,forward,cap.command,1),"direct intent bypassed braking proof");
 std::cout<<"rear chart rejected; scalar start/restart max increase="<<max_step<<"m/s per20ms; final vectors certified\n";
 for(double turn:{90.,180.}){
  auto turning=cfg;turning.response_preview=true;
  PaperGuidance control(turning);control.setVerifiedSeed(origin,100);p=origin;v=origin;double settle=-1;int reverse_commands=0,brake_frames=0;
  for(int i=0;i<1100;++i){double t=1+.02*i,a=(i<100?0:turn)*std::acos(-1)/180;
   Eigen::Vector3d q(std::cos(a),std::sin(a),0);auto o=observation(t,p,v,q);auto r=control.step(o,p,v,q,t,.02);
   if(r.accepted&&r.command.dot(q)<-1e-9)++reverse_commands;
   brake_frames+=r.status=="INTENT_REVERSAL_BRAKE";
   if(i==100&&turn==180)check(!r.accepted&&r.status=="INTENT_REVERSAL_BRAKE","operator reversal kept pushing along old direction");
   if(r.accepted)check(control.motionSafe(o,p,v,r.command,t),"turn command lost motion certificate");
   const Eigen::Vector3d delta=r.command-v;
   if(r.command.norm()<1e-8)v+=clampNorm(-v,cfg.motion.brake_accel*.02);
   else v+=.02*clampNorm(delta/cfg.motion.velocity_tau,cfg.motion.max_accel);
   p+=.02*v;
   if(i>=100&&settle<0&&error(v,q)<10)settle=(i-100)*.02;

  }
  check(reverse_commands==0,"accepted motion made negative progress toward operator intent");

  std::cout<<turn<<"deg guarded turn: settle="<<settle<<"s braking_frames="<<brake_frames<<" reverse_commands="<<reverse_commands<<'\n';
  check(settle>=0&&settle<(turn==180?3:4),"guarded turn failed bounded recovery");
 }
 // A wall ahead is outside the current stopping distance. The controller
 // must choose a longer viable detour now, not keep q until braking is forced.
 auto early_cfg=cfg;early_cfg.response_preview=true;
 PaperGuidance early(early_cfg);early.setVerifiedSeed(origin,2.5);early.confirmPublishedCommand(forward);
 Camera physical(80,60,90,68,10);std::vector<double> ahead(80*60,10.);
 for(int y=0;y<60;++y)for(int x=35;x<=44;++x)ahead[y*80+x]=3.;
 auto wall=std::make_shared<DepthObservation>(physical,ahead,Eigen::Vector3d(.22,0,.02),fixedCameraRotation(),1,1);
 auto scene=observation(1,origin,forward,forward);scene.supporting_views.push_back(wall);
 auto anticipation=early.step(scene,origin,forward,2*forward,1,.02);
 check(anticipation.accepted&&early.motionSafe(scene,origin,forward,anticipation.command,1),"early detour did not retain full motion proof");
 const auto* built=early.observation();check(built!=nullptr,"early detour field missing");
 const Eigen::Vector3d endpoint=built->rotation*built->camera.rayFromPixel(early.field().goal);
 check(error(endpoint,forward)>10,"short stopping domain incorrectly kept the blocked long-term goal");
 std::cout<<"early detour goal angle="<<error(endpoint,forward)<<"deg before mandatory braking\n";
 // The straight-corridor optimization must keep full-state braking checks,
 // including when a corridor is free but the measured velocity points elsewhere.
 auto fast_cfg=cfg;fast_cfg.certified_direct=true;fast_cfg.response_preview=true;
 PaperGuidance fast(fast_cfg);fast.setVerifiedSeed(origin,100);
 auto clear=observation(1,origin,forward,forward);
 std::stringstream direct_snapshot(std::ios::in|std::ios::out|std::ios::binary);
 fast.saveReplay(direct_snapshot,clear,origin,forward,forward,1,.02);
 auto direct=fast.step(clear,origin,forward,forward,1,.02);
 check(direct.accepted&&direct.status=="CERTIFIED_DIRECT"&&fast.motionSafe(clear,origin,forward,direct.command,1),"direct corridor skipped full trajectory proof");
 direct_snapshot.seekg(0);auto direct_replay=PaperGuidance::replay(direct_snapshot);
 check(direct_replay.status==direct.status&&(direct_replay.command-direct.command).norm()<1e-12,"V11 replay lost direct/preview policy");
 PaperGuidance fast_unknown(fast_cfg);
 check(!fast_unknown.step(clear,origin,origin,forward,1,.02).accepted,"direct optimization treated unknown corridor as free");
 // A legacy direction is only a proposal: the full 3-D state and braking
 // certificate remain mandatory, and the exact proposal is saved for replay.
 PaperGuidance proposed(fast_cfg);proposed.setVerifiedSeed(origin,100);
 const Eigen::Vector3d suggested(2.,.4,.6);
 std::stringstream proposed_snapshot(std::ios::in|std::ios::out|std::ios::binary);
 proposed.saveReplay(proposed_snapshot,clear,origin,origin,2*forward,1,.02,&suggested);
 auto certified=proposed.stepWithProposal(clear,origin,origin,2*forward,suggested,1,.02);
 check(certified.accepted&&certified.status=="DEPTH_PROPOSAL_CERTIFIED"&&
       proposed.motionSafe(clear,origin,origin,certified.command,1),"proposal bypassed complete motion proof");
 check(certified.command.norm()<=.024+1e-10&&std::abs(certified.command.z())<=1.,"proposal bypassed vector magnitude limits");
 proposed_snapshot.seekg(0);auto proposed_replay=PaperGuidance::replay(proposed_snapshot);
 check(proposed_replay.status==certified.status&&(proposed_replay.command-certified.command).norm()<1e-12,"V11 proposal replay mismatch");
 PaperGuidance body_only(fast_cfg);body_only.setVerifiedSeed(origin,body_only.envelopeRadius());
 auto unsafe=body_only.stepWithProposal(clear,origin,origin,forward,forward,1,.02);
 check(!unsafe.accepted&&unsafe.command.isZero(),"legacy proposal authorized unknown space outside body");
 auto reverse=proposed.stepWithProposal(clear,origin,forward,-forward,-forward,1,.02);
 check(!reverse.accepted&&reverse.command.isZero(),"legacy proposal skipped intent reversal braking");
 // In certified open space, vector acceleration and jerk remain bounded
 // through a direction change; all shaped commands still need a motion proof.
 auto responsive_cfg=fast_cfg;responsive_cfg.proposal_response_time=.10;
 PaperGuidance smooth(responsive_cfg);smooth.setVerifiedSeed(origin,100);p=origin;v=origin;
 Eigen::Vector3d last=origin,last_acceleration=origin;
 for(int i=0;i<300;++i) {
  const double t=1+.02*i;Eigen::Vector3d target=i<150?Eigen::Vector3d(1.4,0,0):Eigen::Vector3d(1.2,.7,.2);
  auto obs=observation(t,p,v,target);auto result=smooth.stepWithProposal(obs,p,v,target,target,t,.02);
  check(result.accepted&&result.status=="DEPTH_PROPOSAL_CERTIFIED","open-space smoothed proposal rejected");
  const Eigen::Vector3d acceleration=(result.command-last)/.02;
  check(acceleration.norm()<=1.2+1e-8,"proposal vector acceleration exceeded bound");
  check((acceleration-last_acceleration).norm()/.02<=8.+1e-7,"proposal jerk exceeded bound");
  check(smooth.motionSafe(obs,p,v,result.command,t),"jerk-shaped command lost motion proof");
  last=result.command;last_acceleration=acceleration;
  v+=.02*clampNorm((last-v)/fast_cfg.motion.velocity_tau,fast_cfg.motion.max_accel);p+=.02*v;
 }
 {
  const Eigen::Vector3d target(1.3,.2,.1);auto next=observation(7,p,v,target);
  std::stringstream snapshot(std::ios::in|std::ios::out|std::ios::binary);
  smooth.saveReplay(snapshot,next,p,v,target,7,.02,&target);
  auto expected_next=smooth.stepWithProposal(next,p,v,target,target,7,.02);snapshot.seekg(0);
  auto replay_next=PaperGuidance::replay(snapshot);
  check((expected_next.command-replay_next.command).norm()<1e-12,"V13 lost reference response or nonzero acceleration history");
 }
 smooth.confirmPublishedCommand(origin);
 auto restart=smooth.stepWithProposal(observation(8,p,origin,forward),p,origin,forward,forward,8,.02);
 check(restart.command.norm()<=8.*.02*.02+1e-9,"watchdog reset kept stale proposal acceleration");
 // Command slew and actual vehicle acceleration are distinct contracts.
 // A faster reference must still use the unchanged 1.2 m/s^2 plant/certificate,
 // obey jerk/speed limits, and survive exact replay with the non-default bound.
 auto command_cfg=fast_cfg;command_cfg.proposal_command_accel=2.4;command_cfg.proposal_jerk_limit=6.;
 PaperGuidance command_ramp(command_cfg);command_ramp.setVerifiedSeed(origin,100);p=origin;v=origin;last=origin;last_acceleration=origin;
 double peak_command_accel=0.;
 for(int i=0;i<180;++i) {
  const double t=1+.02*i;const Eigen::Vector3d target=i<100?Eigen::Vector3d(2,0,0):Eigen::Vector3d(1.3,.6,.2);
  auto obs=observation(t,p,v,target);auto r=command_ramp.stepWithProposal(obs,p,v,target,target,t,.02);
  check(r.accepted&&command_ramp.motionSafe(obs,p,v,r.command,t),"faster command ramp bypassed physical motion proof");
  const Eigen::Vector3d a=(r.command-last)/.02;peak_command_accel=std::max(peak_command_accel,a.norm());
  check(a.norm()<=2.4+1e-8&&(a-last_acceleration).norm()/.02<=6.+1e-6,"command acceleration/jerk bound violated");
  const Eigen::Vector3d physical=clampNorm((r.command-v)/command_cfg.motion.velocity_tau,command_cfg.motion.max_accel);
  check(physical.norm()<=1.2+1e-9&&r.command.norm()<=2.+1e-9&&std::abs(r.command.z())<=1.,"command ramp altered physical or speed limits");
  last=r.command;last_acceleration=a;v+=.02*physical;p+=.02*v;
 }
 check(peak_command_accel>2.0,"separate command ramp was not exercised");
 {
  auto obs=observation(5,p,v,forward);const Eigen::Vector3d target=2.*forward;
  std::stringstream snapshot(std::ios::in|std::ios::out|std::ios::binary);
  command_ramp.saveReplay(snapshot,obs,p,v,target,5,.02,&target);
  auto expected=command_ramp.stepWithProposal(obs,p,v,target,target,5,.02);snapshot.seekg(0);
  auto actual=PaperGuidance::replay(snapshot);
  check((actual.command-expected.command).norm()<1e-12,"V17 lost velocity-command ramp bound");
 }
 check(command_ramp.stepWithProposal(observation(5.02,p,v,forward),p,v,origin,forward,5.02,.02).command.isZero(),
     "faster command ramp delayed operator release");
 auto model_cfg=fast_cfg;model_cfg.model_velocity_shaping=true;model_cfg.proposal_jerk_limit=3.;
 PaperGuidance model(model_cfg);model.setVerifiedSeed(origin,100);p=origin;v=origin;last_acceleration=origin;
 for(int i=0;i<400;++i) {
  const double t=1+.02*i;Eigen::Vector3d target=i<150?Eigen::Vector3d(1.4,0,0):Eigen::Vector3d(1.2,.7,.2);
  auto obs=observation(t,p,v,target);auto r=model.stepWithProposal(obs,p,v,target,target,t,.02);
  check(r.accepted&&r.status=="DEPTH_PROPOSAL_CERTIFIED","model-shaped proposal failed in open space");
  const Eigen::Vector3d acceleration=clampNorm((r.command-v)/model_cfg.motion.velocity_tau,model_cfg.motion.max_accel);
  check(acceleration.norm()<=1.2+1e-9,"model shaping exceeded plant acceleration");
  check((acceleration-last_acceleration).norm()/.02<=3.+1e-6,"unclipped model shaping exceeded physical jerk");
  check(model.motionSafe(obs,p,v,r.command,t),"model compensation bypassed full braking proof");
  last_acceleration=acceleration;v+=.02*acceleration;p+=.02*v;
 }
 {
  const Eigen::Vector3d target(1.5,.1,.2);auto next=observation(9,p,v,target);
  std::stringstream snapshot(std::ios::in|std::ios::out|std::ios::binary);
  model.saveReplay(snapshot,next,p,v,target,9,.02,&target);
  auto expected=model.stepWithProposal(next,p,v,target,target,9,.02);snapshot.seekg(0);
  auto actual=PaperGuidance::replay(snapshot);
  check((actual.command-expected.command).norm()<1e-12,"V12 lost model shaping policy or jerk value");
 }
 auto ff_cfg=fast_cfg;ff_cfg.proposal_feedforward=true;ff_cfg.proposal_jerk_limit=3.;
 PaperGuidance ff(ff_cfg),noisy(ff_cfg);ff.setVerifiedSeed(origin,100);noisy.setVerifiedSeed(origin,100);
 p=origin;v=origin;last_acceleration=origin;
 for(int i=0;i<400;++i) {
  const double t=1+.02*i;const Eigen::Vector3d target=i<150?Eigen::Vector3d(1.4,0,0):Eigen::Vector3d(1.2,.7,.2);
  auto obs=observation(t,p,v,target);auto r=ff.stepWithProposal(obs,p,v,target,target,t,.02);
  Eigen::Vector3d measured=v;if(i>10)measured.y()+=(i%2?.04:-.04);
  auto perturbed=noisy.stepWithProposal(obs,p,measured,target,target,t,.02);
  check(r.accepted&&perturbed.accepted&&(r.command-perturbed.command).norm()<1e-11,
      "feedforward reference differentiated measurement jitter");
  const Eigen::Vector3d acceleration=clampNorm((r.command-v)/ff_cfg.motion.velocity_tau,ff_cfg.motion.max_accel);
  check(acceleration.norm()<=1.2+1e-9,"feedforward reference exceeded plant acceleration");
  check((acceleration-last_acceleration).norm()/.02<=3.+1e-6,"unclipped feedforward reference exceeded physical jerk");
  check(ff.motionSafe(obs,p,v,r.command,t),"feedforward command bypassed actual-state braking proof");
  last_acceleration=acceleration;v+=.02*acceleration;p+=.02*v;
 }
 {
  const Eigen::Vector3d target(1.5,.1,.2);auto next=observation(9,p,v,target);
  std::stringstream snapshot(std::ios::in|std::ios::out|std::ios::binary);
  ff.saveReplay(snapshot,next,p,v,target,9,.02,&target);
  auto expected=ff.stepWithProposal(next,p,v,target,target,9,.02);snapshot.seekg(0);
  auto actual=PaperGuidance::replay(snapshot);
  check((actual.command-expected.command).norm()<1e-12,"V16 lost independent velocity reference state");
 }
 for(int i=0;i<200;++i) {
  const double t=9.02+.02*i;const Eigen::Vector3d target(2,.6,1.5);auto obs=observation(t,p,v,target);
  auto r=ff.stepWithProposal(obs,p,v,target,target,t,.02);
  check(r.accepted&&r.command.norm()<=2.+1e-10&&std::abs(r.command.z())<=1.+1e-10,
      "feedforward anti-windup violated uniform command limits");
  v+=.02*clampNorm((r.command-v)/ff_cfg.motion.velocity_tau,ff_cfg.motion.max_accel);p+=.02*v;
 }
 ff.confirmPublishedCommand(origin);
 auto fresh=ff.stepWithProposal(observation(14,origin,origin,forward),origin,origin,forward,forward,14,.02);
 check(fresh.accepted&&fresh.command.norm()<=ff_cfg.motion.velocity_tau*ff_cfg.proposal_jerk_limit*.02+1e-9,
     "publication override retained a stale feedforward reference");
 check(ff.stepWithProposal(observation(14.02,origin,origin,forward),origin,origin,origin,forward,14.02,.02).command.isZero(),
     "feedforward delayed operator release");
 auto static_cfg=fast_cfg;static_cfg.uncertainty_rate=0.;
 PaperGuidance static_control(static_cfg);static_control.setVerifiedSeed(origin,100);
 check(std::abs(static_control.envelopeRadius()-.55)<1e-12,"static budget removed depth or execution margin");
 auto static_obs=observation(1,origin,origin,forward);
 check(static_control.motionSafe(static_obs,origin,origin,.1*forward,1.2),"static compensated observation failed inside freshness window");
 check(!static_control.motionSafe(static_obs,origin,origin,.1*forward,1.4),"static budget bypassed depth freshness cutoff");
 PaperGuidance corridor(fast_cfg);corridor.setVerifiedSeed(origin,2.5);
 auto corridor_obs=observation(1,origin,origin,forward);
 check(corridor.proposalCorridor(corridor_obs,origin,forward,1.,1),"certified seed corridor rejected");
 check(!corridor.proposalCorridor(corridor_obs,origin,forward,3.,1),"goal corridor extended seed into unknown space");
 check(corridor.recordedBalls()==0&&corridor.recordedTubes()==0,"goal query fabricated persistent free evidence");
 check(corridor.proposalPath(corridor_obs,{origin,forward,Eigen::Vector3d(0,1,0)},1),
       "fully observed curved path rejected");
 check(!corridor.proposalPath(corridor_obs,{origin,2.3*forward},1),
       "visible path center authorized unobserved body extent");
 check(!corridor.proposalPath(corridor_obs,{origin,forward},1.4),"path bypassed evidence freshness");
 PaperGuidance disconnected(fast_cfg);
 disconnected.setVerifiedNeighborhood({-forward,forward},.7);
 check(disconnected.envelopeKnown(corridor_obs,-forward,disconnected.envelopeRadius(),1)&&
       disconnected.envelopeKnown(corridor_obs,forward,disconnected.envelopeRadius(),1),"disconnected fixture endpoints invalid");
 check(!disconnected.proposalPath(corridor_obs,{-forward,forward},1),
       "known endpoints authorized an unknown intervening segment");
 check(corridor.recordedBalls()==0&&corridor.recordedTubes()==0,"path query fabricated persistent free evidence");
 // Unknown remains unknown, and both teleoperation policy bits survive replay.
 PaperGuidance unknown(cfg);auto o=observation(1,origin,origin,forward);
 std::stringstream stream(std::ios::in|std::ios::out|std::ios::binary);
 unknown.saveReplay(stream,o,origin,origin,forward,1,.02);auto expected=unknown.step(o,origin,origin,forward,1,.02);
 stream.seekg(0);auto actual=PaperGuidance::replay(stream);
 check(!actual.accepted&&actual.status==expected.status&&actual.command.isZero(),"unknown or replay safety regression");
 PaperGuidance replayed(cfg);replayed.setVerifiedSeed(origin,100);
 for(int i=0;i<5;++i)replayed.step(observation(1+.02*i,origin,origin,forward),origin,origin,forward,1+.02*i,.02);
 std::stringstream moving(std::ios::in|std::ios::out|std::ios::binary);o=observation(1.1,origin,origin,forward);
 replayed.saveReplay(moving,o,origin,origin,forward,1.1,.02);expected=replayed.step(o,origin,origin,forward,1.1,.02);
 moving.seekg(0);actual=PaperGuidance::replay(moving);
 check((expected.command-actual.command).norm()<1e-12&&actual.command.norm()<.15,"replay lost scalar ramp history");
 std::cout<<"intent hemisphere, turning, unknown rejection, publication override and V11 replay: PASS\n";
}
