#include "pc_gvf/goal_proposal_cache.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <cstdlib>
#include <iostream>
#include <sstream>
using namespace pc_gvf::depth_angular;
using V=Eigen::Vector3d;
void check(bool ok,const char* why){if(!ok){std::cerr<<why<<'\n';std::exit(1);}}
DepthObservation views(const V& p,double t,bool blind=false,double ceiling=std::numeric_limits<double>::infinity(),double wall=std::numeric_limits<double>::infinity()) {
    Camera c(24,18,90,74,20);
    DepthObservation o(c,std::vector<double>(24*18,0.),p,Eigen::Matrix3d::Identity(),t,1);
    for(int i=0;i<4;++i) {
        const auto yaw=Eigen::AngleAxisd(i*std::acos(-1.)/2,V::UnitZ()).toRotationMatrix();
        auto view=std::make_shared<DepthObservation>(c,std::vector<double>(24*18,blind?0.:20.),
            p+yaw*V(.22,0,.02),yaw*fixedCameraRotation(),t,static_cast<std::uint64_t>(t*10000),i);
        auto hits=std::make_shared<std::vector<V>>();
        if(!blind)for(int y=0;y<c.height();++y)for(int x=0;x<c.width();++x) {
            const V ray=c.ray(x,y),world=view->rotation*ray;
            double length=std::numeric_limits<double>::infinity();
            if(world.z()>1e-9)length=std::min(length,(ceiling-view->origin.z())/world.z());
            if(world.x()>1e-9)length=std::min(length,(wall-view->origin.x())/world.x());
            if(length>0.&&length*ray.z()<c.maxDepth()) {
                view->depth[y*c.width()+x]=length*ray.z();hits->push_back(length*ray);
            }
        }
        view->obstacle_points=hits;o.supporting_views.push_back(view);
    }
    return o;
}
PaperConfig config() {
    PaperConfig c;c.execution_consistent=c.assisted_vertical=c.bounded_exploration=true;
    c.retain_verified_travel=c.retain_certified_volume=true;c.emergency_braking_certificate=true;
    c.certified_direct=true;c.history_duration=60;c.history_uncertainty_rate=c.uncertainty_rate=0.;
    c.motion.max_accel=c.motion.brake_accel=16;c.motion.velocity_tau=.1;c.motion.delay=0.;
    c.motion.rollout_horizon=.2;c.restart_clearance=.02;c.history_sample_interval=.1;
    return c;
}
void boundaryLateralCheck() {
    auto c=config();c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;c.tracking_margin=.015;
    PaperGuidance size(c);c.free_space_ceiling_enabled=true;c.free_space_ceiling_z=size.envelopeRadius()+.12;
    PaperGuidance control(c);const V anchor(-1.2,0,0);control.setVerifiedSeed(anchor,.9);
    auto current=views(V::Zero(),8.);
    auto old=std::make_shared<DepthObservation>(*views(anchor,8.).supporting_views.front());
    old->depth.assign(old->depth.size(),0.); // Conservative unknown pooled view, raw hit still retained.
    const V obstacle(0,0,-control.envelopeRadius()-.025);
    const V raw=old->rotation.transpose()*(obstacle-old->origin);
    Eigen::Vector2d pixel;check(old->camera.pixelFromDirection(raw,&pixel),"below-body fixture native point outside its source FOV");
    old->obstacle_points=std::make_shared<const std::vector<V>>(std::vector<V>{raw});current.supporting_views.push_back(old);
    check(!control.envelopeKnown(current,V::Zero(),control.envelopeRadius()+c.restart_clearance,8.),
        "boundary lateral fixture restart shell should remain unknown");
    const auto lateral=control.step(current,V::Zero(),V::Zero(),V::UnitX(),8.,1./60.);
    check(lateral.accepted&&lateral.authorization=="EXPLORATORY"&&std::abs(lateral.command.z())<1e-9&&lateral.command.norm()>0.,
        "blocked inward descent had no bounded horizontal parallax recovery");
}
void replayMatches(PaperGuidance& control,const DepthObservation& o,const V& p,const V& v,const V& q,double t) {
    std::stringstream s(std::ios::in|std::ios::out|std::ios::binary);control.saveReplay(s,o,p,v,q,t,1./60.);
    check(s.str().substr(0,8)=="EGOPAPRT","active replay not version T");
    auto a=control.step(o,p,v,q,t,1./60.);s.seekg(0);auto b=PaperGuidance::replay(s);
    check(a.status==b.status&&a.authorization==b.authorization&&a.navigation_mode==b.navigation_mode&&
        (a.command-b.command).norm()<1e-12&&a.probe_episode==b.probe_episode&&
        a.unknown_volume==b.unknown_volume&&a.expected_visibility==b.expected_visibility,"active replay decision/state mismatch");
}
int main() {
    boundaryLateralCheck();
    auto c=config();c.motion.body_radius=.48;c.motion.safety_margin=.02;c.motion.rollout_margin=.02;c.tracking_margin=.015;
    PaperGuidance retreat(c);const V anchor(-1,0,0);retreat.setVerifiedSeed(anchor,.9);
    auto first=retreat.step(views(anchor,1.),anchor,V::Zero(),V::UnitZ(),1.,1./60.);
    check(retreat.activeNavigationState().anchor_valid,"retreat anchor missing");
    const V position=V::Zero();const double wall=retreat.envelopeRadius()+.04;
    auto sensor=views(position,1.1,false,std::numeric_limits<double>::infinity(),wall);
    for(const auto& view:sensor.supporting_views)retreat.ingestObservation(*view);
    check(!retreat.retainedDepthObservations().empty(),"native-history fixture retained no views");
    for(const auto& view:retreat.retainedDepthObservations())
        check(bool(view->obstacle_points),"bounded exploration discarded immutable native history");
    const auto r=retreat.step(sensor,position,V::Zero(),V::UnitX(),1.1,1./60.);
    std::cout<<"buffer retreat "<<r.status<<" command "<<r.command.transpose()<<" radius "<<retreat.envelopeRadius()<<" wall "<<wall<<'\n';
    check(r.accepted&&r.command.x()<0,"extra-clearance buffer blocked retreat");
    check(r.authorization=="EXPLORATORY"||r.authorization=="CERTIFIED","retreat authorization missing");
    auto danger=views(position,1.2,false,std::numeric_limits<double>::infinity(),retreat.envelopeRadius()*.6);
    const auto b=retreat.step(danger,position,V::Zero(),V::UnitX(),1.2,1./60.);
    check(!b.accepted&&b.status=="ACTIVE_KNOWN_OBSTACLE","retreat relaxed actual occupied envelope");
    GoalProposalCache cache;cache.save(V::Zero(),V::UnitX(),V::UnitX(),10.,1.);
    check(cache.proposal(V(1,0,0),V::UnitX(),1.1,.3).isZero(),"legacy cache displacement changed");
    check(cache.proposal(V(1,0,0),V::UnitX(),1.1,.3,2.).norm()>.9,"larger nomination displacement not applied");
    check(cache.proposal(V(1,0,0),V::UnitY(),1.1,.3,2.).isZero(),"changed intent retained nomination");
    check(cache.proposal(V(1,0,0),V::UnitX(),1.31,.3,2.).isZero(),"larger displacement renewed cache lifetime");
    PaperGuidance stop_buffer(c);stop_buffer.setVerifiedSeed(anchor,.9);
    stop_buffer.step(views(anchor,1.),anchor,V::Zero(),V::UnitZ(),1.,1./60.);
    auto edge=views(position,1.1,false,std::numeric_limits<double>::infinity(),stop_buffer.envelopeRadius()+.09);
    for(const auto& view:edge.supporting_views)stop_buffer.ingestObservation(*view);
    const auto exit=stop_buffer.step(edge,position,V::Zero(),V::UnitX(),1.1,1./60.);
    check(exit.accepted&&exit.command.x()<0.,"stop-only extra comfort shell blocked valid monotone return");
    PaperGuidance corridor(c);corridor.setVerifiedSeed(anchor,.9);
    corridor.step(views(anchor,1.),anchor,V::Zero(),V::UnitZ(),1.,1./60.);
    auto narrow=views(position,1.1);
    auto raw=std::make_shared<std::vector<V>>();
    const auto front=narrow.supporting_views.front();
    for(double side:{-1.,1.})raw->push_back(front->rotation.transpose()*(V(-.2,side*.64,0)-front->origin));
    const_cast<DepthObservation&>(*front).obstacle_points=raw;
    for(const auto& view:narrow.supporting_views)corridor.ingestObservation(*view);
    const_cast<ActiveNavigationState&>(corridor.activeNavigationState()).recovering=true;
    auto tight=corridor.step(narrow,position,V::Zero(),V::UnitX(),1.1,1./60.);
    check(tight.accepted&&tight.command.x()<0.,"outer comfort band locked a collision-free narrow return");
    PaperGuidance anchor_fix(c);anchor_fix.setVerifiedSeed(V::Zero(),2.5);
    const V rim(0,0,2.5-anchor_fix.envelopeRadius()+.0005);
    auto& state=const_cast<ActiveNavigationState&>(anchor_fix.activeNavigationState());
    state.anchor=rim-V(0,0,.001);state.anchor_valid=true;state.recovering=true;
    state.episode_active=true;state.episode=3;state.unknown_distance=.1;state.unknown_time=.2;state.unknown_volume=.3;
    auto a=anchor_fix.step(views(rim,2.),rim,V::Zero(),V::UnitZ(),2.,1./60.);
    check((anchor_fix.activeNavigationState().anchor-rim).norm()>.12,"degenerate anchor was not replaced");
    check(anchor_fix.activeNavigationState().episode==3&&anchor_fix.activeNavigationState().unknown_distance>=.1&&
        anchor_fix.activeNavigationState().unknown_time>=.2&&anchor_fix.activeNavigationState().unknown_volume>=.3,
        "anchor selection reset cumulative risk budget");
    std::cout<<"alternate anchor "<<a.status<<" anchor "<<anchor_fix.activeNavigationState().anchor.transpose()<<'\n';
    PaperGuidance handoff(c);handoff.setVerifiedSeed(V::Zero(),2.5);
    auto& h=const_cast<ActiveNavigationState&>(handoff.activeNavigationState());
    h.anchor=V::Zero();h.anchor_valid=true;h.episode_active=true;h.episode=7;
    h.unknown_distance=.4;h.unknown_time=.5;h.unknown_volume=.6;
    h.last_position=V::Zero();h.last_time=2.;h.last_unknown=true;
    const auto known=views(V(.01,0,0),2.1);
    check(!handoff.activeNavigationRequested(known,V(.01,0,0),V::UnitX(),2.1),
        "known interior failed to hand control back to normal obstacle avoidance");
    const auto charged=handoff.activeNavigationState();
    check(charged.episode_active&&charged.episode==7&&charged.unknown_distance>=.41-1e-12&&
        charged.unknown_time>=.6-1e-12&&charged.unknown_volume>.6,"handoff erased cumulative exposure");
    check(!handoff.activeNavigationRequested(known,V(.01,0,0),V::UnitX(),2.1)&&
        handoff.activeNavigationState().unknown_time==charged.unknown_time,
        "repeated predicate double-charged exposure");
    h.recovering=true;h.anchor=V(-1,0,0);
    check(handoff.activeNavigationRequested(known,V(.01,0,0),V::UnitX(),2.1),"handoff skipped required recovery");
    h.anchor=V::Zero();
    check(!handoff.activeNavigationRequested(known,V(.01,0,0),V::UnitX(),2.1)&&!h.recovering,
        "completed interior return failed to release recovery");
    h.recovering=false;h.anchor=V::Zero();h.anchor_stamp=2.;h.episode_origin=V::Zero();h.episode_origin_valid=true;h.episode_stamp=2.;
    h.critical_targets={V(1.3,.1,.05)};
    handoff.activeNavigationRequested(views(V(.3,0,0),2.2),V(.3,0,0),V::UnitX(),2.2);
    check(!h.episode_active&&h.unknown_distance==0.&&h.unknown_time==0.,
        "new measured target coverage during certified handoff failed to close episode");
    PaperGuidance separate(c);separate.setVerifiedSeed(V::Zero(),2.5);
    auto& episode=const_cast<ActiveNavigationState&>(separate.activeNavigationState());
    episode.anchor=V(.5,0,0);episode.anchor_valid=true;episode.anchor_stamp=1.;
    episode.episode_active=true;episode.episode=4;episode.episode_origin=V::Zero();episode.episode_origin_valid=true;episode.episode_stamp=1.;
    episode.critical_targets={V(1.5,.1,.05)};episode.unknown_distance=.4;episode.unknown_time=.5;episode.unknown_volume=.6;
    separate.activeNavigationRequested(views(V(.51,0,0),2.),V(.51,0,0),V::UnitX(),2.);
    check(!episode.episode_active&&episode.unknown_distance==0.,
        "moving reliable return destination erased episode displacement history");
    PaperGuidance legacy_episode(c);legacy_episode.setVerifiedSeed(V::Zero(),2.5);
    auto& migrated=const_cast<ActiveNavigationState&>(legacy_episode.activeNavigationState());
    migrated.episode_active=true;migrated.unknown_distance=.3;migrated.unknown_time=.4;migrated.unknown_volume=.5;
    migrated.episode_origin_valid=false;migrated.anchor_valid=false;migrated.observed_targets=1;
    legacy_episode.activeNavigationRequested(views(V(.1,0,0),2.),V(.1,0,0),V::UnitX(),2.);
    check(migrated.episode_active&&migrated.unknown_distance>=.3&&migrated.unknown_time>=.4,
        "legacy replay without a known episode origin replenished its budget");
    PaperGuidance reentry(c);reentry.setVerifiedSeed(V::Zero(),2.5);
    auto& return_state=const_cast<ActiveNavigationState&>(reentry.activeNavigationState());
    return_state.anchor=V(-1,0,0);return_state.anchor_valid=true;return_state.recovering=true;
    return_state.episode_active=true;return_state.episode=5;return_state.episode_origin=V::Zero();return_state.episode_origin_valid=true;return_state.episode_stamp=2.;
    return_state.unknown_distance=.4;return_state.unknown_time=.5;return_state.unknown_volume=.6;
    const V thin(0,0,2.5-reentry.envelopeRadius()-.025);
    auto forward=reentry.step(views(thin,3.),thin,V::Zero(),V::UnitX(),3.,1./60.);
    check(forward.accepted&&forward.authorization=="CERTIFIED"&&forward.command.x()>0.&&
        !return_state.recovering&&return_state.episode_active&&return_state.unknown_distance>=.4,
        "certified forward execution failed recovery handoff or renewed risk budget");
    PaperGuidance rim_anchor(c);rim_anchor.setVerifiedSeed(V::Zero(),2.5);
    const V restart_rim(0,0,2.5-rim_anchor.envelopeRadius()-.025);
    auto rim_views=views(restart_rim,3.);
    auto& rim_state=const_cast<ActiveNavigationState&>(rim_anchor.activeNavigationState());
    rim_state.anchor=restart_rim;rim_state.anchor_valid=true;rim_state.recovering=true;
    auto rim_return=rim_anchor.step(rim_views,restart_rim,V::Zero(),V::UnitX(),3.,1./60.);
    check((rim_state.anchor-restart_rim).norm()>.12&&rim_return.accepted,
        "minimal restart proof replaced the reliable interior return destination");
    rim_anchor.confirmPublishedCommand(V::Zero());
    auto after_watchdog=rim_anchor.step(views(restart_rim,3.+1./60.),restart_rim,V::Zero(),V::UnitX(),3.+1./60.,1./60.);
    check(after_watchdog.accepted&&(rim_state.anchor-restart_rim).norm()>.12,
        "watchdog zero recreated the degenerate rim anchor before physical return");
    PaperGuidance admission(c);admission.setVerifiedSeed(V::Zero(),2.5);
    const V start_cap(0,0,2.5-admission.envelopeRadius()-.01);
    auto cap_views=views(start_cap,4.);
    check(admission.envelopeKnown(cap_views,start_cap,admission.envelopeRadius(),4.),"fixture body should be known");
    check(!admission.envelopeKnown(cap_views,start_cap,admission.envelopeRadius()+c.restart_clearance,4.),
        "fixture execution restart shell should remain unknown");
    check(admission.activeNavigationRequested(cap_views,start_cap,V::UnitX(),4.),
        "missing initial execution envelope entered impossible ordinary field search");
    const auto recovery=admission.step(cap_views,start_cap,V::Zero(),V::UnitX(),4.,1./60.);
    check(recovery.accepted&&recovery.authorization=="EXPLORATORY","missing restart evidence failed bounded recovery assessment");
    std::cout<<"execution admission "<<recovery.status<<" command "<<recovery.command.transpose()<<'\n';
    auto& cancelled=const_cast<ActiveNavigationState&>(admission.activeNavigationState());
    const double spent=cancelled.unknown_time,volume_spent=cancelled.unknown_volume;
    check(cancelled.last_unknown,"accepted unknown candidate lost pending exposure");
    admission.confirmExecutionOverride(cap_views,start_cap,V::Zero(),V::Zero(),V::Zero(),4.);
    check(!cancelled.last_unknown&&admission.publishedCommand().isZero()&&
        cancelled.unknown_time==spent&&cancelled.unknown_volume==volume_spent,
        "watchdog cancellation charged an unexecuted stationary probe or erased past debt");
    admission.step(views(start_cap,4.1),start_cap,V::Zero(),V::UnitX(),4.1,.1);
    check(cancelled.unknown_time==spent,"cancelled known-space hold consumed active exploration time");
    PaperGuidance unproved_stop(c);
    auto& unproved=const_cast<ActiveNavigationState&>(unproved_stop.activeNavigationState());
    unproved.episode_active=true;unproved.last_unknown=true;unproved.unknown_time=.7;
    unproved_stop.confirmExecutionOverride(views(V::Zero(),4.,true),V::Zero(),V::Zero(),V::Zero(),V::Zero(),4.);
    check(unproved.last_unknown&&unproved.unknown_time==.7,"unknown physical body escaped exposure accounting");
    auto delayed_config=c;delayed_config.motion.delay=.2;
    PaperGuidance delayed_stop(delayed_config);delayed_stop.setVerifiedSeed(V::Zero(),2.5);
    auto& delayed=const_cast<ActiveNavigationState&>(delayed_stop.activeNavigationState());delayed.episode_active=true;
    const V delayed_cap(0,0,2.5-delayed_stop.envelopeRadius()-.01);
    delayed_stop.confirmExecutionOverride(views(delayed_cap,4.,true),delayed_cap,V::Zero(),V(0,0,.3),V::Zero(),4.);
    check(delayed.last_unknown,"override ignored still-delayed outward command entering unknown space");
    // A continuous shell may fail although all sparse witnesses are known.
    // An empty event must close on independently measured new full-body room,
    // not on an old seed, repeated stamp, stick release or translation alone.
    PaperGuidance empty_targets(c);empty_targets.setVerifiedSeed(V::Zero(),2.5);
    auto& empty=const_cast<ActiveNavigationState&>(empty_targets.activeNavigationState());
    empty.episode_active=true;empty.episode=9;empty.episode_origin_valid=true;
    empty.episode_origin=V::Zero();empty.episode_stamp=2.;empty.anchor_stamp=2.;
    empty.anchor_valid=true;empty.unknown_distance=.4;empty.unknown_time=.5;empty.unknown_volume=.6;
    auto measured=[&](double capture) {
        auto o=views(V(.3,0,0),2.2,true);
        Camera camera(24,18,90,74,20);
        o.supporting_views.push_back(std::make_shared<DepthObservation>(camera,
            std::vector<double>(24*18,20.),V(-3,0,0),fixedCameraRotation(),capture,100,7));
        return o;
    };
    empty_targets.activeNavigationRequested(measured(1.),V(.3,0,0),V::UnitX(),2.2);
    check(empty.episode_active&&empty.unknown_time>=.5,"empty event renewed budget using old views/seed");
    empty_targets.activeNavigationRequested(measured(2.2),V(.1,0,0),V::UnitX(),2.2);
    check(empty.episode_active,"empty event closed without measured displacement");
    empty_targets.activeNavigationRequested(views(V(.3,0,0),2.2,true),V(.3,0,0),V::UnitX(),2.2);
    check(empty.episode_active,"empty event renewed budget without new measured free space");
    empty_targets.activeNavigationRequested(measured(2.2),V(.3,0,0),V::UnitX(),2.2);
    check(!empty.episode_active&&empty.unknown_time==0.&&empty.unknown_distance==0.,
        "empty witnesses made a newly observed interior episode impossible to close");
    auto bounded=c;bounded.free_space_ceiling_enabled=true;bounded.free_space_ceiling_z=2.5;
    PaperGuidance inward(bounded);inward.setVerifiedSeed(V::Zero(),10.);
    const V high(0,0,2.5-inward.envelopeRadius()-bounded.restart_clearance-.1001);
    auto& near=inward.activeNavigationState();
    auto& writable=const_cast<ActiveNavigationState&>(near);
    writable.anchor=high;writable.anchor_valid=true;writable.recovering=false;
    check(inward.activeNavigationRequested(views(high,5.),high,V::UnitX(),5.),
        "known interior self-anchor cancelled necessary boundary recovery");
    std::stringstream first_boundary(std::ios::in|std::ios::out|std::ios::binary);
    inward.saveReplay(first_boundary,views(high,5.),high,V::Zero(),V::UnitX(),5.,1./60.);
    check(first_boundary.str().substr(0,8)=="EGOPAPRX","uncached boundary hysteresis missing from X replay");
    const auto back=inward.step(views(high,5.),high,V::Zero(),V::UnitX(),5.,1./60.);
    check(back.accepted&&back.authorization=="CERTIFIED"&&back.command.z()<0.,
        "high workspace recovery failed to gain certified inward height room");
    check(!inward.proposalCorridor(views(high,5.1),high,V::UnitZ(),.01,5.1),
        "near-ceiling outward nomination ignored restart room");
    check(inward.proposalCorridor(views(high,5.1),high,-V::UnitZ(),.2,5.1),
        "boundary nomination comfort margin blocked inward retreat");
    const V middle=high-V(0,0,.08);
    check(inward.activeNavigationRequested(views(middle,5.2),middle,V::UnitX(),5.2),
        "boundary recovery exited after only crossing its entry band");
    std::stringstream boundary_snapshot(std::ios::in|std::ios::out|std::ios::binary);
    inward.saveReplay(boundary_snapshot,views(middle,5.2),middle,V::Zero(),V::UnitX(),5.2,1./60.);
    check(boundary_snapshot.str().substr(0,8)=="EGOPAPRZ","boundary-tagged nomination missing from Z replay");
    const auto continued_inward=inward.step(views(middle,5.2),middle,V::Zero(),V::UnitX(),5.2,1./60.);
    check(continued_inward.accepted&&continued_inward.authorization=="CERTIFIED"&&continued_inward.command.z()<0.,
        "boundary exit band failed certified continued inward motion");
    boundary_snapshot.seekg(0);const auto boundary_replayed=PaperGuidance::replay(boundary_snapshot);
    check(boundary_replayed.status==continued_inward.status&&
        (boundary_replayed.command-continued_inward.command).norm()<1e-12,"boundary replay changed the continued return");
    const V deep=high-V(0,0,.3);
    check(!inward.activeNavigationRequested(views(deep,5.3),deep,V::UnitX(),5.3)&&
        !inward.activeNavigationState().recovering,
        "completed inward retreat tried to return to its old upper anchor");
    auto shell_config=c;shell_config.free_space_ceiling_enabled=true;shell_config.free_space_ceiling_z=2.53;
    PaperGuidance shell_inward(shell_config);shell_inward.setVerifiedSeed(V::Zero(),2.5);
    const V shell_cap(0,0,2.5-shell_inward.envelopeRadius()-.01);
    const auto shell_observation=views(shell_cap,5.4);
    check(!shell_inward.envelopeKnown(shell_observation,shell_cap,shell_inward.envelopeRadius()+c.restart_clearance,5.4),
        "inward-risk fixture restart shell unexpectedly known");
    const auto inward_probe=shell_inward.step(shell_observation,shell_cap,V::Zero(),V::UnitX(),5.4,1./60.);
    check(inward_probe.accepted&&inward_probe.authorization=="EXPLORATORY"&&inward_probe.command.z()<0.,
        "unknown boundary recovery did not select inward progress under the original budget");
    PaperGuidance short_return(c);short_return.setVerifiedSeed(V::Zero(),short_return.envelopeRadius()+.04);
    const V short_position(.15,0,0);const auto short_view=views(short_position,5.8);
    auto& short_state=const_cast<ActiveNavigationState&>(short_return.activeNavigationState());
    short_state.episode_active=true;short_state.episode=11;short_state.unknown_time=.8;
    check(short_return.requestVerifiedRetreat(short_view,short_position,5.8)&&
        short_state.verified_retreat&&short_state.anchor.norm()<1e-12&&short_state.unknown_time==.8,
        "nearby reliable return was discarded while generic recovery kept reentering forward");
    PaperGuidance slow_goal(c);slow_goal.setVerifiedSeed(V::Zero(),2.5);
    for(int i=0;i<3;++i) {
        const V p(.5*i,0,0);
        check(slow_goal.rememberVerifiedPosition(views(p,6.+.1*i),p,6.+.1*i),"retreat footprint fixture");
    }
    auto& debt=const_cast<ActiveNavigationState&>(slow_goal.activeNavigationState());
    debt.anchor=V(1,0,0);debt.anchor_valid=true;debt.episode_active=true;
    debt.episode=12;debt.episode_origin=V(1,0,0);debt.episode_origin_valid=true;debt.episode_stamp=6.2;
    debt.critical_targets={V(100,0,0)};debt.unknown_distance=.4;debt.unknown_time=.5;debt.unknown_volume=.6;
    slow_goal.confirmPublishedCommand(V(.01,0,0));
    const auto fresh=views(V(1,0,0),6.3);
    check(slow_goal.requestVerifiedRetreat(fresh,V(1,0,0),6.3),"stalled safe goal failed verified retreat selection");
    const V destination=debt.anchor;
    check((destination-V(1,0,0)).norm()>=.9&&debt.recovering&&debt.unknown_time==.5&&debt.episode==12,
        "retreat request selected self or cleared cumulative debt");
    std::stringstream captured(std::ios::in|std::ios::out|std::ios::binary);
    slow_goal.saveReplay(captured,fresh,V(1,0,0),V::Zero(),V::UnitX(),6.3,1./60.);
    check(captured.str().substr(0,8)=="EGOPAPRW","forced retreat state missing from replay");
    const auto resumed=slow_goal.step(fresh,V(1,0,0),V::Zero(),V::UnitX(),6.3,1./60.);
    check(resumed.accepted&&resumed.command.norm()>.1&&(debt.anchor-destination).norm()<1e-12&&debt.unknown_time>=.5,
        "known interior cancelled requested destination or retained tiny cached speed");
    captured.seekg(0);const auto repeated=PaperGuidance::replay(captured);
    check(repeated.status==resumed.status&&(repeated.command-resumed.command).norm()<1e-12,
        "verified retreat changed replay state semantics");
    return 0;
}
