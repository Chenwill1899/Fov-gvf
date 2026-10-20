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
void replayMatches(PaperGuidance& control,const DepthObservation& o,const V& p,const V& v,const V& q,double t) {
    std::stringstream s(std::ios::in|std::ios::out|std::ios::binary);control.saveReplay(s,o,p,v,q,t,1./60.);
    check(s.str().substr(0,8)=="EGOPAPRU","active replay not version U");
    auto a=control.step(o,p,v,q,t,1./60.);s.seekg(0);auto b=PaperGuidance::replay(s);
    check(a.status==b.status&&a.authorization==b.authorization&&a.navigation_mode==b.navigation_mode&&
        (a.command-b.command).norm()<1e-12&&a.probe_episode==b.probe_episode&&
        a.unknown_volume==b.unknown_volume&&a.expected_visibility==b.expected_visibility,"active replay decision/state mismatch");
}
int main() {
    auto cfg=config();PaperGuidance control(cfg);control.setVerifiedSeed(V::Zero(),2.5);
    // Revocation must reach the independently retained latest command proof,
    // not only the ordinary spatial-history tubes.
    PaperGuidance committed(cfg);committed.setVerifiedSeed(V::Zero(),2.5);
    auto start=views(V::Zero(),1.);
    auto issued=committed.step(start,V::Zero(),V::Zero(),V::UnitZ(),1.,1./60.);
    check(issued.accepted&&issued.authorization=="CERTIFIED","commit fixture failed");
    DepthObservation intrusion(Camera(24,18,90,74,20),std::vector<double>(24*18,0.),
        V(-2,0,0),fixedCameraRotation(),1.1,11000,0);
    intrusion.obstacle_points=std::make_shared<std::vector<V>>(1,
        intrusion.rotation.transpose()*(V(0,0,.1)-intrusion.origin));
    committed.ingestObservation(intrusion);
    auto blind_start=views(V::Zero(),1.1,true);
    check(!committed.envelopeKnown(blind_start,V::Zero(),committed.envelopeRadius(),1.1),
        "native conflict left committed motion as valid free evidence");
    const V edge(0,0,2.5-control.envelopeRadius()+.001);
    auto o=views(edge,1.);check(!control.envelopeKnown(o,edge,control.envelopeRadius(),1.),"fixture cap should be unknown");
    const auto balls=control.recordedBalls(),tubes=control.recordedTubes();
    auto r=control.step(o,edge,V::Zero(),V::UnitZ(),1.,1./60.);
    std::cout<<"edge "<<r.status<<" cmd "<<r.command.transpose()<<" gain "<<r.expected_visibility<<'\n';
    check(r.accepted&&r.authorization=="EXPLORATORY"&&r.command.norm()<=cfg.probe_speed+1e-10,"unknown cap never obtains bounded exploration");
    check(control.recordedBalls()==balls&&control.recordedTubes()==tubes,"exploratory volume became known free");
    check(!control.envelopeKnown(o,edge,control.envelopeRadius(),1.),"risk authorization polluted freedom proof");
    replayMatches(control,views(edge,1.1),edge,V::Zero(),V::UnitZ(),1.1);
    const auto episode=control.activeNavigationState().episode;
    const double volume=control.activeNavigationState().unknown_volume;
    control.cancelActiveNavigation();control.reset();
    auto released=control.step(views(edge,1.2),edge,V::Zero(),V::Zero(),1.2,1./60.);
    check(!released.accepted&&released.command.norm()==0&&control.activeNavigationState().episode==episode&&
        control.activeNavigationState().unknown_volume==volume,"release reset the risk budget or kept motion");
    auto stale=control.step(views(edge,1.2),edge,V::Zero(),V::UnitZ(),2.,1./60.);
    check(!stale.accepted&&stale.status=="ACTIVE_STALE_DEPTH","stale depth authorized exploration");
    PaperGuidance blind(cfg);blind.setVerifiedSeed(V::Zero(),2.5);
    const auto invalid=blind.step(views(edge,1.,true),edge,V::Zero(),V::UnitZ(),1.,.02);
    check(!invalid.accepted&&invalid.status=="PROBE_NO_USABLE_DEPTH","all-invalid cameras authorized blind exploration");
    PaperGuidance no_anchor(cfg);auto none=no_anchor.step(o,edge,V::Zero(),V::UnitZ(),1.,1./60.);
    check(!none.accepted&&none.status=="PROBE_NO_ANCHOR","exploration without return anchor");
    // Native hits are camera-local, must transform to world, including replay.
    auto hit=views(edge,3.);auto physical=std::const_pointer_cast<DepthObservation>(hit.supporting_views[0]);
    physical->obstacle_points=std::make_shared<std::vector<V>>(1,physical->rotation.transpose()*(edge+V(0,0,.35)-physical->origin));
    auto blocked=control.step(hit,edge,V::Zero(),V::UnitZ(),3.,1./60.);
    check(!blocked.accepted&&blocked.status=="ACTIVE_KNOWN_OBSTACLE","known overhead hit bypassed hard gate");
    replayMatches(control,hit,edge,V::Zero(),V::UnitZ(),3.);
    // No new information, no measured progress: repeated control cycles exhaust
    // the SAME episode, and neither release nor a new frame restores allowance.
    PaperGuidance stuck(cfg);stuck.setVerifiedSeed(V::Zero(),2.5);
    bool exhausted=false;
    for(int i=0;i<160;++i) {
        const double t=1.+.1*i;auto a=stuck.step(views(edge,t),edge,V::Zero(),V::UnitZ(),t,.1);
        exhausted|=a.status=="PROBE_BUDGET_EXHAUSTED";
        if(exhausted)check(!a.accepted,"budget revived without new information");
    }
    check(exhausted,"no-information repeated probing failed to exhaust budget");
    const auto budget=stuck.activeNavigationState().unknown_time;stuck.cancelActiveNavigation();
    auto resumed=stuck.step(views(edge,18.),edge,V::Zero(),V::UnitZ(),18.,.1);
    check(!resumed.accepted&&stuck.activeNavigationState().unknown_time>=budget,"release renewed exhausted episode");
    auto floor_cfg=cfg;floor_cfg.free_space_floor_enabled=true;
    PaperGuidance floor(floor_cfg);floor.setVerifiedSeed(V::Zero(),2.5);
    check(!floor.step(views(V::Zero(),1.),V::Zero(),V::Zero(),-V::UnitZ(),1.,.02).accepted,"exploration crossed hard floor");
    // 60Hz closed loop, four horizontal cameras, no true-map input to controller.
    // The empty world is used only by this synthetic sensor fixture.
    PaperGuidance flight(cfg);flight.setVerifiedSeed(V::Zero(),2.5);V p=V::Zero(),v=V::Zero();
    double lateral=0.,peak=0.;int probes=0,certified=0;
    for(int i=0;i<960;++i) {
        const double t=1.+i/60.;auto sensor=views(p,t);
        if(i%6==0)for(const auto& view:sensor.supporting_views)flight.ingestObservation(*view);
        const V q=i<600?V::UnitZ().eval():-V::UnitZ().eval();
        auto a=flight.step(sensor,p,v,q,t,1./60.);
        probes+=a.authorization=="EXPLORATORY";certified+=a.authorization=="CERTIFIED";
        if(a.authorization=="CERTIFIED")check(flight.motionSafe(sensor,p,v,a.command,t),"committed strict command lost proof");
        const V error=a.command-v;
        v+=clampNorm((a.command.norm()<1e-8?1.:1./6.)*error,16./60.);p+=v/60.;
        lateral=std::max(lateral,p.head<2>().norm());if(i<600)peak=std::max(peak,p.z());
        if(a.authorization=="CERTIFIED")check(flight.envelopeKnown(sensor,p,flight.envelopeRadius(),t),"60Hz execution left committed proof");
    }
    std::cout<<"closed loop p="<<p.transpose()<<" peak="<<peak<<" lateral="<<lateral<<" probes="<<probes<<" certified="<<certified<<'\n';
    check(lateral>.4&&peak>2.2&&p.z()<peak-.4&&certified>0,"four-camera assisted climb made no useful motion");
    // Initially hidden ceiling + side wall, observed only through the four
    // horizontal depth cameras. Oracle inequalities below only audit motion.
    PaperGuidance chamber(cfg);chamber.setVerifiedSeed(V::Zero(),.8);p.setZero();v.setZero();
    double ceiling_peak=0.;
    for(int i=0;i<480;++i) {
        const double t=1.+i/60.;auto sensor=views(p,t,false,1.2,1.8);
        if(i%6==0)for(const auto& view:sensor.supporting_views)chamber.ingestObservation(*view);
        const auto a=chamber.step(sensor,p,v,V::UnitZ(),t,1./60.);
        const V error=a.command-v;v+=clampNorm((a.command.norm()<1e-8?1.:1./6.)*error,16./60.);p+=v/60.;
        ceiling_peak=std::max(ceiling_peak,p.z());
        check(p.z()+.48<=1.2+1e-9&&p.x()+.48<=1.8+1e-9,"hidden ceiling/side wall collided during closed loop");
    }
    std::cout<<"hidden-ceiling peak="<<ceiling_peak<<" final="<<p.transpose()<<'\n';
    check(ceiling_peak>.05,"hidden ceiling fixture produced no navigation");
    std::cout<<"active navigation checks passed\n";
}
