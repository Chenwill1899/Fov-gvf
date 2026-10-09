#include "pc_gvf/paper_guidance.hpp"
#include "pc_gvf/verified_start.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
using namespace pc_gvf::depth_angular;
void check(bool ok,const char* message){if(!ok){std::cerr<<message<<'\n';std::exit(1);}}
int main(){
    const std::string root=PC_GVF_PROJECT_DIR;
    const std::string scene=root+"/scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd";
    const std::string occupancy=root+"/scenes/ego_swarm_cloud/occupancy.bin";
    std::string evidence;
    check(certifyDefaultCloudStart(scene,occupancy,Eigen::Vector3d(-64,-11.2,2.8),2.5,&evidence),"verified spawn rejected");
    check(!certifyDefaultCloudStart(scene,occupancy,Eigen::Vector3d(-64,-11.2,2.8),3.0,&evidence),"seed extends beyond geometry bounds");
    check(!certifyDefaultCloudStart(scene,"/nonexistent/occupancy",Eigen::Vector3d(-64,-11.2,2.8),2.5,&evidence),"missing geometry accepted");
    std::ifstream stream(occupancy,std::ios::binary);std::vector<unsigned char> cells(400*300*50);
    stream.read(reinterpret_cast<char*>(cells.data()),cells.size());bool tested=false;
    for(int x=10;x<390&&!tested;++x)for(int y=10;y<290&&!tested;++y)for(int z=5;z<45&&!tested;++z)
        if(cells[(x*300+y)*50+z]) {
            const Eigen::Vector3d center=Eigen::Vector3d(-80,-60,0)+.4*(Eigen::Vector3d(x,y,z)+Eigen::Vector3d::Constant(.5));
            check(!certifyDefaultCloudStart(scene,occupancy,center,.4,&evidence),"occupied seed accepted");tested=true;
        }
    check(tested,"occupied geometry test had no fixture");
    PaperConfig cfg;cfg.history_duration=3.0;
    Camera camera(48,36,90,68,10);std::vector<double> clear(48*36,10),unknown(48*36,0);
    DepthObservation first(camera,clear,Eigen::Vector3d(.22,0,.02),fixedCameraRotation(),1,1);
    PaperGuidance slow(cfg);slow.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5);slow.ingestObservation(first);
    const auto crawl=slow.step(first,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),.02*Eigen::Vector3d::UnitX(),1,.02);
    check(crawl.accepted&&crawl.command.norm()>.019&&crawl.command.norm()<.021,"small operator magnitude lost to lookahead caching");
    PaperConfig short_cfg=cfg;short_cfg.minimum_lookahead=.10;
    short_cfg.motion.planning_horizon=3.;short_cfg.motion.delay=.20;
    short_cfg.motion.rollout_horizon=.20;short_cfg.motion.max_accel=1.2;
    short_cfg.motion.brake_accel=1.2;
    PaperGuidance short_start(short_cfg);
    short_start.setVerifiedSeed(Eigen::Vector3d::Zero(),short_start.envelopeRadius()+.20);
    DepthObservation no_view(camera,unknown,first.origin,first.rotation,1,90);
    auto short_move=short_start.step(no_view,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),2*Eigen::Vector3d::UnitX(),1,.02);
    check(short_move.accepted&&short_move.command.norm()>0&&short_move.command.norm()<.5,
        "short certified prefix could not start at braking-limited speed");
    check(!short_start.motionSafe(no_view,Eigen::Vector3d::Zero(),2*Eigen::Vector3d::UnitX(),Eigen::Vector3d::Zero(),1),
        "short horizon bypassed full measured-speed braking check");
    PaperGuidance planner(cfg);
    check(planner.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5),"seed installation failed");
    check(!planner.setVerifiedSeed(Eigen::Vector3d(10,0,0),2.5),"seed recentered after installation");
    planner.ingestObservation(first);
    auto start=planner.step(first,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1,.02);
    check(start.accepted&&start.command.norm()>.1,"verified initial prefix did not permit motion");
    DepthObservation next(camera,unknown,Eigen::Vector3d(3.22,0,.02),fixedCameraRotation(),2,2);
    auto continued=planner.step(next,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),2,.02);
    check(continued.accepted&&continued.command.norm()>.1,"real historical frustum did not bridge near-field blind region");
    check(!planner.envelopeKnown(next,Eigen::Vector3d(0,4,0),.55,2),"unseen side region became free");
    next.stamp=5;next.version=3;planner.ingestObservation(next);
    auto expired=planner.step(next,Eigen::Vector3d(3,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),5,.02);
    check(!expired.accepted&&expired.status=="UNOBSERVED_BODY_ENVELOPE","expired history authorized motion");
    DepthObservation half_a(camera,clear,Eigen::Vector3d(-2,0,0),fixedCameraRotation(),1,10,0);
    DepthObservation half_b=half_a;half_b.view=1;
    for(int y=0;y<36;++y)for(int x=0;x<48;++x) {
        if(x<22)half_a.depth[y*48+x]=0;
        if(x>25)half_b.depth[y*48+x]=0;
    }
    check(!observedEnvelope(half_a,Eigen::Vector3d::Zero(),.55)&&
        !observedEnvelope(half_b,Eigen::Vector3d::Zero(),.55),"union fixture has single-view coverage");
    DepthObservation united(camera,unknown,Eigen::Vector3d(-2,0,0),fixedCameraRotation(),1,11);
    united.supporting_views={std::make_shared<DepthObservation>(half_a),std::make_shared<DepthObservation>(half_b)};
    PaperGuidance union_planner(cfg);
    check(union_planner.envelopeKnown(united,Eigen::Vector3d::Zero(),.55,1),"complementary observed volumes not combined");
    for(int y=0;y<36;++y)for(int x=23;x<=24;++x){half_a.depth[y*48+x]=0;half_b.depth[y*48+x]=0;}
    united.supporting_views={std::make_shared<DepthObservation>(half_a),std::make_shared<DepthObservation>(half_b)};
    check(!union_planner.envelopeKnown(united,Eigen::Vector3d::Zero(),.55,1),"shared unknown hole filled by union");
    PaperGuidance clock_check(cfg);clock_check.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5);
    clock_check.ingestObservation(first);
    auto timed=first;timed.stamp=1.4;timed.version=2;clock_check.ingestObservation(timed);
    timed.view=1;timed.stamp=1.2;clock_check.ingestObservation(timed);
    check(clock_check.envelopeKnown(timed,Eigen::Vector3d::Zero(),.55,1.4),"out-of-order cameras erased seed");
    timed.view=0;timed.stamp=.5;clock_check.ingestObservation(timed);
    check(!clock_check.envelopeKnown(timed,Eigen::Vector3d::Zero(),.55,.5),"clock rollback kept initialization evidence");
    PaperGuidance chart_only(cfg);
    DepthObservation virtual_chart(camera,unknown,Eigen::Vector3d(3,0,0),fixedCameraRotation(),2,20);
    virtual_chart.supporting_views={std::make_shared<DepthObservation>(first)};
    auto chart_move=chart_only.step(virtual_chart,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),2,.02);
    check(chart_move.accepted,"direction chart failed to use real external evidence");
    virtual_chart.supporting_views.clear();virtual_chart.version++;
    auto no_evidence=chart_only.step(virtual_chart,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),2.1,.02);
    check(!no_evidence.accepted&&no_evidence.status=="UNOBSERVED_BODY_ENVELOPE","virtual chart invented geometric evidence");
    PaperConfig travel_config=cfg;travel_config.retain_verified_travel=true;
    PaperGuidance travel(travel_config);
    DepthObservation reached(camera,unknown,Eigen::Vector3d(3.22,0,.02),fixedCameraRotation(),2,30);
    reached.supporting_views={std::make_shared<DepthObservation>(first)};
    auto arrived=travel.step(reached,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),2,.02);
    check(arrived.accepted,"verified travel fixture failed to arrive");
    reached.supporting_views.clear();reached.stamp=7;reached.version++;
    check(travel.envelopeKnown(reached,Eigen::Vector3d(3,0,0),.55,7),"certified reached space was not retained");
    check(travel.directionalClearance(reached,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),7,3)<.2,
        "reached-space retention expanded into unobserved future space");
    // Handoff uses only geometry certified by an accepted motion, and only
    // after measuring arrival. Losing the originating frame must not erase it.
    PaperGuidance handoff(travel_config);
    DepthObservation seen=first;seen.stamp=2;seen.version=35;
    auto departing=handoff.step(seen,Eigen::Vector3d(3,0,0),Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),2,.02);
    check(departing.accepted,"motion handoff fixture did not depart");
    auto lost=seen;lost.depth=unknown;lost.stamp=2.2;lost.version++;
    const Eigen::Vector3d measured_arrival(3.25,0,0);
    check(!handoff.envelopeKnown(lost,measured_arrival,handoff.envelopeRadius(),2.2),"pending path became general free space before arrival");
    check(handoff.rememberVerifiedPosition(lost,measured_arrival,2.2),"accepted motion could not hand off its measured arrival");
    check(handoff.envelopeKnown(lost,measured_arrival,handoff.envelopeRadius(),2.2),"measured motion footprint not retained");
    check(!handoff.rememberVerifiedPosition(lost,Eigen::Vector3d(5,0,0),2.2),"motion handoff expanded beyond its certified prefix");
    PaperGuidance braking_handoff(travel_config);
    auto fast=braking_handoff.step(seen,Eigen::Vector3d(3,0,0),2*Eigen::Vector3d::UnitX(),2*Eigen::Vector3d::UnitX(),2,.02);
    check(fast.accepted,"fast braking handoff fixture did not depart");
    check(braking_handoff.rememberVerifiedPosition(lost,Eigen::Vector3d(4.4,0,0),2.7),"full braking tail was discarded from arrival certificate");
    // Regression: a corridor can certify the motion radius while lacking the
    // former extra 10 cm required solely by reached-space recording.
    PaperGuidance matched(travel_config);
    const double radius=matched.envelopeRadius();
    DepthObservation tight(camera,std::vector<double>(48*36,3+radius+.03-.22),
        first.origin,first.rotation,2,40);
    const Eigen::Vector3d reached_position(3,0,0);
    check(matched.envelopeKnown(tight,reached_position,radius,2),"tight fixture is not motion-certified");
    check(!matched.envelopeKnown(tight,reached_position,radius+.10,2),"tight fixture permits old oversized record");
    check(matched.rememberVerifiedPosition(tight,reached_position,2),"motion-certified position was not recorded");
    auto blind=tight;blind.depth=unknown;blind.supporting_views.clear();blind.stamp=20;blind.version++;
    check(matched.envelopeKnown(blind,reached_position,radius,20),"saved equal-radius footprint or age stability failed");
    check(matched.envelopeKnown(blind,reached_position,radius+.02,20),"partially observed margin was discarded");
    check(!matched.envelopeKnown(blind,reached_position,radius+.04,20),"partial margin grew beyond observed wall");
    check(!matched.rememberVerifiedPosition(blind,reached_position+Eigen::Vector3d(.2,0,0),20),"record expanded into unknown space");
    check(!matched.motionSafe(blind,reached_position,Eigen::Vector3d::UnitX(),Eigen::Vector3d::Zero(),20),"unknown braking corridor accepted");
    // The final measured position after a short release/braking displacement
    // must be retained even when it is less than the old 15 cm sample spacing.
    PaperGuidance released(travel_config);
    check(released.rememberVerifiedPosition(first,reached_position,1),"release fixture failed");
    const Eigen::Vector3d stopped_position=reached_position+Eigen::Vector3d(.08,0,0);
    check(released.rememberVerifiedPosition(first,stopped_position,1.1),"small braking displacement was not recorded");
    check(released.envelopeKnown(blind,stopped_position,radius,20),"final braking footprint lost when view expired");
    PaperGuidance breadcrumbs(travel_config);
    for(int k=0;k<=20;++k)
        check(breadcrumbs.rememberVerifiedPosition(first,Eigen::Vector3d(3+.05*k,0,0),1),"observed trail recording failed");
    check(breadcrumbs.recordedBalls()>=5,"latest-footprint replacement erased all spatial breadcrumbs");
    check(breadcrumbs.envelopeKnown(blind,Eigen::Vector3d(4,0,0),radius,20),"newest trail footprint was not retained");
    // New obstacle evidence must still revoke a retained measured footprint.
    auto intrusion=blind;intrusion.stamp=21;intrusion.version++;
    intrusion.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,
        intrusion.rotation.transpose()*(reached_position-intrusion.origin));
    matched.ingestObservation(intrusion);
    check(matched.revokedBalls()==1&&!matched.envelopeKnown(blind,reached_position,radius,21),"conflicting reached certificate was retained");
    PaperGuidance exact_hits(cfg);exact_hits.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5);
    auto pooled=first;pooled.depth[18*48+24]=1;
    pooled.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,Eigen::Vector3d(0,0,5));
    exact_hits.ingestObservation(pooled);
    check(exact_hits.envelopeKnown(pooled,Eigen::Vector3d::Zero(),.55,1),"pooled center fabricated a seed collision");
    pooled.stamp=1.1;pooled.version++;
    pooled.obstacle_points=std::make_shared<std::vector<Eigen::Vector3d>>(1,Eigen::Vector3d(0,0,1));
    exact_hits.ingestObservation(pooled);
    check(!exact_hits.envelopeKnown(pooled,Eigen::Vector3d::Zero(),.55,1.1),"actual native hit failed to revoke seed");
    PaperGuidance conflict(cfg);conflict.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5);conflict.ingestObservation(first);
    auto hit=first;hit.stamp=1.03;hit.version=2;hit.depth[18*48+24]=1;
    conflict.ingestObservation(hit);
    auto revoked=conflict.step(hit,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1.03,.02);
    check(!revoked.accepted,"new obstacle failed to revoke contradicted free evidence");
    check(!conflict.setVerifiedSeed(Eigen::Vector3d::Zero(),2.5),"revoked seed silently reissued");
    std::cout<<"verified geometry, fixed seed, history handoff, expiration and contradiction: PASS\n";
}
