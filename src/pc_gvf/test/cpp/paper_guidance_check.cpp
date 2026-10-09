#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace pc_gvf::depth_angular;
void require(bool value,const char* message) {if(!value){std::cerr<<message<<'\n';std::exit(1);}}
AngularField analytic(bool curved) {
    AngularField f;f.width=81;f.height=61;f.blocked.assign(81*61,0);f.potential.resize(81*61);
    for(int y=0;y<61;++y)for(int x=0;x<81;++x) {
        double X=x+30,Y=y-30;
        f.potential[y*81+x]=curved?(X*X-Y*Y)/10000.0:1.0-x/80.0;
    }
    f.valid=true;return f;
}
int main(){
    PaperConfig cfg;
    auto d=conservativeDepthResize({4,4,4,0.2,4,4,4,4,4,4,4,4,4,4,4,4},4,4,2,2,10);
    require(d[1]==0.2,"thin obstacle lost during resize");
    d=conservativeDepthResize({4,4,4,0,4,4,4,4,4,4,4,4,4,4,4,4},4,4,2,2,10);
    require(d[1]==0,"unknown depth treated as free");
    const double inf=std::numeric_limits<double>::infinity();
    require(conservativeDepthResize({inf},1,1,1,1,10)[0]==0,"unconfigured infinity became free");
    require(conservativeDepthResize({inf},1,1,1,1,10,true)[0]==10,"explicit no-return range contract ignored");
    for(double invalid:{0.0,-inf,std::numeric_limits<double>::quiet_NaN()})
        require(conservativeDepthResize({inf,invalid},2,1,1,1,10,true)[0]==0,"invalid depth hidden by no-return contract");
    for(bool curved:{false,true}) {
        auto f=analytic(curved);FlowBox box;
        require(box.initialize(f,Eigen::Vector2d(40,30),cfg),"analytic anchor failed");
        Eigen::Vector2d y(42,33),j;double chi;
        require(box.evaluate(f,y,cfg,&chi,&j),"flow-box coordinate/J failed");
        require(std::abs(j.dot(f.flow(y).normalized()))<0.015,"J u is not zero");
        double initial=std::abs(chi),last=initial;
        Eigen::Vector2d no_feedback_state=y;
        for(int k=0;k<100;++k)
            no_feedback_state+=0.005*f.flow(no_feedback_state).normalized();
        double no_feedback_error;
        require(box.coordinate(f,no_feedback_state,cfg,&no_feedback_error),"ablation section failed");
        require(std::abs(no_feedback_error)>initial*0.98,"flow alone unexpectedly recovered reference");
        for(int k=0;k<100;++k) {
            require(box.evaluate(f,y,cfg,&chi,&j),"tracking left regular chart");
            Eigen::Vector2d rate=0.5*f.flow(y).normalized()-j*chi/j.squaredNorm();
            y+=0.01*rate;
            double value;require(box.coordinate(f,y,cfg,&value),"section return failed");
            require(std::abs(value)<=last+0.002,"transverse error increased");last=std::abs(value);
        }
        require(last<initial*0.40,"transverse error did not decay at expected rate");
        std::cout<<"analytic curved="<<curved<<" initial="<<initial<<" final="<<last<<'\n';
    }
    // A regular point can be closer to a blocked boundary than the numerical
    // derivative stencil. Its anchor derivative is still known analytically.
    auto narrow=analytic(false);
    for(int y=0;y<narrow.height;++y)for(int x=0;x<=40;++x)narrow.blocked[y*narrow.width+x]=1;
    FlowBox narrow_box;Eigen::Vector2d nj;double nc=1;
    require(narrow_box.initialize(narrow,Eigen::Vector2d(40.51,30),cfg),"open narrow anchor rejected");
    require(narrow_box.evaluate(narrow,Eigen::Vector2d(40.51,30),cfg,&nc,&nj)&&nc==0&&
        std::abs(nj.dot(narrow.flow(Eigen::Vector2d(40.51,30))))<1e-12,"exact anchor derivative lost");
    require(!narrow_box.initialize(narrow,Eigen::Vector2d(40.5,30),cfg),"blocked-boundary anchor accepted");
    // A non-polynomial field exercises matching derivatives at patch seams.
    auto seam=analytic(false);
    for(int y=0;y<seam.height;++y)for(int x=0;x<seam.width;++x)
        seam.potential[y*seam.width+x]=std::sin(.13*x)*std::cos(.09*y)+.01*x;
    const double h=1e-4;
    const Eigen::Vector2d seam_point(40,30.3),axis(h,0);
    const auto left=seam.flow(seam_point-axis),at=seam.flow(seam_point),right=seam.flow(seam_point+axis);
    require((left-right).norm()<1e-5,"field not continuous at patch seam");
    require(((at-left)/h-(right-at)/h).norm()<1e-5,"field Jacobian not continuous at patch seam");
    Camera camera(48,36,90,68,10);
    std::vector<double> clear(48*36,10);
    DepthObservation obs(camera,clear,Eigen::Vector3d::Zero(),fixedCameraRotation(),1,1);
    PaperGuidance planner(cfg);
    require(planner.directionalClearance(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1)==0,"unobserved near field accepted");
    obs.depth.assign(48*36,0);
    require(planner.directionalClearance(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1)==0,"invalid depth accepted");
    require(!planner.motionSafe(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),Eigen::Vector3d::Zero(),1),"unsafe braking tail accepted");
    obs.depth=clear;
    require(planner.directionalClearance(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),2)==0,"stale observation accepted");
    require(!observedEnvelope(obs,Eigen::Vector3d::Zero(),.45),"unobserved body at camera accepted");
    obs.origin=Eigen::Vector3d(-2,0,0);
    require(observedEnvelope(obs,Eigen::Vector3d::Zero(),.45),"fully observed body rejected");
    require(planner.directionalClearance(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),1)>7,"observed forward segment rejected");
    require(!planner.motionSafe(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d(8,0,0),Eigen::Vector3d::Zero(),1),"braking beyond observed volume accepted");
    obs.depth[18*48+24]=0;
    require(!observedEnvelope(obs,Eigen::Vector3d::Zero(),.45),"unknown inside body footprint accepted");
    obs.depth=clear;
    obs.depth[23*48+29]=1.9;
    require(observedEnvelope(obs,Eigen::Vector3d::Zero(),.45),"off-sphere corner incorrectly blocks envelope");
    obs.depth=clear;
    require(!observedEnvelope(obs,Eigen::Vector3d(0,3,0),.45),"sphere crossing FOV plane accepted");
    auto start=std::chrono::steady_clock::now();
    auto result=planner.step(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,0.4,0.1),1,0.02);
    std::cout<<"first status="<<result.status<<" v="<<result.command.transpose()<<" residual="<<result.field_residual<<'\n';
    require(result.accepted&&result.command.norm()>0.01,"paper planner did not produce motion");
    require(result.field_residual<1e-7&&result.coupling_error<1e-10,"coarse-fine solve failed");
    // A Dirichlet source is not yet a regular tracking neighborhood. Advance
    // the measured direction through initialization before testing continuation.
    auto enter_tracking=[&](PaperGuidance& core,double& time,PaperResult current) {
        for(int k=0;k<80&&!current.tracking;++k) {
            time+=.02;obs.stamp=time;++obs.version;
            current=core.step(obs,Eigen::Vector3d::Zero(),current.command,Eigen::Vector3d(1,.4,.1),time,.02);
            require(current.accepted,"source initialization failed to reach regular field");
        }
        require(current.tracking,"source never entered regular tracking");
        time+=.06;obs.stamp=time;++obs.version;
        current=core.step(obs,Eigen::Vector3d::Zero(),current.command,Eigen::Vector3d(1,.4,.1),time,.02);
        require(current.tracking&&current.refreshed,"regular tracking was lost at refresh");
        return current;
    };
    require(!result.jump_valid,"initialization fabricated a coordinate jump");
    double test_time=1;result=enter_tracking(planner,test_time,result);
    const auto anchor=planner.box().anchor;
    auto held=planner.step(obs,Eigen::Vector3d::Zero(),result.command,Eigen::Vector3d(1,0.4,0.1),test_time+.02,0.02);
    require(!held.refreshed&&(planner.box().anchor-anchor).norm()<1e-10,"anchor changed inside field interval");
    obs.stamp=test_time+.06;++obs.version;
    auto continued=planner.step(obs,Eigen::Vector3d::Zero(),held.command,Eigen::Vector3d(1,0.4,0.1),test_time+.06,0.02);
    std::cout<<"refresh status="<<continued.status<<" continued="<<continued.continued<<" reset="<<continued.reset_reason<<'\n';
    require(continued.refreshed&&continued.accepted&&continued.continued,"field continuation failed");
    require(continued.jump_valid&&continued.jump<=continued.jump_bound+1e-8&&continued.jump_bound==2*cfg.section_radius,"valid section jump exceeded radius bound");
    if(result.tracking) require(held.mismatch_valid&&std::isfinite(held.execution_mismatch),"held-frame execution mismatch missing");
    obs.stamp=test_time+.12;++obs.version;
    auto changed=planner.step(obs,Eigen::Vector3d::Zero(),continued.command,Eigen::Vector3d(1,-0.3,0),test_time+.12,0.02);
    require(changed.reset_reason=="COMMAND_CHANGE","command change did not reset reference");
    PaperGuidance transferred(cfg);
    obs.stamp=2.0;obs.version=10;obs.rotation=fixedCameraRotation();
    auto old=transferred.step(obs,Eigen::Vector3d::Zero(),Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,.4,.1),2.0,.02);
    double transfer_time=2;old=enter_tracking(transferred,transfer_time,old);
    obs.stamp=transfer_time+.06;++obs.version;
    obs.rotation=Eigen::AngleAxisd(0.04,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation();
    auto moved=transferred.step(obs,Eigen::Vector3d::Zero(),old.command,Eigen::Vector3d(1,.4,.1),transfer_time+.06,.02);
    require(moved.continued&&moved.accepted,"world-direction chart transfer failed");
    auto blocked=analytic(false);blocked.blocked[30*blocked.width+42]=1;
    require(!blocked.segmentFree(Eigen::Vector2d(40,30),Eigen::Vector2d(44,30)),"angular segment crossed blocked cell");
    auto corner=analytic(false);corner.blocked[30*corner.width+41]=1;
    require(!corner.segmentFree(Eigen::Vector2d(40,30),Eigen::Vector2d(41,31)),"angular segment cut blocked cell corner");
    std::cout<<"paper numeric checks time_ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<'\n';
}
