#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>
#include <stdexcept>
using namespace pc_gvf::depth_angular;
int main() {
    AngularField obsolete;obsolete.width=2;obsolete.height=2;obsolete.blocked.assign(4,1);
    obsolete.valid=true;obsolete.potential.assign(4,1.);
    if(solveHarmonicField(obsolete,std::vector<double>(4,0.)).valid)return 6;
    const int w=48,h=36;Camera camera(w,h,90,68,10.);
    AngularField grid;grid.width=w;grid.height=h;grid.blocked.assign(w*h,0);
    std::vector<double> boundary(w*h,std::numeric_limits<double>::quiet_NaN());
    for(int y=0;y<h;++y){boundary[y*w]=1;boundary[y*w+w-1]=0;}
    auto previous=solveHarmonicField(grid,boundary);
    DepthObservation old(camera,{},Eigen::Vector3d::Zero(),fixedCameraRotation(),0.);
    int cold_work=0,warm_work=0;double cold_ms=0,warm_ms=0,error=0;
    for(int k=0;k<100;++k) {
        auto pose=old;
        // Alternating stationary and small moving-camera frames; the new
        // occupancy and Dirichlet values always remain authoritative.
        if(k%4==0)pose.origin.y()+=.002;
        if(k%7==0)pose.rotation=Eigen::AngleAxisd(.0005,Eigen::Vector3d::UnitZ()).toRotationMatrix()*pose.rotation;
        if(k==50)for(int y=8;y<18;++y)grid.blocked[y*w+23]=1;
        auto start=std::chrono::steady_clock::now();auto cold=solveHarmonicField(grid,boundary);
        cold_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        start=std::chrono::steady_clock::now();auto warm=solveHarmonicField(grid,boundary,reprojectPotential(previous,old,grid,pose,4.));
        warm_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(!cold.valid||!warm.valid||warm.residual>1e-7)return 2;
        for(int i=0;i<w*h;++i)if(!grid.blocked[i])error=std::max(error,std::abs(cold.potential[i]-warm.potential[i]));
        cold_work+=cold.iterations;warm_work+=warm.iterations;previous=warm;old=pose;
    }
    std::cout<<"frames,cold_iterations,incremental_iterations,max_potential_error,cold_ms,incremental_ms\n100,"
        <<cold_work<<','<<warm_work<<','<<error<<','<<cold_ms<<','<<warm_ms<<'\n';
    // A field is only a directional proposal; the current frame remains the
    // authority for geometry. Exercise reuse after independent ego motion,
    // then remove current evidence and ensure the old field grants no passage.
    PaperConfig cfg;cfg.incremental_field=true;cfg.motion.body_radius=.48;
    cfg.motion.safety_margin=.04;cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.03;
    cfg.motion.max_accel=cfg.motion.brake_accel=1.2;cfg.smooth_speed=true;
    cfg.response_preview=true;cfg.intent_guard=true;
    PaperGuidance core(cfg);
    DepthObservation visible(camera,std::vector<double>(w*h,10.),Eigen::Vector3d(-2,0,0),fixedCameraRotation(),1.,1);
    const Eigen::Vector3d intent(1.,.4,.1),velocity(.1,0,0);
    auto first=core.step(visible,Eigen::Vector3d::Zero(),velocity,intent,1.,.02);
    if(!first.accepted||!core.field().valid){std::cerr<<"initial field: "<<first.status<<" valid="<<core.field().valid<<"\n";return 3;}
    if(core.incrementalGoalProposal(Eigen::Vector3d::Zero(),intent,1.1).norm()<1e-5)return 7;
    if(core.incrementalGoalProposal(Eigen::Vector3d::Zero(),-intent,1.1).norm()>1e-5)return 8;
    if(core.incrementalGoalProposal(Eigen::Vector3d::Zero(),intent,1.6).norm()>1e-5)return 9;
    core.confirmPublishedCommand(Eigen::Vector3d::Zero());
    visible.stamp=1.1;++visible.version;
    visible.rotation=Eigen::AngleAxisd(.02,Eigen::Vector3d::UnitZ()).toRotationMatrix()*visible.rotation;
    const Eigen::Vector3d moved(.02,.01,0);
    auto reused=core.stepWithProposal(visible,moved,Eigen::Vector3d::Zero(),intent,Eigen::Vector3d::Zero(),1.1,.02);
    if(!reused.accepted||reused.status!="INCREMENTAL_GOAL_CERTIFIED"||
       !core.motionSafe(visible,moved,Eigen::Vector3d::Zero(),reused.command,1.1)){std::cerr<<"reused: "<<reused.status<<" command="<<reused.command.transpose()<<" max_clearance="<<core.maximumClearance()<<"\n";return 4;}
    visible.depth.assign(w*h,0.);visible.stamp=1.2;++visible.version;
    auto denied=core.stepWithProposal(visible,moved,Eigen::Vector3d::Zero(),intent,Eigen::Vector3d::Zero(),1.2,.02);
    if(denied.accepted||denied.command.norm()>1e-8)return 5;
    // New Dirichlet data and occlusions remain authoritative even when an
    // exactly unchanged camera permits direct transfer of the old potential.
    double changed_error=0.;
    for(int k=0;k<40;++k) {
        auto changed=grid;auto fixed=boundary;
        for(int y=0;y<h;++y){fixed[y*w]=k<20?.7+.005*k:.007+.00001*k;fixed[y*w+w-1]=k<20?.1:.001;}
        for(int y=5;y<25;++y)changed.blocked[y*w+18+(k%3)]=1;
        auto pose=old;
        if(k%2){pose.origin+=Eigen::Vector3d(.01,-.02,.003);
            pose.rotation=Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ()).toRotationMatrix()*pose.rotation;}
        const auto cold=solveHarmonicField(changed,fixed);
        auto guess=reprojectPotential(previous,old,changed,pose,4.);
        if(k%5==0)guess[w+2]=std::numeric_limits<double>::quiet_NaN();
        const auto warm=solveHarmonicField(changed,fixed,guess);
        if(!cold.valid||!warm.valid||warm.residual>1e-7)return 10;
        for(int i=0;i<w*h;++i)if(!changed.blocked[i]){
            changed_error=std::max(changed_error,std::abs(cold.potential[i]-warm.potential[i]));
            if(std::isfinite(fixed[i])&&warm.potential[i]!=fixed[i])return 11;
        }
    }
    if(changed_error>1e-6)return 12;
    std::cout<<"changed_boundary_frames,max_error\n40,"<<changed_error<<'\n';
    std::cout<<"reused_goal_with_current_motion_proof,unknown_current_frame_rejected\n1,1\n";
    return warm_work<cold_work&&error<1e-6?0:1;
}
