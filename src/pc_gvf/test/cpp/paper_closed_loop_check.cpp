#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
using namespace pc_gvf::depth_angular;
struct Sphere {Eigen::Vector3d center;double radius;};
std::vector<double> render(const Camera& camera,const Eigen::Vector3d& origin,
    const Eigen::Matrix3d& rotation,const std::vector<Sphere>& obstacles) {
    std::vector<double> depth(camera.width()*camera.height(),camera.maxDepth());
    for(int y=0;y<camera.height();++y)for(int x=0;x<camera.width();++x){
        Eigen::Vector3d ray=rotation*camera.ray(x,y);double range=camera.maxDepth()/camera.ray(x,y).z();
        for(const auto& sphere:obstacles){Eigen::Vector3d delta=sphere.center-origin;
            double along=delta.dot(ray),disc=sphere.radius*sphere.radius-delta.squaredNorm()+along*along;
            if(disc>=0&&along>0){double hit=along-std::sqrt(disc);if(hit>0)range=std::min(range,hit);}}
        depth[y*camera.width()+x]=range*camera.ray(x,y).z();
    }return depth;
}
int main(int argc,char** argv){
    std::ofstream csv;if(argc>1){csv.open(argv[1]);csv<<"case,t,x,y,z,vx,vy,vz,chi,jump,tracking,continued,reset,status,compute_ms,clearance,state_u,state_v,source_u,source_v,goal_u,goal_v\n";}
    bool passed=true;
    for(std::string name:{"clear","pillar","disturbance","obstacle_shift","feasible_shift","command_change","no_feedback","no_continuation","single_field"}) {
        PaperConfig cfg;cfg.motion.safety_margin=.20;cfg.motion.max_accel=1.2;cfg.motion.planning_horizon=3.0;
        cfg.feedback=name!="no_feedback";cfg.continuation=name!="no_continuation";cfg.coarse_fine=name!="single_field";
        PaperGuidance planner(cfg);Camera camera(48,36,90,68,20);
        Eigen::Vector3d position=Eigen::Vector3d::Zero(),velocity=Eigen::Vector3d::Zero();
        std::vector<Sphere> spheres;if(name!="clear"&&name!="command_change")spheres.push_back({Eigen::Vector3d(3.5,0,0),.6});
        int accepted=0,tracking=0,continued=0,resets=0,collisions=0;double minclear=1e9,maxchi=0;
        bool previous_tracking=false,disturbance_applied=false;
        std::vector<double> times;
        for(int k=0;k<500;++k){double now=k*.02;
            Eigen::Vector3d intent(1,0,0);
            if(name=="command_change"&&now>=4)intent=Eigen::Vector3d(0,1,0);
            if(name=="obstacle_shift"&&k==150)spheres[0].center.y()=.6;
            if(name=="feasible_shift"&&k==150)spheres[0].center.y()=-std::copysign(.6,position.y());
            if((name=="disturbance"||name=="no_feedback"||name=="no_continuation"||name=="single_field")&&now>=.8&&previous_tracking&&!disturbance_applied){velocity.z()+=.15;disturbance_applied=true;}
            double view_yaw=std::atan2(intent.y(),intent.x());
            Eigen::Matrix3d rotation=Eigen::AngleAxisd(view_yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation();
            // Fully observed external-camera fixture: isolate the paper method
            // from the onboard near-field blind volume, which has its own test.
            Eigen::Vector3d observer=position-2.0*intent.normalized();
            auto depth=render(camera,observer,rotation,spheres);
            DepthObservation obs(camera,depth,observer,rotation,now,k+1,view_yaw>1?1:0);
            auto begin=std::chrono::steady_clock::now();auto out=planner.step(obs,position,velocity,intent,now,.02);
            double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();times.push_back(ms);
            previous_tracking=out.tracking;
            accepted+=out.accepted;tracking+=out.tracking;continued+=out.continued;resets+=!out.reset_reason.empty();maxchi=std::max(maxchi,std::abs(out.chi));
            velocity+=.02*clampNorm((out.command-velocity)/(out.command.norm()<1e-8?.02:.22),1.2);position+=.02*velocity;
            for(const auto& sphere:spheres){double clearance=(position-sphere.center).norm()-sphere.radius-.25;
                minclear=std::min(minclear,clearance);if(clearance<0)++collisions;}
            if(csv)csv<<name<<','<<now<<','<<position.x()<<','<<position.y()<<','<<position.z()<<','<<velocity.x()<<','<<velocity.y()<<','<<velocity.z()<<','<<out.chi<<','<<out.jump<<','<<out.tracking<<','<<out.continued<<','<<!out.reset_reason.empty()<<','<<out.status<<','<<ms<<','<<minclear<<','<<out.state.x()<<','<<out.state.y()<<','<<planner.field().source.x()<<','<<planner.field().source.y()<<','<<planner.field().goal.x()<<','<<planner.field().goal.y()<<'\n';
        }
        std::sort(times.begin(),times.end());double mean=0;for(double t:times)mean+=t/times.size();
        std::cout<<name<<" final="<<position.transpose()<<" accepted="<<accepted<<" tracking="<<tracking<<" continued="<<continued<<" resets="<<resets<<" collisions="<<collisions<<" minclear="<<minclear<<" maxchi="<<maxchi<<" ms(mean/p95/max)="<<mean<<'/'<<times[static_cast<std::size_t>(.95*times.size())]<<'/'<<times.back()<<'\n';
        if(collisions||position.norm()<1.0||accepted<100)passed=false;
        if(name!="obstacle_shift"&&name!="command_change"&&position.x()<4.5)passed=false;
        if((name=="pillar"||name=="disturbance"||name=="feasible_shift")&&tracking<30)passed=false;
        // Completing a bypass must recover operator direction, not merely
        // accumulate distance along the old avoidance direction.
        if(name=="pillar"||name=="disturbance"||name=="feasible_shift"||name=="command_change") {
            const Eigen::Vector3d wanted=name=="command_change"?Eigen::Vector3d::UnitY():Eigen::Vector3d::UnitX();
            const double error=velocity.norm()>1e-6?std::acos(std::max(-1.0,std::min(1.0,velocity.normalized().dot(wanted))))*180/3.141592653589793:180;
            std::cout<<name<<" final_direction_error_deg="<<error<<'\n';
            if(error>10||(name=="command_change"&&position.y()<3))passed=false;
        }
    }
    return passed?0:1;
}
