#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <iostream>
using namespace pc_gvf::depth_angular;
std::vector<double> render(const Camera& c,const Eigen::Vector3d& origin,const Eigen::Vector3d& obstacle) {
    std::vector<double> d(c.rays().size(),c.maxDepth());
    for(std::size_t i=0;i<d.size();++i) {
        const Eigen::Vector3d ray=fixedCameraRotation()*c.rays()[i],delta=obstacle-origin;
        const double along=delta.dot(ray),disc=.2*.2-delta.squaredNorm()+along*along;
        if(disc>=0&&along>0){double r=along-std::sqrt(disc);if(r>0)d[i]=r*c.rays()[i].z();}
    }return d;
}
int main() {
    Camera c(48,36,90,68,10.);int collisions[2]={};double distance[2]={},control_distance[2]={};
    std::cout<<"trial,predictive,collision,min_surface_gap_m,progress_m\n";
    for(int trial=0;trial<7;++trial)for(int mode=0;mode<2;++mode) {
        PaperConfig cfg;cfg.dynamic_obstacles=mode;cfg.motion.body_radius=.48;cfg.motion.safety_margin=.02;
        cfg.depth_uncertainty=.03;cfg.uncertainty_rate=.1;cfg.observation_timeout=.3;cfg.motion.rollout_margin=.02;cfg.motion.delay=.2;
        cfg.motion.rollout_horizon=.2;cfg.motion.max_accel=1.2;cfg.motion.brake_accel=1.2;
        cfg.motion.velocity_tau=.22;cfg.certified_direct=true;cfg.adaptive_lookahead=true;
        PaperGuidance planner(cfg);Eigen::Vector3d p=Eigen::Vector3d::Zero(),v(1,0,0);
        double gap=1e9;bool collision=false;
        for(int k=0;k<100;++k) {
            const double t=k*.05;Eigen::Vector3d obstacle(1.5+.15*trial,2.-t,0);
            if(trial==5)obstacle=Eigen::Vector3d(100,100,100);
            if(trial==6)obstacle=Eigen::Vector3d(3,4.+t,0);
            Eigen::Vector3d origin=p-Eigen::Vector3d(2,0,0);
            DepthObservation o(c,render(c,origin,obstacle),origin,fixedCameraRotation(),t,k+1);
            planner.ingestObservation(o);
            auto r=planner.stepWithProposal(o,p,v,Eigen::Vector3d::UnitX(),Eigen::Vector3d::UnitX(),t,.05);
            const Eigen::Vector3d next_v=v+.05*clampNorm((r.command-v)/(r.command.norm()<1e-8?.05:.22),1.2);
            const Eigen::Vector3d next=p+.025*(v+next_v),next_obstacle=obstacle+Eigen::Vector3d(0,trial==6?.05:-.05,0);
            const Eigen::Vector3d a=p-obstacle,delta=next-next_obstacle-a;
            const double u=delta.squaredNorm()>1e-16?std::max(0.,std::min(1.,-a.dot(delta)/delta.squaredNorm())):0.;
            gap=std::min(gap,(a+u*delta).norm()-.68-1.2*.05*.05/8.);collision=collision||gap<0;
            p=next;v=next_v;
        }
        collisions[mode]+=collision;if(trial<5)distance[mode]+=p.x();else control_distance[mode]+=p.x();
        std::cout<<trial<<','<<mode<<','<<collision<<','<<gap<<','<<p.x()<<'\n';
    }
    std::cerr<<"collisions "<<collisions[0]<<" -> "<<collisions[1]<<"; progress "<<distance[0]/5<<" -> "<<distance[1]/5<<'\n';
    return collisions[1]==0&&collisions[0]>0&&control_distance[1]>=.95*control_distance[0]?0:1;
}
