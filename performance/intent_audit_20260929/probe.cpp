#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
using namespace pc_gvf::depth_angular;
int main() {
  PaperConfig cfg; cfg.field_interval=.10; cfg.chart_speed=16; cfg.adaptive_lookahead=true;
  cfg.motion.body_radius=.48; cfg.motion.safety_margin=.02; cfg.motion.delay=.20; cfg.motion.rollout_horizon=.20;
  Camera camera(24,18,90,68,20); PaperGuidance planner(cfg);
  planner.setVerifiedSeed(Eigen::Vector3d::Zero(),20);
  Eigen::Vector3d velocity(1,0,0);
  std::cout<<"step,intent_deg,refreshed,continued,reset,accepted,status,goal_deg,cmd_deg\n";
  for(int k=0;k<=21;++k) {
    double degrees=k<=20?k:35,angle=degrees*std::acos(-1)/180., now=1+.12*k;
    Eigen::Vector3d q(std::cos(angle),std::sin(angle),0);
    DepthObservation obs(camera,std::vector<double>(24*18,0),Eigen::Vector3d::Zero(),fixedCameraRotation(),now,k+1);
    auto out=planner.step(obs,Eigen::Vector3d::Zero(),velocity,q,now,.02);
    const auto* view=planner.observation(); Eigen::Vector3d goal=Eigen::Vector3d::Zero();
    if(view)goal=view->rotation*view->camera.rayFromPixel(planner.field().goal);
    std::cout<<k<<','<<degrees<<','<<out.refreshed<<','<<out.continued<<','<<out.reset_reason<<','<<out.accepted<<','<<out.status<<','<<std::atan2(goal.y(),goal.x())*180/std::acos(-1)<<','<<std::atan2(out.command.y(),out.command.x())*180/std::acos(-1)<<'\n';
    if(out.accepted)velocity=out.command;
  }
}
