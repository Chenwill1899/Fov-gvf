#include "pc_gvf/omni_depth.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <iostream>
using namespace pc_gvf::depth_angular;
int main(){
 const double pi=std::acos(-1.);Camera c(32,24,90,68,10.);
 std::vector<std::shared_ptr<const DepthObservation>> views;
 for(int j=0;j<4;++j){const auto rot=Eigen::AngleAxisd(j*pi/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();views.push_back(std::make_shared<DepthObservation>(c,std::vector<double>(768,10.),.3*rot*Eigen::Vector3d::UnitX(),rot*fixedCameraRotation(),1.,1,j));}
 std::cout<<std::setprecision(17)<<"mode,frame,boundary_deg,selected_yaw_deg,intent_error_rad,certificates\n";
 for(int mode=0;mode<2;++mode){OmniDepthConfig cfg;cfg.horizon=4.;cfg.continuous_refinement=mode;Eigen::Vector3d previous=Eigen::Vector3d::Zero();
  for(int frame=0;frame<40;++frame){const double boundary=8.1+.3*frame;int calls=0;
   const auto allowed=[&](const Eigen::Vector3d& d){return std::atan2(d.y(),d.x())*180/pi>=boundary&&std::abs(d.z())<.1;};
   auto proof=[&](const Eigen::Vector3d& d){++calls;return allowed(d);};
   const auto result=omniDepthProposal(views,Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),previous,1.05,.5,cfg,proof);
   if(!result.valid||!allowed(result.direction)||calls!=result.certificates||calls>64)return 1;
   std::cout<<mode<<','<<frame<<','<<boundary<<','<<std::atan2(result.direction.y(),result.direction.x())*180/pi<<','<<std::acos(std::max(-1.,std::min(1.,result.direction.x())))<<','<<calls<<'\n';previous=result.direction;
  }
 }
}
