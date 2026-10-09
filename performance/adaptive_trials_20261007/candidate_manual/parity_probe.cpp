#include "pc_gvf/omni_depth.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
namespace pc_gvf { namespace depth_angular {
OmniDepthResult baselineOmniDepthProposal(const std::vector<std::shared_ptr<const DepthObservation>>&,
 const Eigen::Vector3d&,const Eigen::Vector3d&,const Eigen::Vector3d&,double,double,
 const OmniDepthConfig&,const std::function<bool(const Eigen::Vector3d&)>&);
} }
using namespace pc_gvf::depth_angular;
int main(){
 const double pi=std::acos(-1.);Camera c(32,24,90,68,10.);
 for(int k=0;k<96;++k){
  std::vector<std::shared_ptr<const DepthObservation>> views;
  for(int j=0;j<4;++j){const auto rot=Eigen::AngleAxisd(.03*k+j*pi/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
   auto view=std::make_shared<DepthObservation>(c,std::vector<double>(768,10.),.3*rot*Eigen::Vector3d::UnitX(),rot*fixedCameraRotation(),1.,k+1,j);
   for(int q=0;q<12;++q)view->depth[(k*31+j*17+q*53)%768]=(k%5==0)?0.:1.8+.1*q;
   if(k%11!=0||j!=0)views.push_back(view);
  }
  const double a=.087*k;const Eigen::Vector3d intent(std::cos(a),std::sin(a),.05*std::sin(k));
  const Eigen::Vector3d previous(std::cos(a+.2),std::sin(a+.2),0.);
  OmniDepthConfig cfg;cfg.horizon=3.;cfg.retain_previous=k%3==0;cfg.include_atlas=k%2;
  auto proof=[&](const Eigen::Vector3d& d){return d.dot(intent.normalized())>.6&&d.y()>.05*std::sin(k);};
  const auto old=baselineOmniDepthProposal(views,Eigen::Vector3d(.03,0,0),intent,previous,1.05,.5,cfg,proof);
  const auto current=omniDepthProposal(views,Eigen::Vector3d(.03,0,0),intent,previous,1.05,.5,cfg,proof);
  if(old.valid!=current.valid||old.direction!=current.direction||old.clearance!=current.clearance||old.observed!=current.observed||old.certificates!=current.certificates||old.retained_previous!=current.retained_previous||old.used_short_horizon!=current.used_short_horizon){std::cerr<<"mismatch "<<k<<'\n';return 1;}
 }
 std::cout<<"disabled_mode_frozen_parity_cases,passed\n96,96\n";
}
