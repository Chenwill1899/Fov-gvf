#include "pc_gvf/paper_guidance.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <cstdint>
using namespace pc_gvf::depth_angular;
#include "tube_functions.inc"
int main(){
  std::uint64_t queries=0,mismatches=0;auto compare=[&](const CertifiedTube& t,const Eigen::Vector3d& p,double r){++queries;bool a=frozen::tubeContains(t,p,r),b=candidate::tubeContains(t,p,r);if(a!=b){++mismatches;if(mismatches<5)std::cerr<<"mismatch "<<t.a.transpose()<<" : "<<t.b.transpose()<<" p "<<p.transpose()<<" radius "<<r<<" tube "<<t.radius<<'\n';}};
  std::mt19937_64 rng(91287421);std::uniform_real_distribution<double> unit(-1.,1.),positive(0.,1.);
  for(double magnitude:{0.,1e-12,1.,100.,1e6,1e12,1e100,1e300})for(int n=0;n<60000;++n){
    Eigen::Vector3d a=Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*magnitude;
    Eigen::Vector3d b=a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(n%4?.5:1e-12);
    double tr=.58+positive(rng),qr=(n%13==0?1.1:.99)*tr*positive(rng);
    CertifiedTube tube{a,b,tr};Eigen::Vector3d p=a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(n%3?3.:.3);
    compare(tube,p,qr);
    for(int axis=0;axis<3;++axis){p=a;p[axis]=std::min(a[axis],b[axis])-(tr-qr);compare(tube,p,qr);p[axis]=std::nextafter(p[axis],-std::numeric_limits<double>::infinity());compare(tube,p,qr);p[axis]=std::nextafter(p[axis],std::numeric_limits<double>::infinity());compare(tube,p,qr);}
  }
  const double bad[]={0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::max()};
  for(double a:bad)for(double b:bad)for(double p:bad)for(double r:bad)for(double q:bad)compare(CertifiedTube{Eigen::Vector3d(a,0,0),Eigen::Vector3d(b,0,0),r},Eigen::Vector3d(p,0,0),q);
  std::cout<<"queries,mismatches\n"<<queries<<','<<mismatches<<'\n';return mismatches?1:0;
}
