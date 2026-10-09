#include "pc_gvf/spherical_memory.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
using namespace pc_gvf::depth_angular;
int main(){
 Camera c(32,24,90.,68.,10.);
 DepthObservation old(c,std::vector<double>(768,10.),Eigen::Vector3d(1e9,0,0),Eigen::Matrix3d::Identity(),1.,1);
 const double lower=1e9-2.,upper=std::nextafter(lower,INFINITY);
 const double z=.03*std::sqrt(2.)-(upper-1e9)+1e-9;
 auto points=std::make_shared<std::vector<Eigen::Vector3d>>();points->push_back({lower-1e9,0,z});points->push_back({upper-1e9,0,z});
 auto next=old;next.stamp=1.1;next.version=2;next.obstacle_points=points;
 SphericalMemory exact,fast;exact.ingest(old,false);fast.ingest(old,true);
 auto exact_ref=exact.evidence(1.)[0],fast_ref=fast.evidence(1.)[0];
 const bool contained=observedEnvelope(*exact_ref,old.origin+points->back(),.03);
 exact.ingest(next,false);fast.ingest(next,true);
 std::cout<<std::setprecision(17)<<"world_origin="<<old.origin.x()<<" lower="<<lower<<" upper="<<upper<<" hit_z="<<z<<" contained="<<contained<<" pointwise_revoked="<<exact_ref->revoked<<" block_revoked="<<fast_ref->revoked<<'\n';
}
