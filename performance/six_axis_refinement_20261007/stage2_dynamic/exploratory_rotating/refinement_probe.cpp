#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <ctime>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <limits>
using namespace pc_gvf::depth_angular;
static std::uint64_t digest=1469598103934665603ULL;
void mix(double d){std::uint64_t u;std::memcpy(&u,&d,sizeof u);digest=(digest^u)*1099511628211ULL;}
int main(){
  Camera c(8,8,90,68,50.);std::cout<<"case,repeat,frames,ingest_cpu_ms,track_digest,decision_digest,tracks\n";
  for(int count:{16,64,256,900})for(int rep=0;rep<5;++rep){
    DynamicObstacles model;digest=1469598103934665603ULL;std::uint64_t decision=0;double elapsed=0.;
    for(int k=0;k<160;++k){
      double t=.05*k;Eigen::Vector3d origin(.004*k,.1*std::sin(.01*k),0.);
      Eigen::Matrix3d rotation=Eigen::AngleAxisd(.01*k,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation();
      DepthObservation o(c,std::vector<double>(64,50.),origin,rotation,t,k+1,k%2);
      auto points=std::make_shared<std::vector<Eigen::Vector3d>>();points->reserve(count);
      for(int i=0;i<count;++i){
        const Eigen::Vector3d world(5.+.47*(i%30)+.1*std::sin(t),-7.+.47*(i/30)+.16*std::cos(.7*t),.15+.31*(i%3));
        auto p=(rotation.transpose()*(world-origin)).eval();
        // Retain all returns in the actual camera front half-space.
        if(p.z()>0.&&p.z()<49.99)points->push_back(p);
      }
      o.obstacle_points=points;const auto start=std::clock();model.ingest(o);elapsed+=double(std::clock()-start)/CLOCKS_PER_SEC;
      mix(model.overloaded());for(const auto& tr:model.tracks()){for(int j=0;j<3;++j){mix(tr.point[j]);mix(tr.velocity[j]);}mix(tr.stamp);mix(tr.radius);mix(tr.view);mix(tr.observations);}
      for(int j=0;j<12;++j)decision=decision*1099511628211ULL+model.segmentSafe(Eigen::Vector3d(1.,j-6.,0.),Eigen::Vector3d(8.,j-6.,0.),.58,t,0.,.8);
    }
    std::cout<<count<<','<<rep<<",160,"<<std::setprecision(12)<<elapsed*1000<<','<<digest<<','<<decision<<','<<model.tracks().size()<<'\n';
  }
  const auto z=Eigen::Vector3d::Zero().eval();double nan=std::numeric_limits<double>::quiet_NaN();ObstacleTrack bad;bad.stamp=1.;bad.point.x()=nan;
  int malformed_accept=obstacleSegmentsSafe({},false,z,z,nan,1.,0.,.2)+obstacleSegmentsSafe({},false,z,z,.58,1.,0.,nan)+obstacleSegmentsSafe({},false,z,z,.58,1.,-.1,.2)+obstacleSegmentsSafe({bad},false,z,z,.58,1.,0.,.2);
  std::cerr<<"malformed_accepted="<<malformed_accept<<'\n';
}
