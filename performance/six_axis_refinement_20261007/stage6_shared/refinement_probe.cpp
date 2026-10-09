#include "pc_gvf/shared_obstacles.hpp"
#include <random>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cstdint>
using namespace pc_gvf::depth_angular;
SharedObstaclePacket packet(int robot,int count,double offset){SharedObstaclePacket p;p.source="peer_"+std::to_string(robot);p.session="boot";p.frame="world";p.sequence=1;p.stamp=1.;for(int i=0;i<count;++i){ObstacleTrack t;t.point=Eigen::Vector3d(offset+.3*(i%16),offset+.3*((i/16)%16),.3*(i/256));t.velocity=Eigen::Vector3d(.1*(i%3-1),.1*(i%5-2),0);t.stamp=1.-.001*(i%50);t.radius=.26;t.observations=i%5?3:1;p.tracks.push_back(t);}return p;}
int main(){
  std::cout<<"case,repeat,queries,cpu_ms,decisions_digest,reference_mismatches\n";
  for(int count:{32,256,1024,4096})for(int rep=0;rep<5;++rep){
    SharedObstacleMap map;std::vector<SharedObstaclePacket> all;for(int r=0;r<8;++r){all.push_back(packet(r,count,12.+r));if(!map.merge(all.back(),1.,"world"))return 2;}
    std::uint64_t digest=0;const auto start=std::clock();for(int k=0;k<3000;++k)digest=digest*1099511628211ULL+map.segmentSafe(Eigen::Vector3d(0.,.01*(k%30),0.),Eigen::Vector3d(1.,.01*(k%30),0.),.58,1.2,0.,.8);
    const auto elapsed=double(std::clock()-start)/CLOCKS_PER_SEC;
    std::mt19937 rng(121);std::uniform_real_distribution<double> pos(-10.,30.),dt(0.,.49),horizon(0.,2.);int mismatch=0;
    for(int k=0;k<1500;++k){const Eigen::Vector3d a(pos(rng),pos(rng),pos(rng)*.1),b(pos(rng),pos(rng),pos(rng)*.1);double now=1.+dt(rng),end=horizon(rng);bool reference=true;for(const auto& p:all)reference=reference&&obstacleSegmentsSafe(p.tracks,p.overloaded,a,b,.58,now,0.,end);if(map.segmentSafe(a,b,.58,now,0.,end)!=reference)++mismatch;}
    std::cout<<count<<','<<rep<<",3000,"<<std::setprecision(12)<<elapsed*1000<<','<<digest<<','<<mismatch<<'\n';
  }
  int capacity_unsafe=0,invalid_disabled=0,expiry_rejected=0,near_missed=0;
  for(int trial=0;trial<32;++trial){SharedObstacleMap map;for(int r=0;r<8;++r)if(!map.merge(packet(r,1,20.+r),1.,"world"))return 3;auto danger=packet(8,1,0.);danger.tracks[0].point=Eigen::Vector3d(.5+.01*trial,0.,0.);if(map.merge(danger,1.,"world"))return 4;capacity_unsafe+=map.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.,0.,.5);expiry_rejected+=!map.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.501,0.,.5);
    SharedObstacleMap valid;for(int r=0;r<8;++r)valid.merge(packet(r,1,20.+r),1.,"world");danger.frame="bad";valid.merge(danger,1.,"world");invalid_disabled+=!valid.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.,0.,.5);
    SharedObstacleMap near;danger.frame="world";near.merge(danger,1.,"world");near_missed+=near.segmentSafe(Eigen::Vector3d::Zero(),Eigen::Vector3d::UnitX(),.58,1.,0.,.5);
  }
  std::cerr<<"overcapacity_hazards_accepted="<<capacity_unsafe<<" invalid_packets_disabled_clear_map="<<invalid_disabled<<" expired_capacity_rejected="<<expiry_rejected<<" near_hazards_missed="<<near_missed<<'\n';
}
