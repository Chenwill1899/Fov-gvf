#include "pc_gvf/shared_obstacles.hpp"
#include <iostream>
#include <iomanip>
using namespace pc_gvf::depth_angular;
int main(){
 for(int which=0;which<2;++which){
  SharedObstaclePacket p;p.source="review";p.session="review";p.frame="world";p.sequence=1;p.stamp=which?1.:1e9;
  ObstacleTrack t;t.point.setZero();t.velocity.setZero();t.stamp=p.stamp+(which?.02:0.);t.radius=0.;t.observations=1;p.tracks.push_back(t);
  const double now=p.stamp,end=.04,radius=.58;
  const double exact=radius+.03+3.*end+.25*end*end;
  const double quick_time=now+end-p.stamp;
  const double quick=radius+.03+3.*quick_time+.25*quick_time*quick_time;
  const Eigen::Vector3d a(which?100.:(exact+quick)/2.,0,0);
  SharedObstacleMap map;
  const bool merged=map.merge(p,now,"world");
  std::cout<<std::setprecision(17)<<"case="<<which<<" age="<<now-t.stamp<<" exact_time="<<end<<" pre_fix_quick_time="<<quick_time<<" exact_bound="<<exact<<" pre_fix_quick_bound="<<quick<<" query_x="<<a.x()<<" merged="<<merged<<" exact_safe="<<obstacleSegmentsSafe(p.tracks,false,a,a,radius,now,0.,end)<<" broadphase_safe="<<map.segmentSafe(a,a,radius,now,0.,end)<<'\n';
 }
}
