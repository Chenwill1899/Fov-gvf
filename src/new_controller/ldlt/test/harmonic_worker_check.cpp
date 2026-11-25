#include "new_controller/harmonic_worker.hpp"
#include "new_controller/depth_observations.hpp"
#include <iostream>
using namespace ego4p0;
static void require(bool b,const char *why){if(!b)throw std::runtime_error(why);}
static double checksum(const HarmonicResult &r){double out=0;for(uint32_t i=0;i<r.fine.phi.size();++i)if(r.fine.boundary[i])out+=(i+1)*r.fine.phi[i];return out;}
int main(){try {
  const uint32_t w=96,h=72;std::array<CameraCalibration,4>cal;
  for(uint32_t c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);cal[c].K={.6*w,0,w*.5,0,.65*w,h*.5,0,0,1};cal[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};}
  FrameObservationBuilder builder;builder.configure(cal,w,h,1,1,16,4,5,false,true,true,true);PixelRayTables rays;rays.configure(cal,w,h);DepthFrame f;f.allocate(w*h);builder.allocate(f);for(auto &d:f.D)std::fill(d.begin(),d.end(),10.f);f.sequence=1;f.t_ns=2000000;builder.process(f,rays);
  HarmonicWorker worker;worker.start(cal,w,h,5,128);worker.submit_depth(f);
  HarmonicIntent intent;intent.q0=sphere_point(-20*direction_pi/180,0);intent.desired={sphere_point(20*direction_pi/180,0),1,true,true};worker.submit_intent(intent);
  const HarmonicResult *held=nullptr;const auto deadline=field_now_ns()+3000000000LL;
  while(field_now_ns()<deadline){held=worker.latest();if(held)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  require(held&&held->sequence==1&&held->fine.status==FieldStatus::Valid,"async initial job failed");const auto before=checksum(*held);const auto version=held->version;
  for(uint32_t i=2;i<=12;++i){f.sequence=i;f.t_ns=i*2000000;builder.process(f,rays);worker.submit_depth(f);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  require(checksum(*held)==before&&held->version==version,"completed result overwritten while reader holds it");
  const HarmonicResult *latest=nullptr;
  while(field_now_ns()<deadline){latest=worker.latest();if(latest&&latest->sequence==12)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  require(latest&&latest->sequence==12&&latest->version>version&&latest->reused,"latest-only data/cache publication failed");
  // Intent-only update schedules a new field without a new depth frame.
  intent.desired.d_desired=sphere_point(10*direction_pi/180,0);worker.submit_intent(intent);const auto old_version=latest->version;
  while(field_now_ns()<deadline){latest=worker.latest();if(latest&&latest->version>old_version)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  require(latest&&latest->version>old_version&&HarmonicPlanner::same_intent(latest->intent,intent),"intent-only invalidation failed");
  worker.stop();require(!worker.failed()&&worker.policy()==SCHED_OTHER&&worker.priority()==0,"field worker scheduling/error");
  std::cout<<"PASS: "<<worker.events().size()<<" completed async jobs, latest-only snapshots, held-slot immutability, intent-only refresh, SCHED_OTHER outside reader core, clean stop\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
