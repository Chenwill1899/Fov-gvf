// Paired timing of the real R05 queries which required a pixel-level veto.
// Depth preprocessing is deliberately outside the two compared kernels.
#define main archived_cruise_entry
#include "before/controller.cpp"
#undef main
#include "../../../src/auto_cruise/image_depth_clearance.hpp"
#include <iomanip>
#include <map>

struct Sample {uint64_t sequence;BodyState pose;Direction3 world;double radius;};
int main(int argc,char **argv) try {
  if(argc!=3 || getuid()==0)throw std::runtime_error("expected regular user, calibration and capture directory");
  std::ifstream stream(std::filesystem::path(argv[2])/"queries.txt");
  std::map<uint64_t,std::vector<Sample>> groups;
  Sample sample;
  while(stream>>sample.sequence){
    for(auto &v:sample.pose.p_W)stream>>v;
    for(auto &v:sample.pose.R_WB)stream>>v;
    for(auto &v:sample.world)stream>>v;
    if(!(stream>>sample.radius))throw std::runtime_error("truncated record");
    groups[sample.sequence].push_back(sample);
  }
  if(groups.empty())throw std::runtime_error("no captured refinements");
  cpu_set_t mask;CPU_ZERO(&mask);for(unsigned i=0;i<5;++i)CPU_SET(i,&mask);
  if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("cannot bind CPUs");
  uint32_t w,h;const auto cal=navigation_calibration(argv[1],w,h);
  const auto version=navigation_calibration_version(cal,w,h);
  NavigationPipeline pipe;pipe.configure(cal,w,h,version);pipe.enable_fifo5();
  pipe.builder().refresh_worker_schedulers();
  sched_param sched{};
  if(sched_getscheduler(0)!=SCHED_FIFO || sched_getparam(0,&sched) || sched.sched_priority!=5 ||
     sched_getaffinity(0,sizeof(mask),&mask) || CPU_COUNT(&mask)!=1 || !CPU_ISSET(0,&mask))
    throw std::runtime_error("FIFO5 coordinator mismatch");
  for(unsigned i=0;i<4;++i)if(pipe.builder().worker_policies()[i]!=SCHED_FIFO ||
      pipe.builder().worker_priorities()[i]!=5 || pipe.builder().worker_cpus()[i]!=int(i+1))
    throw std::runtime_error("FIFO5 worker mismatch");
  std::cerr<<"five FIFO5 threads verified, CPU0..4\n";
  const auto &geometry=pipe.builder().local_geometry();ImageDepthRig rig;rig.configure(geometry);
  CruiseDepthFrame f;pipe.allocate(f);
  std::cout<<"sequence,query,round,image_first,old_raw_ns,image_raw_ns,old_surface_ns,image_surface_ns\n";
  for(const auto &[sequence,samples]:groups){
    std::ifstream input(std::filesystem::path(argv[2])/(std::to_string(sequence)+".f32"),std::ios::binary);
    for(auto &image:f.depth.D)if(!input.read(reinterpret_cast<char*>(image.data()),image.size()*sizeof(float)))
      throw std::runtime_error("truncated depth frame");
    const auto t=sequence*BaseConfig::period_ns;
    navigation_identity(f,sequence,version);f.observation_pose=samples[0].pose;
    f.observation_pose.t_ns=f.depth.t_ns=t;f.identity.simulation_stamp_ns=t;f.identity.camera_stamp_ns.fill(t);
    f.depth.local.ready=f.depth.observations.ready=false;
    f.prepare(geometry);
    NavigationState state;state.actual=f.observation_pose;state.actual_valid=true;state.current_simulation_ns=t;
    const DesiredMotion intent{{1,0,0},12.,true,true};
    if(!pipe.process_if(f,state,intent,false,false,[]{return NavigationPlan{false,false,nullptr};}).input_ready)
      throw std::runtime_error("frame rejected");
    for(unsigned n=0;n<samples.size();++n){
      const auto &q=samples[n];
      for(int round=-2;round<20;++round){
        bool hit[2]{};TubeStatus status[2]{};int64_t raw[2]{},veto[2]{};
        for(int j=0;j<2;++j){
          const int image=(j+round+2+n)%2;
          auto a=field_now_ns();
          status[image]=image?(*f.raw)(q.world,q.radius):
              RawDepthBallOracle(f.depth,geometry,f.observation_pose)(q.world,q.radius);
          auto b=field_now_ns();
          hit[image]=image?rig.surface_intersects(f,geometry,q.world,q.radius):
                            measured_surface_intersects(f,geometry,q.world,q.radius);
          raw[image]=b-a;veto[image]=field_now_ns()-b;
        }
        if(status[0]!=TubeStatus::PotentialObstacle || status[0]!=status[1] || hit[0]!=hit[1])
          throw std::runtime_error("captured query mismatch");
        if(round>=0)std::cout<<sequence<<','<<n<<','<<round<<','<<(round+n)%2<<','<<raw[0]<<','<<raw[1]<<','<<veto[0]<<','<<veto[1]<<'\n';
      }
    }
    usleep(1000);
  }
  return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 2;}
