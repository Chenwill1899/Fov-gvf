// Exercise the exact controller's pixel-footprint veto against its broad ROI.
#define main adaptive_controller_entry
#include "../../../src/auto_cruise/controller.cpp"
#undef main

int main(int argc,char **argv) try {
  if(argc!=2)return 2;
  uint32_t w,h;
  const auto cal=navigation_calibration(argv[1],w,h);
  const auto version=navigation_calibration_version(cal,w,h);
  NavigationPipeline pipeline;
  pipeline.configure(cal,w,h,version);
  CruiseDepthFrame obs;
  pipeline.allocate(obs);
  BodyState body;
  const auto &g=pipeline.builder().local_geometry();
  ImageDepthRig rig;rig.configure(g);
  const auto &transform=cal[0].T_BC;
  const Vec center{transform[3]+1.5*transform[2],transform[7]+1.5*transform[6],transform[11]+1.5*transform[10]};
  uint64_t sequence=0;
  auto check=[&](const char *name,int u,int v,float depth,TubeStatus broad,bool conflict){
    for(auto &image:obs.depth.D)std::fill(image.begin(),image.end(),4.5f);
    if(u>=0)obs.depth.D[0][size_t(v)*w+u]=depth;
    navigation_identity(obs,++sequence,version);
    body.t_ns=sequence*BaseConfig::period_ns;
    obs.observation_pose=body;
    obs.depth.t_ns=body.t_ns;
    obs.identity.simulation_stamp_ns=body.t_ns;
    obs.identity.camera_stamp_ns.fill(body.t_ns);
    NavigationState state;state.actual_valid=true;state.actual=body;state.current_simulation_ns=body.t_ns;
    const DesiredMotion intent{{1.,0.,0.},speed_limit,true,true};
    const auto &output=pipeline.process_if(obs,state,intent,false,false,[]{return NavigationPlan{false,false,nullptr};});
    if(!output.input_ready)throw std::runtime_error("invalid fixture frame");
    obs.prepare(g);
    const auto coarse=(*obs.raw)(center,.30);
    const bool precise=rig.surface_intersects(obs,g,center,.30);
    std::cout<<name<<" "<<tube_status_name(coarse)<<" "<<precise<<"\n";
    if(coarse!=broad||precise!=conflict)throw std::runtime_error(name);
  };
  check("clear",-1,0,0.,TubeStatus::ObservedClear,false);
  check("inside",160,120,1.5f,TubeStatus::PotentialObstacle,true);
  check("occluder_before_ball",160,120,.65f,TubeStatus::PotentialObstacle,false);
  check("rectangle_corner",188,148,1.5f,TubeStatus::PotentialObstacle,false);
  // Edge of the finite pixel footprint touches the ball although its central ray misses.
  check("pixel_edge",192,120,1.5f,TubeStatus::PotentialObstacle,true);
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
