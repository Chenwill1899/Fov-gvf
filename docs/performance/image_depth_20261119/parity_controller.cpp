// Functional instrument only. Production FIFO times use the uninstrumented ELF.
#include "../../../src/auto_cruise/image_depth_clearance.hpp"
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <fstream>
#include <random>

using namespace ego4p0;
bool archived_surface(const NavigationObservation &,const LocalGeometryTables &,const Direction3 &,double);
namespace {
struct Counts {
  std::array<uint64_t,4> broad{};
  std::array<uint64_t,2> surface{};
  size_t frames=0,image_misses=0,off_image=0;
  double max_miss_error=0.;
  void report() const {
    std::fprintf(stderr,"BALL_PARITY frames=%zu unknown=%lu clear=%lu obstacle=%lu outside=%lu surface_free=%lu surface_hit=%lu image_misses=%zu off_image=%zu max_miss_error=%.12g\n",
      frames,broad[0],broad[1],broad[2],broad[3],surface[0],surface[1],image_misses,off_image,max_miss_error);
  }
} counts;
std::filesystem::path capture_dir;
void save_refinement(const NavigationObservation &f,const Direction3 &world,double radius){
  if(capture_dir.empty())return;
  std::filesystem::create_directories(capture_dir);
  const auto file=capture_dir/(std::to_string(f.depth.sequence)+".f32");
  if(!std::filesystem::exists(file)){
    std::ofstream bytes(file,std::ios::binary);
    for(const auto &image:f.depth.D)bytes.write(reinterpret_cast<const char*>(image.data()),image.size()*sizeof(float));
    if(!bytes)throw std::runtime_error("cannot save refinement frame");
  }
  std::ofstream queries(capture_dir/"queries.txt",std::ios::app);
  queries<<std::setprecision(17)<<f.depth.sequence;
  for(double x:f.observation_pose.p_W)queries<<' '<<x;
  for(double x:f.observation_pose.R_WB)queries<<' '<<x;
  for(double x:world)queries<<' '<<x;
  queries<<' '<<radius<<"\n";
  if(!queries)throw std::runtime_error("cannot save refinement query");
}
struct CheckedRaw {
  const CruiseDepthFrame &frame;
  const LocalGeometryTables &geometry;
  TubeStatus operator()(const Direction3 &world,double radius) const {
    const auto observed=(*frame.raw)(world,radius);
    const auto expected=RawDepthBallOracle(frame.depth,geometry,frame.observation_pose)(world,radius);
    if(observed!=expected)throw std::runtime_error("raw ball status mismatch");
    ++counts.broad.at(unsigned(observed));
    return observed;
  }
};
struct CheckedFrame : CruiseDepthFrame {
  std::optional<CheckedRaw> raw;
  void prepare(const LocalGeometryTables &g){
    CruiseDepthFrame::prepare(g);
    raw.emplace(CheckedRaw{*this,g});
    ++counts.frames;
  }
};
struct CheckedRig : ImageDepthRig {
  mutable Direction3 comparison_heading{};
  HeadingPlane heading_plane(const BodyState &pose,const Direction3 &body) const {
    comparison_heading=body;
    return ImageDepthRig::heading_plane(pose,body);
  }
  std::optional<double> image_lateral_miss(const HeadingPlane &plane,const Direction3 &body,
                                           const LocalGeometryTables &g) const {
    const auto actual=ImageDepthRig::image_lateral_miss(plane,body,g);
    if(actual){
      const auto error=nav_add(body,comparison_heading,-std::max(0.,direction_dot(body,comparison_heading)));
      const double diff=std::abs(*actual-nav_norm(error));
      counts.max_miss_error=std::max(counts.max_miss_error,diff);
      if(diff>1e-8)throw std::runtime_error("image lateral miss mismatch");
      ++counts.image_misses;
    }else ++counts.off_image;
    return actual;
  }
  bool surface_intersects(const NavigationObservation &frame,const LocalGeometryTables &g,
                          const Direction3 &world,double radius) const {
    const bool actual=ImageDepthRig::surface_intersects(frame,g,world,radius);
    if(actual!=archived_surface(frame,g,world,radius))throw std::runtime_error("surface veto mismatch");
    save_refinement(frame,world,radius);
    ++counts.surface[actual];
    return actual;
  }
};
}
#define CruiseDepthFrame CheckedFrame
#define ImageDepthRig CheckedRig
#define main current_flight_entry
#include "../../../src/auto_cruise/controller.cpp"
#undef main
#undef CruiseDepthFrame
#undef ImageDepthRig

// The archive images supply the original RTX depth. Synthetic patterns add
// missing pixels, tile edges, near-plane crossings and conflicting cameras.
void fixtures(const char *calfile,const char *depthfile,bool skew){
  uint32_t w,h;
  auto cal=navigation_calibration(calfile,w,h);
  if(skew){
    for(auto &camera:cal){
      camera.K[1]=8.;camera.K[3]=-.75;
      camera.T_BC[0]*=1.+1e-8; // permissible nearly-rigid calibration
    }
  }
  const auto version=navigation_calibration_version(cal,w,h);
  NavigationPipeline pipe;pipe.configure(cal,w,h,version);
  const auto &g=pipe.builder().local_geometry();
  CheckedRig rig;rig.configure(g);
  std::array<CheckedFrame,2> frames;
  for(auto &f:frames)pipe.allocate(f);
  std::mt19937 gen(19453);
  std::uniform_real_distribution<double> number(-1.,1.);
  for(unsigned k=0;k<12;++k){
    auto &f=frames[k%2];
    BodyState pose;
    const double a=.53*k,b=.09*(k%5);const double ca=std::cos(a),sa=std::sin(a),cb=std::cos(b),sb=std::sin(b);
    pose.R_WB={ca*cb,-sa,ca*sb,sa*cb,ca,sa*sb,-sb,0.,cb};
    pose.p_W={-16.+.1*k,4.2-.3*k,3.+k*.05};
    pose.t_ns=(k+1)*BaseConfig::period_ns;
    std::ifstream input(depthfile,std::ios::binary);
    for(auto &img:f.depth.D){
      if(k%4==0){
        if(!input.read(reinterpret_cast<char*>(img.data()),img.size()*sizeof(float)))throw std::runtime_error("short fixture image");
        for(float &z:img)if(std::isinf(z)&&z>0)z=4.5f;
      }else{
        std::fill(img.begin(),img.end(),k%4==1?4.5f:std::numeric_limits<float>::quiet_NaN());
        if(k%4==3)for(unsigned v=0;v<h;++v)for(unsigned u=0;u<w;++u){
          const unsigned patch=(u/16+v/13)%7;
          if(patch!=0)img[v*w+u]=patch<3?1.5f:4.5f;
          if((u+v)%137==0)img[v*w+u]=.18f;
        }
      }
    }
    navigation_identity(f,k+1,version);
    f.observation_pose=pose;f.depth.t_ns=pose.t_ns;
    f.identity.simulation_stamp_ns=pose.t_ns;f.identity.camera_stamp_ns.fill(pose.t_ns);
    f.depth.local.ready=f.depth.observations.ready=false;
    f.prepare(g);
    if((*f.raw)(pose.p_W,.3)!=TubeStatus::Unknown)throw std::runtime_error("stale frame accepted before preprocessing");
    NavigationState state;state.actual=pose;state.actual_valid=true;state.current_simulation_ns=pose.t_ns;
    const DesiredMotion intent{{1,0,0},12.,true,true};
    if(!pipe.process_if(f,state,intent,false,false,[]{return NavigationPlan{false,false,nullptr};}).input_ready)
      throw std::runtime_error("fixture rejected");
    for(unsigned n=0;n<1800;++n){
      const auto &obs=frames[(k && n%3==0)?(k-1)%2:k%2];
      const double length=n%4==0?.45:4.75;
      const Direction3 body{length*number(gen),length*number(gen),length*number(gen)};
      const auto world=nav_add(obs.observation_pose.p_W,nav_rotate(obs.observation_pose.R_WB,body));
      const double radius=.02+.28*std::abs(number(gen));
      Direction3 heading=nav_add(body,Direction3{number(gen),number(gen),number(gen)},.35);
      if(n%7==0)heading={0.,0.,1.};
      if(n%11==0)heading=ref_scale(body,-1.);
      if(unit_direction(heading)){
        const auto plane=rig.heading_plane(obs.observation_pose,heading);
        rig.image_lateral_miss(plane,body,g);
      }
      (*obs.raw)(world,radius);
      if(n%4==0)rig.surface_intersects(obs,g,world,radius);
    }
    // Tangent to the inflated finite pixel at the exact optical-Z sample.
    auto &img=f.depth.D[0];
    auto center_pixel=(h/2)*w+w/2;
    if(valid_depth(img[center_pixel])){
      const auto &t=cal[0].T_BC;const Direction3 origin{t[3],t[7],t[11]};
      const double z=img[center_pixel];
      const auto contact=nav_add(origin,g.pixel(0,center_pixel).q,z);
      const double half=.5*(nav_norm(nav_add(g.pixel(0,1).q,g.pixel(0,0).q,-1.))+
                             nav_norm(nav_add(g.pixel(0,w).q,g.pixel(0,0).q,-1.)))+1e-9;
      for(double delta:{-1e-10,-1e-14,0.,1e-14,1e-10}){
        auto body=contact;body[2]+=.30*ref_rotation_norm_bound(pose.R_WB)+1e-7+z*half+delta;
        rig.surface_intersects(f,g,nav_add(pose.p_W,nav_rotate(pose.R_WB,body)),.30);
      }
    }
  }
}
int main(int argc,char **argv) try {
  if(argc==4 && std::string(argv[1])=="--fixtures"){
    fixtures(argv[2],argv[3],false);fixtures(argv[2],argv[3],true);
    for(auto n:counts.broad)if(!n)throw std::runtime_error("missing ball status in fixtures");
    if(!counts.surface[0] || !counts.surface[1])throw std::runtime_error("missing surface outcome");
  }else{
    if(argc==10)capture_dir=std::filesystem::path(argv[4]).parent_path()/"refinement_frames";
    if(current_flight_entry(argc,argv)!=0)return 2;
  }
  counts.report();
  return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 2;}
