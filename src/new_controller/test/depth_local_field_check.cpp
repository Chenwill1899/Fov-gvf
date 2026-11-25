#include "new_controller/depth_size_query.hpp"
#include "new_controller/desired_direction.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
#include <random>

static std::atomic<bool> counting{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {if(counting.load()) ++allocations;if(void *p=std::malloc(n?n:1)) return p;throw std::bad_alloc();}
void operator delete(void *p) noexcept {std::free(p);}void operator delete(void *p,std::size_t) noexcept {std::free(p);}
void *operator new[](std::size_t n) {return ::operator new(n);}void operator delete[](void *p) noexcept {::operator delete(p);}void operator delete[](void *p,std::size_t) noexcept {::operator delete(p);}
using namespace ego4p0;
static void require(bool b,const char *m) {if(!b) throw std::runtime_error(m);}
static std::array<CameraCalibration,4> calibration(uint32_t w,uint32_t h) {
  std::array<CameraCalibration,4> cal;
  for(uint32_t c=0;c<4;++c) {
    const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);auto &k=cal[c];
    k.K={30,.2,w*.5,0,31,h*.5,0,0,1};k.near_clip_z_m=.05;
    k.T_BC={si,0,co,.22*co,-co,0,si,.22*si,0,-1,0,.02,0,0,0,1};
  }
  return cal;
}
static void geometry_and_sources() {
  std::mt19937 rng(3127);uint32_t checks=0;
  for(const auto shape:std::array<std::array<uint32_t,2>,4>{{{4,5},{17,19},{35,29},{320,240}}}) {
    const uint32_t w=shape[0],h=shape[1],n=w*h;auto cal=calibration(w,h);PixelRayTables rays;rays.configure(cal,w,h);
    FrameObservationBuilder parallel,serial;
    parallel.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true);
    serial.configure_real(cal,w,h,1,1,16,3,5,false,true,false,true);
    require(!parallel.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true),"static geometry not reused");
    DepthFrame f,reference;for(auto *p:{&f,&reference}) p->allocate(n);parallel.allocate(f);serial.allocate(reference);
    f.t_ns=reference.t_ns=2000000;f.sequence=reference.sequence=1;
    for(uint32_t c=0;c<4;++c) for(uint32_t p=0;p<n;++p) {
      const float z=(rng()%9==0) ? std::numeric_limits<float>::quiet_NaN() : .01f+(rng()%5000)*.002f;
      f.D[c][p]=reference.D[c][p]=z;
      const auto &r=parallel.local_geometry().pixel(c,p);const auto &t=cal[c].T_BC;
      auto range=[&](double z0){return std::hypot(z0*r.q[0]+t[3],z0*r.q[1]+t[7],z0*r.q[2]+t[11]);};
      require(std::abs(range(r.z_limit)-ControllerLimits::local_range_m)<2e-14 && range(r.z_limit*(1-1e-6))<ControllerLimits::local_range_m && range(r.z_limit*(1+1e-6))>ControllerLimits::local_range_m,"offset optical-depth threshold incorrect");++checks;
    }
    parallel.process(f,rays);serial.process(reference,rays);
    const auto &d=f.local;require(d.excluded_blocks==reference.local.excluded_blocks && d.excluded_cells==reference.local.excluded_cells,"parallel compact totals differ");
    require(f.observations.offsets.empty() && f.observations.sources.empty() && !f.observations.full_provenance,"default path stores full provenance");
    uint32_t total=0;
    const LocalFieldView view(f,parallel.local_geometry());
    for(uint32_t c=0;c<4;++c) for(uint32_t p=0;p<n;++p) {
      const double z=f.D_filled[c][p];const auto &r=parallel.local_geometry().pixel(c,p);const auto &t=cal[c].T_BC;
      const bool inside=std::isfinite(z) && z>0 && std::hypot(z*r.q[0]+t[3],z*r.q[1]+t[7],z*r.q[2]+t[11])<=ControllerLimits::local_range_m;
      const auto source=view.pixel_obstacle(c,p%w,p/w);require(bool(source)==inside,"full-resolution local edge/threshold pixel lost or fabricated");
      if(!inside) continue;
      ++total;const double x=z*r.q[0]+t[3],y=z*r.q[1]+t[7],zz=z*r.q[2]+t[11];
      const auto exact=f.observations.layout.cell(std::atan2(y,x),std::atan2(zz,std::hypot(x,y)));
      require(exact && source->direction_cell==*exact,"on-demand pixel has wrong body angle");
      require(d.excluded_camera_mask[*exact]&(1U<<c),"near pixel direction escaped conservative block exclusion");
    }
    (void)total;
    require(d.excluded_camera_mask==reference.local.excluded_camera_mask,"parallel exclusion differs from serial");
    for(uint32_t i=0;i<300;++i) {
      const uint32_t c=rng()%4,u=rng()%w,v=rng()%h,x1=u+1+rng()%(w-u),y1=v+1+rng()%(h-v);
      uint32_t count=0;std::optional<float> minimum;
      for(uint32_t y=v;y<y1;++y) for(uint32_t x=u;x<x1;++x) {float z=f.D[c][y*w+x];if(valid_depth(z)) {++count;if(!minimum || z<*minimum) minimum=z;}}
      const auto q=DepthObservationView(f).region(c,{int(u),int(v),int(x1),int(y1)},DepthImageKind::Raw);
      require(q.z==minimum && q.valid_pixels==count && q.total_pixels==(x1-u)*(y1-v),"raw summary/edge query not exact");
    }
    counting=true;allocations=0;
    for(uint32_t i=0;i<5;++i) {++f.sequence;parallel.process(f,rays);require(!parallel.process(f,rays),"duplicate group rebuilt");const auto s=LocalFieldView(f,parallel.local_geometry()).sphere({1,0,0});(void)s;}
    counting=false;require(allocations==0,"steady local/size processing allocated");
    require(parallel.local_geometry().builds()==1,"local projection table rebuilt in processing");
  }
  std::cout<<"threshold rays="<<checks<<", on-demand exact pixel provenance/conservative block exclusion/serial-parallel equality, 1200 raw ROIs and zero steady allocations; ";
}
static void size_and_unknown() {
  constexpr uint32_t w=65,h=49;auto cal=calibration(w,h);PixelRayTables rays;rays.configure(cal,w,h);FrameObservationBuilder builder;
  builder.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true);DepthFrame f;f.allocate(w*h);builder.allocate(f);f.sequence=1;f.t_ns=1;
  for(auto &d:f.D) std::fill(d.begin(),d.end(),10.f);
  builder.process(f,rays);
  const auto &g=builder.local_geometry();const LocalFieldView view(f,g);
  require(view.sphere({1,0,0}).status==EnvelopeStatus::ObservedClear,"fully raw observed sphere not available");
  require(view.sphere({0,0,0}).status==EnvelopeStatus::CurrentPositionNotEvaluated,"current position rechecked");
  require(view.sphere({.05,0,0}).status==EnvelopeStatus::Unknown,"unproved blind area marked clear");
  SimulationInitialClearance proof{true,1,1,.3,"synthetic exact-time collision-free ball proof"};
  require(view.sphere({.05,0,0},&proof).status==EnvelopeStatus::SimulationInitialClear,"explicit initial basis not used");
  ++proof.sequence;require(view.sphere({.05,0,0},&proof).status==EnvelopeStatus::Unknown,"stale initial basis reused");
  require(view.sphere({ControllerLimits::local_range_m+.01,0,0}).status==EnvelopeStatus::OutsideLocalRange,"outside range sphere accepted");
  const auto clipped=g.project(0,{1,1,0});require(clipped.status==FootprintStatus::Projected && !clipped.fully_inside,"edge footprint silently treated as fully observed");
  require(view.sphere({1,1,0}).status==EnvelopeStatus::Unknown,"partially out-of-image sphere certified clear");
  const auto foot=g.project(0,{1,0,0});const uint32_t x=(foot.region.u0+foot.region.u1)/2,y=(foot.region.v0+foot.region.v1)/2,p=y*w+x;
  f.D[0][p]=std::numeric_limits<float>::quiet_NaN();++f.sequence;builder.process(f,rays);
  require(f.D_filled[0][p]==10.f && view.sphere({1,0,0}).status==EnvelopeStatus::Unknown,"repair automatically granted free evidence");
  f.D[0][p]=.3f;++f.sequence;builder.process(f,rays);
  require(view.sphere({1,0,0}).status==EnvelopeStatus::PotentialObstacle,"full-res thin obstacle ignored by body footprint");
  const uint32_t id=y/3*f.observations.block_columns+x/3;
  require(f.observations.blocks[0][id].mean_z>g.block(0,id).z_limit && bool(view.pixel_obstacle(0,x,y)),"far block mean hid a close edge");
  std::mt19937 rng(4);uint32_t samples=0;
  for(uint32_t c=0;c<4;++c) for(uint32_t trial=0;trial<30;++trial) {
    const double a=c*direction_pi/2;const std::array<double,3> center{std::cos(a)*(1+trial*.03),std::sin(a)*(1+trial*.03),.1};
    const auto projection=g.project(c,center);require(projection.status==FootprintStatus::Projected,"sphere projection failed");
    const auto &t=cal[c].T_BC;const auto &k=cal[c].K;
    for(uint32_t j=0;j<500;++j) {
      const double z=-1+2.*double(rng())/rng.max(),phi=2*direction_pi*double(rng())/rng.max(),xy=std::sqrt(1-z*z);
      const std::array<double,3> point{center[0]+.1*xy*std::cos(phi)-t[3],center[1]+.1*xy*std::sin(phi)-t[7],center[2]+.1*z-t[11]};
      std::array<double,3> pc{};for(uint32_t i=0;i<3;++i) for(uint32_t q=0;q<3;++q) pc[i]+=t[q*4+i]*point[q];
      const double u=(k[0]*pc[0]+k[1]*pc[1])/pc[2]+k[2],v=(k[3]*pc[0]+k[4]*pc[1])/pc[2]+k[5];
      require(u>=projection.u_min-1e-11 && u<=projection.u_max+1e-11 && v>=projection.v_min-1e-11 && v<=projection.v_max+1e-11,"sphere tangent bounds fail to enclose sampled surface");++samples;
    }
  }
  for(auto &c:cal) c.near_clip_z_m=std::numeric_limits<double>::quiet_NaN();
  builder.configure_real(cal,w,h,1,1,16,3,5,true,true,true,true);builder.allocate(f);
  for(auto &d:f.D) std::fill(d.begin(),d.end(),10.f);
  ++f.sequence;builder.process(f,rays);
  require(LocalFieldView(f,builder.local_geometry()).sphere({1,0,0}).status==EnvelopeStatus::Unknown,"missing clipping readback granted free evidence");
  FrameObservationBuilder other;other.configure_real(cal,w,h,1,1,16,3,5,false,true,true,true);
  require(!LocalFieldView(f,other.local_geometry()).ready(),"geometry from another configuration accepted");
  std::cout<<samples<<" sphere projection samples, raw holes/thin edge/blind region/initial proof checks; ";
}
static void desired() {
  BodyState s=body_state(0,0,std::sqrt(.5),std::sqrt(.5),{0,0,0},1);s.p_W={2,3,4};
  auto d=desired_motion({DesiredInputMode::WorldGoal,{2,5,4},20,1},s);
  require(d.valid && d.has_direction && std::abs(d.d_desired[0]-1)<1e-12 && std::abs(d.d_desired[1])<1e-12 && d.v_desired==ControllerLimits::max_speed_mps,"goal transform/limit wrong");
  d=desired_motion({DesiredInputMode::WorldGoal,{2,3,4},10,1},s);require(!d.has_direction && d.v_desired==0,"goal zero normalized");
  d=desired_motion({DesiredInputMode::BodyVelocity,{0,-3,4},0,1},s);require(d.v_desired==5 && d.d_desired[1]==-.6 && d.d_desired[2]==.8,"body velocity magnitude semantics wrong");
  d=desired_motion({DesiredInputMode::WorldVelocity,{0,2,0},0,1},s);require(std::abs(d.v_desired-2)<1e-12 && std::abs(d.d_desired[0]-1)<1e-12,"world input not converted");
  d=desired_motion({DesiredInputMode::BodyVelocity,{1e-12,0,0},0,1},s);require(d.v_desired==0 && !d.has_direction,"near zero direction normalized");
  d=desired_motion({DesiredInputMode::BodyVelocity,{std::numeric_limits<double>::quiet_NaN(),0,0},0,1},s);require(!d.valid && d.v_desired==0,"nonfinite input propagated");
  d=desired_motion({DesiredInputMode::BodyVelocity,{0,0,30},0,1},s);
  require(d.v_desired==12 && d.d_desired[2]==1,"vertical input still has an axis limiter");
  d=desired_motion({DesiredInputMode::BodyVelocity,{12,12,12},0,1},s);
  require(d.v_desired==12 && std::abs(d.d_desired[2]-1/std::sqrt(3.))<1e-12,"diagonal input exceeds 3-D norm cap");
  std::array<double,3> velocity{};for(uint32_t i=0;i<1000;++i) {
    const auto next=acceleration_limited_reference(velocity,i<500 ? std::array<double,3>{10,0,0} : std::array<double,3>{-10,0,0});
    require(std::hypot(next[0]-velocity[0],next[1]-velocity[1],next[2]-velocity[2])<=ControllerLimits::max_acceleration_mps2*ControllerLimits::period_s+1e-15,"reference acceleration exceeds 20m/s2 at 5ms");
    require(std::hypot(next[0],next[1],next[2])<=10.000000000001,"reference speed cap exceeded");velocity=next;
  }
  std::cout<<"desired world/body/goal/zero/invalid and reference 0.10m/s-per-tick limits\n";
}
int main() {try {geometry_and_sources();size_and_unknown();desired();std::cout<<"PASS\n";return 0;}
 catch(const std::exception &e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
