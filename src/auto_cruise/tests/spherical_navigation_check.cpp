#include "../directional_guidance.hpp"
#include <iostream>
#include <filesystem>
#include "new_controller/navigation_io.hpp"
#include "new_controller/spherical_navigation.hpp"

using namespace ego4p0;
namespace sp=ego4p0::spherical;

static void require(bool value,const char *label){
  if(!value)throw std::runtime_error(label);
}

static void check_position_flowbox(){
  constexpr double V=12.,gain=2.,en=.04,eb=-.06;
  for(double k:{0.,1e-10,.4}){
    // t=+X, curvature toward +Y. Construct points on the same neighbouring
    // streamline independently in Cartesian test coordinates.
    PositionFlowBox box({},{{sp::pi/2.,0.},k});
    for(double station:{-.05,0.,.12,.24}){
      const double phase=k*station,c=std::cos(phase),s=std::sin(phase);
      const double x=(k?s/k:station)-en*s;
      const double y=(k?2.*std::pow(std::sin(phase/2.),2)/k:0.)+en*c;
      const auto offset=sp::from_xyz(x,y,eb),v=sp::from_xyz(11.9,.8,-.2);
      PositionFlowBox::Coordinates q;
      require(box.coordinates(offset,q),"local chart refused");
      require(std::abs(q.phase_m-station)<2e-12 && std::abs(q.normal_m-en)<2e-12 &&
              std::abs(q.binormal_m-eb)<2e-12,"section labels must stay constant along the streamline");
      const auto g=box.guidance(q,v,V,gain);
      const auto desired=sp::to_xyz({g.desired,V});
      require(std::abs(-s*desired[0]+c*desired[1]+gain*en)<2e-12 &&
              std::abs(desired[2]+gain*eb)<2e-12,"J v_desired must equal -K eta");
      constexpr double h=1e-6;
      PositionFlowBox::Coordinates plus,minus;
      require(box.coordinates(sp::add(offset,v,h),plus)&&box.coordinates(sp::add(offset,v,-h),minus),"derivative chart");
      const auto fwd=box.guidance(plus,v,V,gain),back=box.guidance(minus,v,V,gain);
      const auto df=sp::scale(sp::add({fwd.desired,1.},{back.desired,1.},-1.),.5/h);
      require(sp::distance(df,g.direction_rate)<3e-7,"direction feedforward differential");
    }
  }
  PositionFlowBox curved({},{{sp::pi/2.,0.},.5});
  PositionFlowBox::Coordinates unused;
  require(!curved.coordinates(sp::from_xyz(0.,2.,0.),unused),"singular field axis accepted");
  const sp::Bearing upright{sp::pi,.5*sp::pi},toward=sp::advance(upright,{.6,.8},sp::pi/2.);
  PositionFlowBox polar(upright,{toward,.3});
  const auto offset=sp::add({upright,.11},{toward,.07});
  PositionFlowBox canonical({},{{sp::pi/2.,0.},.3});
  PositionFlowBox::Coordinates a,b;
  require(polar.coordinates(offset,a)&&canonical.coordinates(sp::from_xyz(.11,.07,0.),b)&&
          std::abs(a.normal_m-b.normal_m)<2e-12 && std::abs(a.phase_m-b.phase_m)<2e-12 &&
          std::abs(a.binormal_m)<2e-12,"rotated frame including pole");
}

static void check_field() {
  SphericalStencil grid;grid.configure(3.,true);FieldLayer field;field.allocate(grid.layout);
  for(const sp::Bearing goal: {sp::Bearing{3.14159,.4},sp::Bearing{-3.14159,-.7},sp::Bearing{0.,sp::pi/2.}}){
    grid.prepare_goal(goal);
    for(uint32_t i=0;i<grid.nodes.size();++i){
      require(grid.goal_score(i)==sp::cosine(grid.nodes[i].bearing,goal),"goal score must preserve exact ties");
      for(uint32_t j: {0U,i,uint32_t(grid.nodes.size()-1-i)})
        require(grid.node_cosine(i,j)==sp::cosine(grid.nodes[i].bearing,grid.nodes[j].bearing),"grid cosine equivalence");
    }
  }
  field.status=FieldStatus::Valid;
  for(size_t i=0;i<grid.nodes.size();++i){
    field.boundary[i]=uint8_t(FieldBoundary::Interior);field.state[i]=uint8_t(CandidateStatus::Available);
    const auto p=grid.nodes[i].bearing;const double rho=direction_pi/2-p.el;
    field.phi[i]=.5+.2*rho*std::cos(p.az);
  }
  sp::Tangent gradient;auto pole=bearing_gradient(grid,field,{0.,direction_pi/2},1e-9,gradient);
  require(pole.valid && pole.local_chart && std::abs(pole.potential-.5)<1e-10 &&
          std::abs(gradient.north+1.)<1e-9,"regular pole chart");
  CurveEvidence evidence;
  require(angular_arc_clear(grid.layout,field,{3.12,.05},{-3.12,.05},evidence),"wrap around seam");
  field.state[*grid.layout.cell(-direction_pi,.05)]=uint8_t(CandidateStatus::Unknown);
  require(!angular_arc_clear(grid.layout,field,{3.10,.05},{-3.10,.05},evidence)&&
          (evidence.kind&CurveUnknown),"unknown at seam blocks whole arc");
}

static void check_reference() {
  HarmonicResult f;f.curve.resize(3);f.curve_size=3;f.termination=CurveEnd::Exit;
  DirectionGridLayout grid;grid.configure(1.,1.);
  f.fine.allocate(grid);std::fill(f.fine.state.begin(),f.fine.state.end(),uint8_t(CandidateStatus::Available));
  std::fill(f.fine.boundary.begin(),f.fine.boundary.end(),uint8_t(FieldBoundary::Interior));
  f.curve[0].alpha=3.13;f.curve[1].alpha=-3.13;f.curve[1].beta=.02;
  f.curve[2].alpha=-3.08;f.curve[2].beta=.06;
  const auto anchor=sp::from_xyz(11.,-3.,2.);
  SphericalReference r;require(r.build(f,anchor,.18),"reference build");
  const double s=r.length();
  std::array<double,3> independent=sp::to_xyz(anchor);
  for(unsigned i=0;i<12000;++i){
    const auto h=sp::to_xyz({r.heading((i+.5)*s/12000.),s/12000.});
    for(int j=0;j<3;++j)independent[j]+=h[j];
  }
  const auto final=r.sample(s);const auto numerical=sp::from_xyz(independent[0],independent[1],independent[2]);
  require(sp::distance(final.position,numerical)<=final.error+1e-10,"radial position integral bound");
  double lo=0.,hi=s;const sp::Bearing join_heading{-3.13+.18,.02};
  for(int i=0;i<75;++i){
    const double a=(2*lo+hi)/3,b=(lo+2*hi)/3;
    if(sp::angle(r.heading(a),join_heading)<sp::angle(r.heading(b),join_heading))hi=b;
    else lo=a;
  }
  const double join=(lo+hi)*.5;
  require(sp::angle(r.heading(join),join_heading)<1e-10 && r.sample(join).curvature.r>1e-4 &&
          sp::distance(r.sample(join-1e-7).curvature,r.sample(join+1e-7).curvature)<2e-5,"continuous curvature at spline knot");
  for(int i=1;i<24;++i){
    const double at=s*i/24.,ds=1e-6;
    const auto sample=r.sample(at);
    auto fwd=sp::log(sample.heading,r.heading(at+ds));
    auto back=sp::log(sample.heading,r.heading(at-ds));
    sp::Tangent velocity{(fwd.east-back.east)/(2.*ds),(fwd.north-back.north)/(2.*ds)};
    const double mag=std::hypot(velocity.east,velocity.north);
    const auto axis=sp::advance(sample.heading,{velocity.east/mag,velocity.north/mag},direction_pi/2);
    require(sp::distance({axis,mag},sample.curvature)<2e-6,"normal-chart differential");
    require(sample.curvature.r<=NavigationConfig::reference_max_curvature_per_m+1e-12,"bounded metric curvature");
  }
  {
    auto obstacle=f;obstacle.curve[0].alpha=-.50;obstacle.curve[0].beta=0.;
    obstacle.curve[1].alpha=0.;obstacle.curve[1].beta=0.;
    obstacle.curve[2].alpha=.2;obstacle.curve[2].beta=.5;
    for(double az=-.32;az<-.02;az+=.007)
      for(double el=-.10;el<-.019;el+=.007)obstacle.fine.state[*grid.cell(az,el)]=uint8_t(CandidateStatus::Unknown);
    SphericalReference rejected;
    require(!rejected.build(obstacle,anchor,0.),"spline overshoot into unknown cells");
  }
  const double reference_length=CurvatureSchedule::make(.42).length;
  sp::Radial midpoint{},endpoint{};
  for(uint32_t count:{2u,31u}){
    f.curve_size=count;f.curve.resize(count);
    for(uint32_t i=0;i<count;++i){
      const auto point=sp::advance({3.12,.10},{.8,.6},.42*i/(count-1));
      f.curve[i].alpha=point.az;f.curve[i].beta=point.el;
    }
    SphericalReference great;require(great.build(f,anchor,.18),"great circle resampling");
    require(std::abs(great.length()-reference_length)<1e-11,"sampling-invariant total turn length");
    const auto mid=great.sample(great.length()*.5).position,end=great.sample(great.length()).position;
    if(count==2){midpoint=mid;endpoint=end;}
    else require(sp::distance(midpoint,mid)<1e-11 && sp::distance(endpoint,end)<1e-11,"sampling-invariant metric positions");
    require(sp::angle(great.heading(great.length()*.5),
      {sp::advance({3.12,.10},{.8,.6},.21).az+.18,sp::advance({3.12,.10},{.8,.6},.21).el})<1e-11,"sampling-invariant midpoint");
  }
  require(sp::angle(final.heading,{-3.08+.18,.06})<1e-10,"tail heading");
}
static void packet(NavigationObservation &o,uint64_t g,int64_t stamp){
  o.depth.sequence=g;o.depth.t_ns=stamp;o.identity={g,1,g,stamp,{stamp,stamp,stamp,stamp},15};
  o.observation_pose.t_ns=stamp;o.observation_pose_valid=true;
}
static void check_deferred_panorama(const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h) {
  FrameObservationBuilder builder;
  builder.configure(cal,w,h,1,1,16,4,0,false,true,true,true,false,true,true);
  PixelRayTables rays;rays.configure(cal,w,h);
  DepthFrame eager,deferred,intervening;
  for(auto f:{&eager,&deferred,&intervening}){
    f->allocate(size_t(w)*h);builder.allocate(*f);
    for(auto &image:f->D)std::fill(image.begin(),image.end(),10.f);
  }
  eager.D[0][w*h/2]=.8f;eager.D[1][w*h/2]=NAN;deferred.D=eager.D;
  eager.sequence=deferred.sequence=1;intervening.sequence=2;
  builder.process(eager,rays);builder.process(deferred,rays,true);
  builder.process(intervening,rays);
  builder.process(deferred,rays);
  const auto &a=eager.local,&b=deferred.local;
  require(b.ready && !b.panorama_pending && a.node_state==b.node_state &&
          a.excluded_camera_mask==b.excluded_camera_mask && a.observation_cameras==b.observation_cameras &&
          a.complete_cameras==b.complete_cameras && a.missing_prefix==b.missing_prefix &&
          !std::memcmp(a.min_range.data(),b.min_range.data(),a.min_range.size()*sizeof(double)),
          "deferred panorama must use its own image and exclusions after intervening frame");
}
static void check_pipeline(const std::filesystem::path &fixture) {
  uint32_t w,h;const auto cal=navigation_calibration(fixture/"simulation.sensors.json",w,h);
  check_deferred_panorama(cal,w,h);
  SphericalNavigation pipeline;pipeline.configure(cal,w,h,1,true);
  NavigationObservation o;pipeline.allocate(o);for(auto &image:o.depth.D)std::fill(image.begin(),image.end(),10.f);
  auto oracle=[](sp::Radial,double radius){return radius>BaseConfig::body_radius_m?TubeStatus::ObservedClear:TubeStatus::Unknown;};
  sp::Radial position{},velocity{{},12.},input=velocity,omega{};
  packet(o,1,BaseConfig::period_ns);
  const auto image_only=pipeline.process(o,o.depth.t_ns,position,velocity,{.45,.11},true,false);
  const LocalFieldView view(o.depth,pipeline.builder().local_geometry());
  require(image_only.input_ready && view.image_evidence_ready() && !view.ready(),"image-only group exposed a full field");
  ++o.depth.t_ns;
  require(!view.image_evidence_ready(),"stale image evidence accepted");
  --o.depth.t_ns;
  const auto builds=pipeline.builder().builds();
  const auto turn=pipeline.process(o,o.depth.t_ns,position,velocity,{.45,.11},true,true);
  require(view.ready() && pipeline.builder().builds()==builds,"same-group field must finish without rereading cameras");
  require(turn.angular_ready && turn.spatial_ready && turn.turn_length_m>0.,"nontrivial spherical harmonic path");
  const auto wrapped=pipeline.process(o,o.depth.t_ns,position,velocity,{2.*direction_pi+.45,.11},true,true);
  require(wrapped.spatial_ready && std::abs(wrapped.turn_length_m-turn.turn_length_m)<1e-9,"wrapped goal longitude");
  const auto &angular=pipeline.angular_step(o,o.depth.t_ns,position,velocity,{.45,.11},true);
  require(angular.angular_ready && !angular.spatial_ready && !angular.spatial_enabled &&
          angular.spatial_ns==0 && pipeline.field().curve_size>1,"angular-only step built a position reference");
  const auto heading=cruise::field_step_heading(pipeline.field(),{.45,.11},0.);
  const auto &next_sample=pipeline.field().curve[1];
  require(heading && sp::angle(*heading,{next_sample.alpha,next_sample.beta})<1e-12,
          "multi-sample field must advance along its first arc");
  const auto further=cruise::field_step_heading(pipeline.field(),{.45,.11},.04);
  require(further && sp::angle(*heading,*further)>1e-4,"available turn budget stuck at first grid sample");
  const auto &stored=pipeline.field();
  const auto n=stored.curve_size;
  uint32_t cursor=0;
  const auto &resume=stored.curve[n/2];
  const auto continued=cruise::continue_field_heading(stored,{resume.alpha,resume.beta},{.45,.11},.04,cursor);
  require(continued && cursor==n/2,"reuse must advance on the stored angular curve");
  require(!cruise::continue_field_heading(stored,{-1.5,-.6},{.45,.11},.04,cursor),
          "off-curve heading must require a new field");
  packet(o,2,2*BaseConfig::period_ns);
  const auto image_tick=pipeline.angular_step(o,o.depth.t_ns,position,velocity,{.45,.11},true,false);
  require(image_tick.input_ready && !view.ready() && !image_tick.field_attempted && image_tick.field_ns==0 &&
          pipeline.field().curve_size==n,"a new frame must retain steering without rebuilding a panorama or field");
  const auto raw_builds=pipeline.builder().builds();
  const auto replan=pipeline.angular_step(o,o.depth.t_ns,position,velocity,{.42,.09},true,true);
  require(replan.angular_ready && replan.field_attempted && !replan.spatial_enabled && !replan.spatial_ready &&
          replan.spatial_ns==0 && pipeline.builder().builds()==raw_builds,"replan on fresh images without rereading cameras");
  require(std::abs(cruise::required_free_travel(12.)-.60)<1e-12,"execution and braking travel");
  for(int i=1;i<=15;++i){
    packet(o,i+1,(i+1)*BaseConfig::period_ns);
    const auto tick=pipeline.rolling_tick(o,o.depth.t_ns,position,velocity,{.45,.11},input,omega,oracle);
    require(tick.mode==RollingMode::Tracking && tick.committed==(i==1),"rolling attach and renew");
    if(i==1){
      const auto duplicate=pipeline.rolling_tick(o,o.depth.t_ns,position,velocity,{.45,.11},input,omega,oracle);
      require(duplicate.mode==RollingMode::RepeatedState && duplicate.plant_input.r==0.,"duplicate state emits no second command");
    }
    input=tick.plant_input;omega=tick.omega;
    const auto applied=sp::add(velocity,sp::add(input,velocity,-1.),BaseConfig::physics_dt_s/BaseConfig::plant_tau_s);
    require(sp::distance(applied,tick.next_velocity)<1e-9,"sampled actuator response");
    velocity=applied;position=sp::add(position,velocity,BaseConfig::physics_dt_s);
  }
  packet(o,17,17*BaseConfig::period_ns);
  const auto denied=pipeline.rolling_tick(o,o.depth.t_ns,position,velocity,{.45,.11},input,omega,
    [](sp::Radial,double){return TubeStatus::Unknown;});
  require(denied.mode==RollingMode::Protection,"revoke previous prefix on missing proof");
  // Same heading, off the reference in both transverse dimensions. Leave
  // the committed reference in place when injecting the translation.
  for(const sp::Bearing goal:{sp::Bearing{},sp::Bearing{.45,.11}}){
    pipeline.reset_epoch();position={};velocity={{},12.};input=velocity;omega={};
    double initial_error=0.,final_error=0.;
    for(int i=1;i<=180;++i){
      if(i==6)position=sp::add(position,sp::from_xyz(0.,.06,-.08));
      packet(o,i,i*BaseConfig::period_ns);
      const auto t=pipeline.rolling_tick(o,o.depth.t_ns,position,velocity,goal,input,omega,oracle);
      require(t.mode==RollingMode::Tracking && t.committed==(i==1) && t.flowbox_valid,
              "parallel displacement must retain the original reference");
      final_error=std::hypot(t.flowbox_normal_m,t.flowbox_binormal_m);
      if(i==6){
        initial_error=final_error;
        require(initial_error>.095 && initial_error<.105,"metric offset detected on the committed reference");
        if(goal.az==0.){
          auto v=sp::to_xyz(t.next_velocity);
          require(std::abs(initial_error-.1)<1e-10 && v[1]<0. && v[2]>0.,"parallel displacement feedback sign");
        }
      }
      velocity=t.next_velocity;position=sp::add(position,velocity,BaseConfig::physics_dt_s);
      input=t.plant_input;omega=t.omega;
    }
    require(final_error<initial_error*.25,"metric recovery with correct initial heading");
    std::cout<<(goal.az==0.?"straight":"turn")<<" 0.06/-0.08 m, displacement norm "
             <<initial_error<<" -> "<<final_error<<" m in "<<174*BaseConfig::physics_dt_s<<" s\n";
  }
  pipeline.reset_epoch();position={};velocity={{},12.};input=velocity;omega={};
  packet(o,1,BaseConfig::period_ns);
  const auto attached=pipeline.rolling_tick(o,o.depth.t_ns,position,velocity,{},input,omega,oracle);
  require(attached.mode==RollingMode::Tracking,"displacement rejection fixture");
  packet(o,2,2*BaseConfig::period_ns);
  const auto too_far=pipeline.rolling_tick(o,o.depth.t_ns,sp::from_xyz(.06,.3,0),velocity,{},
                                           attached.plant_input,attached.omega,oracle);
  require(too_far.flowbox_valid && too_far.mode==RollingMode::Protection &&
          too_far.reason==RollingReason::Controller,"displacement above the certified bound must disengage");
  for(int64_t short_step:{1,1000000,4999999}){
    pipeline.reset_epoch();
    packet(o,1,BaseConfig::period_ns);const sp::Radial nominal{{},12.};
    require(pipeline.rolling_tick(o,o.depth.t_ns,{},nominal,{},nominal,{},oracle).mode==RollingMode::Tracking,"timing fixture attachment");
    packet(o,2,BaseConfig::period_ns+short_step);
    const auto discontinuity=pipeline.rolling_tick(o,o.depth.t_ns,{},nominal,{},nominal,{},oracle);
    require(discontinuity.mode==RollingMode::Protection && discontinuity.reason==RollingReason::Timing &&
            discontinuity.plant_input.r==0.,"positive irregular step is rejected");
  }
  o.identity.camera_mask=7;
  require(!pipeline.process(o,o.depth.t_ns,position,velocity,{},true,true).input_ready,"missing camera");
  packet(o,18,18*BaseConfig::period_ns);
  require(!pipeline.process(o,o.depth.t_ns+10000001,position,velocity,{},true,true).input_ready,"stale frame");
  packet(o,19,19*BaseConfig::period_ns);
  require(!pipeline.process(o,o.depth.t_ns,position,velocity,{NAN,0.},true,true).input_ready,"invalid bearing");
  require(!pipeline.process(o,o.depth.t_ns,{{NAN,0.},1.},velocity,{},true,true).input_ready,"invalid radial position");
}
int main(int argc,char **argv)try{
  if(argc!=2)return 2;
  check_position_flowbox();check_field();check_reference();check_pipeline(argv[1]);
  std::cout<<"PASS polar field, seam and unknown coverage, reference integral, tilted harmonic plan, rolling renewal and refusal\n";
  return 0;
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
