#define main legacy_export_driver_not_executed
#include "harmonic_fields_benchmark.cpp"
#undef main
#include "new_controller/navigation_io.hpp"
#include <iostream>
using namespace ego4p0;namespace fs=std::filesystem;
void require(bool b,const char*s){if(!b)throw std::runtime_error(s);}
static void evidence(const fs::path&dir,NavigationPipeline&p,const NavigationObservation&o,const fs::path&archive){
  navigation_export(dir,p);save_harmonic(dir,p.field());save_block_levels(dir,o.depth,p.builder().local_geometry(),p.planner());
  std::ofstream raw(dir/"raw.bin",std::ios::binary),filled(dir/"filled.bin",std::ios::binary),coeff(dir/"range_coefficients.bin",std::ios::binary);
  for(int c=0;c<4;++c){raw.write(reinterpret_cast<const char*>(o.depth.D[c].data()),o.depth.D[c].size()*sizeof(float));filled.write(reinterpret_cast<const char*>(o.depth.D_filled[c].data()),o.depth.D_filled[c].size()*sizeof(float));const auto&v=p.builder().local_geometry().virtual_tables().coefficients(c);coeff.write(reinterpret_cast<const char*>(v.data()),v.size()*sizeof(double));}
  fs::copy_file(archive/"simulation.sensors.json",dir/"simulation.sensors.json",fs::copy_options::overwrite_existing);
  const auto&i=p.intent();std::ofstream input(dir/"input.json");input<<std::setprecision(17)<<"{\"q0\":["<<i.q0[0]<<','<<i.q0[1]<<','<<i.q0[2]<<"],\"desired\":["<<i.desired.d_desired[0]<<','<<i.desired.d_desired[1]<<','<<i.desired.d_desired[2]<<"],\"synthetic_angular_domain\":false,\"offline_q0_selected_from_available\":true,\"new_simulation\":false}";
}
int main(int argc,char**argv){try{
  require(NavigationLimits::compensation_input({17,0,0},{15,0,0},.005)&&!NavigationLimits::compensation_input({17.001,0,0},{15,0,0},.005)&&!NavigationLimits::compensation_input({3,0,0},{0,0,0},.005),"compensation 17/450 actual-dt validation");
  require(!NavigationLimits::compensation_input({1,0,0},{0,0,0},.002),"do not inflate a small actual interval to 5ms");
  require(NavigationLimits::heading_input({10,0,0},{9.5,0,0},.005)&&!NavigationLimits::heading_input({10.01,0,0},{9.5,0,0},.005)&&!NavigationLimits::heading_input({1,0,0},{0,0,0},.005),"heading 10/100 actual-dt validation");
  if(argc!=3)throw std::runtime_error("archive output");uint32_t w,h;auto cal=navigation_calibration(fs::path(argv[1])/"simulation.sensors.json",w,h);NavigationPipeline p;p.configure(cal,w,h,1);NavigationObservation o;p.allocate(o);navigation_read_depth(fs::path(argv[1])/"last_depth.bin",o);navigation_identity(o,1);
  // Select a documented offline test anchor; no measured motion is fabricated.
  p.builder().process(o.depth,p.rays());const auto desired_anchor=sphere_point(-20*direction_pi/180,0);Direction3 anchor{};double best=-2;const auto&st=p.planner().fine_stencil();for(uint32_t i=0;i<o.depth.local.node_state.size();++i)if(o.depth.local.node_state[i]==uint8_t(CandidateStatus::Available)){double score=direction_dot(st.nodes[i].direction,desired_anchor);if(score>best){best=score;anchor=st.nodes[i].direction;}}
  require(best>.8,"archive available anchor");NavigationState state;state.actual_valid=true;state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;state.actual.v_B=nav_scale(anchor,12);DesiredMotion desired{sphere_point(20*direction_pi/180,0),12,true,true};
  auto a=p.process(o,state,desired);std::cout<<"archive invalid="<<navigation_invalid_name(a.invalid)<<" angular="<<a.angular_ready<<" coarse="<<p.field().coarse.nodes<<" fine="<<p.field().fine.nodes<<'\n';require(o.depth.observations.second_ready&&o.depth.observations.second_columns==27&&o.depth.observations.second_rows==20,"worker secondary summaries before Omega");require(a.input_ready&&a.angular_ready,"archive complete input-to-Q chain");evidence(fs::path(argv[2])/"archive",p,o,argv[1]);
  const auto field_version=a.field_version;const auto old_builds0=p.builder().builds();auto b=p.process(o,state,desired);require(b.angular_ready&&b.field_version==field_version&&p.field().reused&&p.builder().builds()==old_builds0,"same frame exact field cache reuse");
  require(b.capture_ns==0,"same complete identity reuses captured input");
  auto full_cold=p.process(o,state,desired,true);require(full_cold.angular_ready&&!p.field().reused,"cold invalidates field cache");
  navigation_identity(o,2);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;const auto builds=p.builder().builds();b=p.process(o,state,desired);require(b.angular_ready&&p.field().reused&&p.builder().builds()>builds,"new group rebuilds image and strictly reuses field");
  state.actual.p_W[0]=.01;b=p.process(o,state,desired);require(b.angular_ready&&b.actual.p_W[0]==.01,"actual position reaches unified output");state.actual.p_W[0]=0;
  const auto actual_world=state.actual.v_B;state.actual.R_WB={0,-1,0,1,0,0,0,0,1};state.actual.v_B={actual_world[1],-actual_world[0],actual_world[2]};p.process(o,state,desired);require(nav_angle(p.intent().q0,anchor)<1e-12,"actual world velocity rather than heading");state.actual.R_WB={1,0,0,0,1,0,0,0,1};state.actual.v_B=actual_world;
  const float saved_pixel=o.depth.D[0][0];o.depth.D[0][0]=.01f;++o.identity.payload_version;const auto old_builds=p.builder().builds();auto changed=p.process(o,state,desired);require(changed.input_ready&&!changed.angular_ready&&p.builder().builds()>old_builds,"same-group payload revision forces raw rebuild");o.depth.D[0][0]=saved_pixel;++o.identity.payload_version;require(p.process(o,state,desired).angular_ready,"payload repair rebuild recovers");
  const auto recent=o.identity;navigation_identity(o,1);require(p.process(o,state,desired).invalid==NavigationInvalid::GroupRegression,"older group rejected");o.identity=recent;o.depth.sequence=recent.group;o.depth.t_ns=recent.simulation_stamp_ns;o.observation_pose.t_ns=o.depth.t_ns;
  state.actual.R_WB[0]=std::numeric_limits<double>::quiet_NaN();require(p.process(o,state,desired).invalid==NavigationInvalid::InvalidState,"nonfinite actual pose rejected");state.actual.R_WB[0]=1;
  auto missing_raw=std::move(o.depth.D[1]);require(p.process(o,state,desired).invalid==NavigationInvalid::MissingCamera,"missing raw payload rejected");o.depth.D[1]=std::move(missing_raw);
  auto good=o.identity;o.identity.camera_mask=7;auto bad=p.process(o,state,desired);require(bad.invalid==NavigationInvalid::MissingCamera&&!bad.angular_ready&&p.field().status!=FieldStatus::Valid,"missing input invalidates old output");o.identity=good;
  o.identity.camera_stamp_ns[2]++;require(p.process(o,state,desired).invalid==NavigationInvalid::GroupMismatch,"mixed group timestamps rejected");o.identity=good;
  o.identity.calibration_version=2;require(p.process(o,state,desired).invalid==NavigationInvalid::CalibrationMismatch,"calibration version mismatch");o.identity=good;
  state.configuration_version++;require(p.process(o,state,desired).invalid==NavigationInvalid::ConfigurationMismatch,"configuration mismatch");state.configuration_version=NavigationConfig::version;
  auto now=state.current_simulation_ns;state.current_simulation_ns+=11000000;require(p.process(o,state,desired).invalid==NavigationInvalid::StaleObservation,"stale observation");state.current_simulation_ns=now-1;state.actual.t_ns=now-1;require(p.process(o,state,desired).invalid==NavigationInvalid::FutureObservation,"future observation");state.current_simulation_ns=now;state.actual.t_ns=now;
  o.observation_pose_valid=false;require(p.process(o,state,desired).invalid==NavigationInvalid::MissingObservationPose,"missing observation pose");o.observation_pose_valid=true;state.actual_valid=false;require(p.process(o,state,desired).invalid==NavigationInvalid::MissingActualState,"missing state");state.actual_valid=true;
  p.configure(cal,w,h,2);p.allocate(o);navigation_read_depth(fs::path(argv[1])/"last_depth.bin",o);navigation_identity(o,3,2);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;require(p.process(o,state,desired).angular_ready,"calibration reconfigure invalidates then rebuilds");
  // Shape reuse is independent of placement and raw depth evidence.
  const auto spatial=p.process(o,state,desired,false,true);
  require(spatial.spatial_ready&&!spatial.occupancy_reused&&!p.tube().sections().empty(),"archive spatial and tube ready");
  const auto reused=p.process(o,state,desired,false,true);
  require(reused.spatial_ready&&reused.spatial_reused&&reused.occupancy_reused&&reused.geometry_version==spatial.geometry_version,"exact spatial and evidence reuse");
  state.actual.p_W[0]+=.01;
  const auto moved=p.process(o,state,desired,false,true);
  require(moved.spatial_ready&&moved.spatial_reused&&!moved.occupancy_reused&&moved.geometry_version!=spatial.geometry_version,"anchor change reuses local shape and recomputes occupancy");
  require(moved.field_version==reused.field_version&&p.field().reused&&moved.capture_ns==0,"translation reuses angular solution, not placed occupancy proof");
  navigation_identity(o,4,2);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;
  const auto new_group=p.process(o,state,desired,false,true);
  require(new_group.spatial_ready&&new_group.spatial_reused&&!new_group.occupancy_reused,"new group always checks raw evidence");
  state.current_simulation_ns+=11000000;
  require(!p.process(o,state,desired,false,true).spatial_ready&&p.tube().sections().empty(),"stale input invalidates old tube");
  // An incomplete raw image may be repaired for obstacles, never declared free.
  for(auto&d:o.depth.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());navigation_identity(o,5,2);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;bad=p.process(o,state,desired,false,true);require(bad.input_ready&&!bad.angular_ready&&!bad.spatial_ready&&p.tube().sections().empty(),"all unknown rejects curve");evidence(fs::path(argv[2])/"unknown",p,o,argv[1]);
  // Synthetic known near planes are only a test input. The real archive
  // does not record those planes and cannot certify free space by assumption.
  for(auto &camera:cal)camera.near_clip_z_m=.65;
  p.configure(cal,w,h,3);p.allocate(o);
  for(auto&d:o.depth.D)std::fill(d.begin(),d.end(),10.f);
  navigation_identity(o,6,3);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;
  state.actual.v_B={12,0,0};desired.d_desired={1,0,0};
  const auto calibrated=p.process(o,state,desired,false,true);
  require(calibrated.spatial_ready&&calibrated.observed_sections>0,"calibrated raw ROI certifies an open tube");
  RawDepthBallOracle oracle(o.depth,p.builder().local_geometry(),o.observation_pose);
  require(oracle({1.02,0,.02},.30)==TubeStatus::Unknown,"ball reaching the physical camera near plane is unknown");
  navigation_identity(o,7,3);state.actual.t_ns=o.depth.t_ns;state.current_simulation_ns=o.depth.t_ns;
  require(p.process_if(o,state,desired,false,true,[]{return NavigationPlan{false,false,nullptr};}).input_ready,"rolling can skip capture for a new group");
  const auto after_skip=p.process(o,state,desired,false,true);
  require(after_skip.spatial_ready&&after_skip.capture_ns>0,"new group skipped by rolling still needs its own captured input");
  std::cout<<"PASS real archive / identities / missing inputs / exact reuse / calibration and config invalidation / no stale output / Unknown\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 2;}}
