#include "new_controller/harmonic_planner.hpp"
#include "new_controller/harmonic_io.hpp"
#include "new_controller/spatial_reference.hpp"
#include <filesystem>
#include <functional>
#include <iostream>

using namespace ego4p0;
namespace {
std::string context;
unsigned comparisons=0,hits=0,poisoned_hits=0;
void need(bool ok,const char*why){if(!ok)throw std::runtime_error(context+": "+why);}
bool same_number(double a,double b){return a==b||(std::isnan(a)&&std::isnan(b));}
template<class T> void same_prefix(const std::vector<T>&a,const std::vector<T>&b,size_t n,const char*why){
  need(a.size()>=n&&b.size()>=n&&std::equal(a.begin(),a.begin()+n,b.begin()),why);
}
void same_numbers(const std::vector<double>&a,const std::vector<double>&b,size_t n,const char*why){
  need(a.size()>=n&&b.size()>=n,"numeric prefix is truncated");for(size_t i=0;i<n;++i)need(same_number(a[i],b[i]),why);
}
std::array<CameraCalibration,4> calibration(uint32_t w=320,uint32_t h=240){
  std::array<CameraCalibration,4>cal;
  for(uint32_t c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);
    cal[c].K={.6*w,0,w*.5,0,.65*w,h*.5,0,0,1};
    cal[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};cal[c].near_clip_z_m=.65;
  }return cal;
}
HarmonicIntent make_intent(double a=-20.25,double b=.5,double da=20.25,double db=.5){
  HarmonicIntent i;i.q0=sphere_point(a*direction_pi/180,b*direction_pi/180);
  i.desired={sphere_point(da*direction_pi/180,db*direction_pi/180),12,true,true};i.R_WB={1,0,0,0,1,0,0,0,1};return i;
}
void equal_layer(const FieldLayer&a,const FieldLayer&b){
  need(a.status==b.status&&a.nodes==b.nodes&&a.iterations==b.iterations&&a.reused==b.reused&&a.warm_started==b.warm_started&&a.constant_boundary==b.constant_boundary,"layer state/cache metadata differs");
  need(a.boundary==b.boundary&&a.state==b.state&&a.cameras==b.cameras,"layer boundary/state/camera evidence differs");
  need(a.phi.size()==b.phi.size()&&same_number(a.residual,b.residual),"layer residual/size differs");same_numbers(a.phi,b.phi,a.phi.size(),"layer phi differs");
}
void equal_result(const HarmonicResult&a,const HarmonicResult&b){
  need(a.status==b.status&&a.termination==b.termination&&a.curve_size==b.curve_size,"planner result differs");
  need(a.version==b.version&&a.field_version==b.field_version&&a.curve_version==b.curve_version&&a.reused==b.reused,"version/Q cache behavior differs");
  need(a.sequence==b.sequence&&a.configuration==b.configuration&&a.geometry==b.geometry&&a.parameter_revision==b.parameter_revision&&a.stamp_ns==b.stamp_ns,"input identities differ");
  need(a.expanded==b.expanded&&a.coarse_mixed==b.coarse_mixed&&a.exit_clipped==b.exit_clipped&&a.fine_attempts==b.fine_attempts,"route/domain metadata differs");
  need(a.coarse_topology_only==b.coarse_topology_only&&std::string(a.coarse_topology_failure)==b.coarse_topology_failure,"topology mode/result differs");
  need(a.angular_length==b.angular_length&&a.anchor_potential==b.anchor_potential&&a.exit_direction==b.exit_direction,"curve geometry metadata differs");
  need(a.integration_steps==b.integration_steps&&a.gradient_queries==b.gradient_queries&&a.segment_checks==b.segment_checks&&a.curve_failure_kind==b.curve_failure_kind&&a.curve_failure_cell==b.curve_failure_cell&&a.curve_rejected_trials==b.curve_rejected_trials&&a.curve_min_accepted_step_rad==b.curve_min_accepted_step_rad,"integration decisions differ");
  equal_layer(a.coarse,b.coarse);equal_layer(a.fine,b.fine);
  const auto&x=a.adaptive;const auto&y=b.adaptive;
  need(x.nodes==y.nodes&&x.edges==y.edges&&x.refined_nodes==y.refined_nodes,"adaptive counts differ");
  if(x.nodes){
    need(x.owner==y.owner&&x.raw_edges==y.raw_edges,"complete owner/raw face count differs");
    same_prefix(x.representative,y.representative,x.nodes,"representatives differ");same_prefix(x.boundary,y.boundary,x.nodes,"adaptive labels differ");
    same_prefix(x.refined,y.refined,x.nodes,"refined flags differ");same_prefix(x.offset,y.offset,x.nodes+1,"CSR offsets differ");
    same_prefix(x.neighbor,y.neighbor,x.edges,"CSR neighbors differ");same_prefix(x.weight,y.weight,x.edges,"CSR weights differ");same_numbers(x.phi,y.phi,x.nodes,"active adaptive phi differs");
  }
  for(size_t camera=0;camera<4;++camera){need(a.coarse_image[camera].size()==b.coarse_image[camera].size(),"summary dimensions differ");
    for(size_t k=0;k<a.coarse_image[camera].size();++k){const auto&u=a.coarse_image[camera][k];const auto&v=b.coarse_image[camera][k];
      need(u.total==v.total&&u.valid==v.valid&&u.raw_valid==v.raw_valid&&same_number(u.mean_z,v.mean_z)&&same_number(u.min_rho,v.min_rho)&&same_number(u.raw_min_rho,v.raw_min_rho),"new observation summary was lost");}}
  for(uint32_t k=0;k<a.curve_size;++k){const auto&u=a.curve[k];const auto&v=b.curve[k];
    need(u.direction==v.direction&&u.gradient_B==v.gradient_B&&u.unit_B==v.unit_B&&u.cell==v.cell&&u.valid==v.valid&&u.local_chart==v.local_chart&&u.zero_field==v.zero_field,"Q vector/status differs");
    const double av[]={u.alpha,u.beta,u.F_alpha,u.F_beta,u.norm2,u.tau,u.zeta,u.potential,u.magnitude,u.unit_alpha,u.unit_beta};
    const double bv[]={v.alpha,v.beta,v.F_alpha,v.F_beta,v.norm2,v.tau,v.zeta,v.potential,v.magnitude,v.unit_alpha,v.unit_beta};
    for(size_t j=0;j<std::size(av);++j)need(same_number(av[j],bv[j]),"Q scalar differs");
  }
}
void independent_checks(const HarmonicPlanner&p,const HarmonicDepthInput&d,const HarmonicResult&r){
  if(r.coarse_topology_only&&r.coarse.status==FieldStatus::TopologyReady){
    need(std::isnan(r.coarse.residual)&&r.coarse.iterations==0&&r.coarse.solve_ns==0&&r.coarse.solve_cpu_ns==0,"topology certificate masquerades as a numerical solve");
    for(double phi:r.coarse.phi)need(std::isnan(phi),"topology-only coarse phi is not NaN");
    for(uint32_t i=0;i<r.adaptive.nodes;++i)need(std::isnan(r.adaptive.phi[i]),"restored adaptive phi is not NaN");
    need(p.coarse_trial_stats().total_ns==0,"topology mode called coarse numerical backend");
  }
  if(r.fine.status!=FieldStatus::Valid)return;
  const auto&s=p.fine_stencil();double residual=0.;
  for(uint32_t i=0;i<s.nodes.size();++i)if(r.fine.boundary[i]){
    need(d.fine_state[i]==uint8_t(CandidateStatus::Available),"fine field entered original Unknown/obstacle");
    need(std::isfinite(r.fine.phi[i])&&r.fine.phi[i]>=-1e-7&&r.fine.phi[i]<=1+1e-7,"fine potential range failed");
    if(FieldBoundary(r.fine.boundary[i])!=FieldBoundary::Interior)continue;
    double sum=0,diag=0;for(unsigned j=0;j<4;++j){const auto n=s.nodes[i].neighbor[j];if(n==UINT32_MAX||!r.fine.boundary[n])continue;sum+=s.nodes[i].weight[j]*(r.fine.phi[i]-r.fine.phi[n]);diag+=s.nodes[i].weight[j];}
    need(diag>0,"fine isolated Interior node");residual=std::max(residual,std::abs(sum)/diag);
  }
  need(residual<=p.config().residual_tolerance,"independent FV residual failed");
  FieldLayer mask=r.fine;for(uint32_t i=0;i<s.nodes.size();++i)mask.boundary[i]=d.fine_state[i]==uint8_t(CandidateStatus::Available)?uint8_t(FieldBoundary::Interior):0;
  for(uint32_t k=1;k<r.curve_size;++k)need(FineCurveIntegrator::segment_clear(s,mask,r.curve[k-1].direction,r.curve[k].direction),"Q crosses original input mask");
}
void compare_spatial(const HarmonicResult&a,const HarmonicResult&b){
  if(a.status!=FieldStatus::Valid)return;std::vector<Direction3>x,y;
  for(uint32_t i=0;i<a.curve_size;++i){x.push_back(a.curve[i].direction);y.push_back(b.curve[i].direction);}
  SpatialReference u,v;need(u.build(x,a.intent.p_W,a.intent.R_WB)==v.build(y,b.intent.p_W,b.intent.R_WB),"spatial acceptance differs");
  if(!u.valid())return;need(u.nodes()==v.nodes()&&u.profile().length==v.profile().length,"spatial knots differ");
  for(unsigned i=0;i<=8;++i){const double s=(u.profile().length+1)*i/8.;const auto p=u.query(s),q=v.query(s);
    need(p.position_W==q.position_W&&p.tangent_W==q.tangent_W&&p.curvature_W==q.curvature_W&&p.position_error_bound_m==q.position_error_bound_m&&p.zeta==q.zeta&&p.kappa==q.kappa&&p.dkappa_ds==q.dkappa_ds,"3D spatial geometry/certificate differs");}
}
std::array<size_t,8> sizes(const AdaptiveCoarseField&a){return {a.owner.size(),a.offset.size(),a.neighbor.size(),a.weight.size(),a.representative.size(),a.phi.size(),a.boundary.size(),a.refined.size()};}
void poison(HarmonicResult&r){
  auto&a=r.adaptive;a.nodes=a.edges=a.refined_nodes=a.raw_edges=UINT32_MAX;a.edge_merge_ns=123456789;
  for(auto*v:{&a.owner,&a.offset,&a.neighbor,&a.representative})std::fill(v->begin(),v->end(),UINT32_MAX);
  std::fill(a.weight.begin(),a.weight.end(),-123.);std::fill(a.phi.begin(),a.phi.end(),123.);
  std::fill(a.boundary.begin(),a.boundary.end(),255);std::fill(a.refined.begin(),a.refined.end(),255);
  r.coarse_mixed=true;std::fill(r.coarse.phi.begin(),r.coarse.phi.end(),123.);
  std::fill(r.coarse.state.begin(),r.coarse.state.end(),255);std::fill(r.coarse.cameras.begin(),r.coarse.cameras.end(),255);
}
struct Pair {
  HarmonicPlanner cached,oracle;HarmonicDepthInput data;
  std::array<HarmonicResult,3>out,reference;
  std::array<std::array<size_t,8>,3>allocated{};
  uint32_t width=320,height=240;unsigned slot=0;bool native=false;
  Pair(){configure({});summaries_far();rectangle();}
  void configure(HarmonicConfig config,std::array<CameraCalibration,4>cal=calibration(),uint32_t w=320,uint32_t h=240,bool use_native=false){
    width=w;height=h;native=use_native;config.force_coarse_rebuild=false;cached.configure(cal,w,h,config,4);
    config.force_coarse_rebuild=true;oracle.configure(cal,w,h,config,4);
    if(native){cached.set_native_preconditioner(CoarsePreconditioner::IC0);oracle.set_native_preconditioner(CoarsePreconditioner::IC0);}
    else{cached.set_trial_solvers(SparseTrialMode::EigenLDLT,true);oracle.set_trial_solvers(SparseTrialMode::EigenLDLT,true);}
    cached.allocate(data,w,h);data.ready=true;data.sequence=data.configuration=data.geometry=1;
    for(unsigned i=0;i<3;++i){out[i]={};reference[i]={};cached.allocate(out[i]);oracle.allocate(reference[i]);allocated[i]=sizes(out[i].adaptive);}slot=0;
  }
  void summaries_far(){for(uint32_t c=0;c<4;++c)for(uint32_t i=0;i<data.coarse_image[c].size();++i){auto&v=data.coarse_image[c][i];const auto span=data.first_block_edge*data.second_block_edge,x=i%data.columns*span,y=i/data.columns*span;
      v.total=v.valid=v.raw_valid=(std::min(x+span,width)-x)*(std::min(y+span,height)-y);v.mean_z=v.min_rho=v.raw_min_rho=10.;}}
  void summaries(const std::function<void(ImageLevelSummary&)>&change){for(auto&c:data.coarse_image)for(auto&s:c)change(s);}
  void region(const std::function<bool(double,double)>&available){for(uint32_t i=0;i<data.fine_state.size();++i){const bool good=available(-180+(i%360+.5),-90+(i/360+.5));data.fine_state[i]=uint8_t(good?CandidateStatus::Available:CandidateStatus::Unknown);data.cameras[i]=good?1:0;}}
  void rectangle(){region([](double a,double b){return a>=-45&&a<45&&b>=-20&&b<20;});}
  uint32_t cell(double a,double b)const{return cached.fine_stencil().cell(sphere_point(a*direction_pi/180,b*direction_pi/180));}
  HarmonicResult&run(const char*name,const HarmonicIntent&i,bool expected_hit,bool require_valid=true,bool dirty=false,bool spatial=false){
    context=name;const unsigned current=slot++%3;auto&a=out[current];auto&b=reference[current];if(dirty)poison(a);
    ++data.sequence;cached.build(data,i,a);oracle.build(data,i,b);
    need(sizes(a.adaptive)==allocated[current],"cache restore resized caller graph storage");
    equal_result(a,b);need(a.coarse_graph_reused==expected_hit,"graph cache decision wrong");
    need(!b.coarse_graph_reused&&b.coarse_graph_cache_ns==0,"forced-rebuild oracle touched graph cache");
    need(a.coarse_graph_cache_ns>=0&&a.coarse_graph_cache_ns<=a.coarse.domain_ns,"graph-cache phase is outside coarse-domain accounting");
    if(expected_hit){++hits;if(dirty)++poisoned_hits;need(a.adaptive.edge_merge_ns==0,"graph hit still reports edge merge");}
    if(require_valid)need(a.status==FieldStatus::Valid&&a.termination==CurveEnd::Exit,"valid fixture rejected");
    independent_checks(cached,data,a);if(spatial)compare_spatial(a,b);++comparisons;return a;
  }
};
void graph_cache_cases(const std::filesystem::path&exports){
  Pair p;auto in=make_intent();p.run("cold graph",in,false,true,false,true);
  for(unsigned i=0;i<6;++i)p.run("three alternating poisoned output slots",in,true,true,true,i==5);
  // Allocate a fresh destination after the cache was populated. No graph data
  // in this output object has ever been built or validated.
  const unsigned fresh=p.slot%3;p.out[fresh]={};p.reference[fresh]={};p.cached.allocate(p.out[fresh]);p.oracle.allocate(p.reference[fresh]);p.allocated[fresh]=sizes(p.out[fresh].adaptive);
  auto&r=p.run("fresh allocated output on cache hit",in,true,true,true,true);
  if(!exports.empty()){std::filesystem::create_directories(exports);save_harmonic(exports,r);}
  auto version=r.field_version,curve=r.curve_version;
  in.desired.d_desired=sphere_point(20.5*direction_pi/180,.5*direction_pi/180);
  auto&micro=p.run("desired changes inside target cell",in,true);need(!micro.reused&&micro.field_version==version&&micro.curve_version>curve,"graph hit incorrectly reuses changed-intent Q");
  in.q0=sphere_point(-20.1*direction_pi/180,.5*direction_pi/180);p.run("q0 subcell change",in,true);
  in.q0=sphere_point(-18.25*direction_pi/180,.5*direction_pi/180);p.run("q0 other cell same component",in,true);
  in.p_W={2,-3,1};p.run("translation changes placement only",in,true,true,true,true);
  in.desired.d_desired=sphere_point(24.5*direction_pi/180,.5*direction_pi/180);p.run("target changes",in,false);p.run("changed target repeat",in,true);
  in.desired.d_desired=sphere_point(-24.5*direction_pi/180,.5*direction_pi/180);p.run("target and far rim change",in,false);p.run("opposite target repeat",in,true);
  in=make_intent();p.run("restore original target",in,false);
  const auto old_cameras=p.out[(p.slot-1)%3].coarse.cameras;
  std::fill(p.data.cameras.begin(),p.data.cameras.end(),8);auto&cameras=p.run("camera evidence changes without graph change",in,true,true,true);
  need(cameras.coarse.cameras!=old_cameras&&cameras.fine.cameras==p.data.cameras,"camera evidence was reused from old graph");
  p.summaries([](auto&s){s.mean_z=12.;s.valid=s.total-1;});p.run("irrelevant summary values still exported",in,true);
  p.summaries([](auto&s){s.raw_valid=s.total-1;});auto&missing=p.run("raw completeness loss with same fine component",in,false);
  need(missing.coarse_mixed,"raw completeness loss lost mixed-domain diagnostic");p.run("same incomplete summary shape",in,true,true,true);
  p.summaries_far();p.run("raw completeness recovery",in,false);
  p.summaries([](auto&s){s.total=s.valid=s.raw_valid=0;});p.run("zero raw total",in,false);p.summaries_far();p.run("raw total recovery",in,false);
  const double clear=ControllerLimits::local_range_m+ControllerLimits::radius_m;
  p.summaries([&](auto&s){s.raw_min_rho=clear;});p.run("raw clearance equality is not complete",in,false);
  p.summaries([&](auto&s){s.raw_min_rho=std::nextafter(clear,std::numeric_limits<double>::infinity());});p.run("raw clearance just above threshold",in,false);
  p.summaries([&](auto&s){s.raw_min_rho=std::nextafter(clear,0.);});p.run("raw clearance just below threshold",in,false);
  p.summaries([](auto&s){s.raw_min_rho=std::numeric_limits<double>::quiet_NaN();});p.run("NaN raw clearance same Unknown classification",in,true);
  p.summaries([](auto&s){s.raw_min_rho=std::numeric_limits<double>::infinity();});p.run("infinite raw clearance remains Unknown",in,true);
  p.summaries_far();p.run("finite raw clearance recovery",in,false);
  p.summaries([](auto&s){s.min_rho=ControllerLimits::local_range_m;});p.run("filled obstacle at range threshold",in,false);
  p.summaries([](auto&s){s.min_rho=std::nextafter(ControllerLimits::local_range_m,std::numeric_limits<double>::infinity());});p.run("filled range just above threshold",in,false);
  p.summaries([](auto&s){s.min_rho=std::numeric_limits<double>::quiet_NaN();});p.run("NaN filled summary is no near proof",in,true);
  p.summaries_far();p.run("equivalent far summary",in,true);
  const auto remote=p.cell(-35,15),added=p.cell(45.5,.5);p.data.fine_state[remote]=uint8_t(CandidateStatus::Unknown);p.data.fine_state[added]=uint8_t(CandidateStatus::Available);
  p.run("same node count different component mask",in,false,true,true);p.rectangle();p.run("component repair",in,false);
  p.data.fine_state[remote]=uint8_t(CandidateStatus::Unavailable);p.run("remote non-ROI obstacle",in,false);p.data.fine_state[remote]=uint8_t(CandidateStatus::Available);p.run("remote obstacle repair",in,false);
  p.region([](double a,double b){return a>-30&&a<30&&b>=0&&b<1;});p.run("single row bridge domain",in,false,true,false,true);
  p.data.fine_state[p.cell(.5,.5)]=uint8_t(CandidateStatus::Unknown);auto&cut=p.run("bridge removed exit clipped",in,false);need(cut.exit_clipped,"bridge removal did not clip unreachable exit");
  p.data.fine_state[p.cell(.5,.5)]=uint8_t(CandidateStatus::Available);p.run("bridge restored",in,false);
  p.rectangle();p.run("restore full domain",in,false);p.run("prime failure lifecycle",in,true);
  p.data.ready=false;p.run("not-ready rejects before hit",in,false,false,true);p.data.ready=true;
  auto&ready=p.run("not-ready repaired cold",in,false);need(!ready.coarse.reused,"not-ready retained topology certificate");
  p.data.fine_state[p.cell(-20.25,.5)]=uint8_t(CandidateStatus::Unknown);p.run("Unknown q0 rejects",in,false,false,true);
  p.data.fine_state[p.cell(-20.25,.5)]=uint8_t(CandidateStatus::Available);auto&known=p.run("Unknown q0 repaired graph cold",in,false);
  need(known.coarse.reused,"graph invalidation changed old topology reuse semantics");
  auto idle=in;idle.desired.valid=false;p.run("Idle rejects before hit",idle,false,false,true);
  auto&awake=p.run("Idle repaired graph cold",in,false);need(awake.coarse.reused,"Idle graph invalidation changed topology cache");
  p.region([&](double a,double b){return a>=-21&&a< -20&&b>=0&&b<1;});p.run("singleton component rejects",in,false,false,true);p.rectangle();p.run("singleton repaired graph cold",in,false);
  p.cached.invalidate_cache();p.oracle.invalidate_cache();p.run("explicit invalidate",in,false);
  ++p.data.geometry;p.run("geometry identity change",in,false);++p.data.configuration;p.run("input configuration change",in,false);
  ++in.coordinate_revision;p.run("coordinate identity change",in,false);in.R_WB={0,-1,0,1,0,0,0,0,1};p.run("rotation identity change",in,false,true,true,true);
  HarmonicConfig cfg;cfg.boundary_cap_deg=6;p.configure(cfg);p.summaries_far();p.rectangle();in=make_intent();p.run("boundary cap reconfigure",in,false);p.run("new cap graph hit",in,true,true,true);
  cfg.boundary_cap_deg=30;p.configure(cfg);p.summaries_far();p.rectangle();p.run("cap clamped by low-target separation",in,false);p.run("clamped cap repeat",in,true);
  auto cal=calibration();cal[0].K[0]+=1;p.configure({},cal);p.summaries_far();p.rectangle();p.run("camera calibration reconfigure",in,false);p.run("new calibration repeat",in,true);
  p.configure({},calibration(324,240),324,240);p.summaries_far();p.rectangle();p.run("image dimensions and source-table reconfigure",in,false);p.run("new image dimensions repeat",in,true,true,true);
  // Rejection after a valid graph/fine field must not populate the new cache.
  cfg={};cfg.max_curve_steps=1;p.configure(cfg);p.summaries_far();p.rectangle();auto&limited=p.run("curve step-limit after graph validation",in,false,false);
  need(limited.status!=FieldStatus::Valid&&limited.coarse.status==FieldStatus::TopologyReady&&limited.fine.status==FieldStatus::Valid,"step-limit fixture did not reach valid graph/fine field");
  auto&again=p.run("rejected curve cannot seed graph cache",in,false,false,true);need(again.coarse.reused,"rejected curve altered original topology cache semantics");
  // Optional numerical modes never read or write this graph cache.
  cfg={};cfg.force_coarse_solve=true;p.configure(cfg);p.summaries_far();p.rectangle();p.run("forced numerical mode cold",in,false);auto&numeric=p.run("forced numerical mode repeat",in,false,true,true);
  need(numeric.coarse.status==FieldStatus::Valid&&!numeric.coarse_topology_only&&numeric.coarse.reused&&numeric.coarse_graph_cache_ns==0,"numeric mode entered graph cache");
  cfg={};cfg.fine_neumann_roi=false;p.configure(cfg);p.summaries_far();p.rectangle();p.run("legacy coupling cold",in,false);auto&coupled=p.run("legacy coupling repeat",in,false,true,true,true);
  need(coupled.coarse.status==FieldStatus::Valid&&!coupled.coarse_topology_only&&coupled.coarse_graph_cache_ns==0,"legacy coupling entered graph cache");
  p.configure({},calibration(),320,240,true);p.summaries_far();p.rectangle();p.run("native IC0 fine backend cold",in,false);p.run("native IC0 fine backend graph hit",in,true,true,true,true);
}
}
int main(int argc,char**argv){try{
  need(argc<=2,"optional argument is export directory");graph_cache_cases(argc==2?std::filesystem::path(argv[1]):std::filesystem::path{});
  std::cout<<"PASS coarse graph cache: "<<comparisons<<" forced-rebuild oracle comparisons, "<<hits<<" strict graph hits, "<<poisoned_hits<<" poisoned-output hits; complete graph/metadata/fine/Q/3D equality, original-mask/FV checks, summary thresholds, failure lifecycle and native/numerical branches. Correctness only; no performance conclusion.\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
