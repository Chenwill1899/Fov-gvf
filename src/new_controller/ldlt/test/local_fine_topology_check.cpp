#include "new_controller/harmonic_planner.hpp"
#include "new_controller/harmonic_topology.hpp"
#include "new_controller/harmonic_io.hpp"
#include "new_controller/navigation_io.hpp"
#include <functional>
#include <iostream>
#include <sstream>

using namespace ego4p0;
namespace fs=std::filesystem;
namespace {
std::string context;
unsigned differential_cases=0,negative_cases=0,navigation_cases=0;
void need(bool ok,const char *why){if(!ok)throw std::runtime_error(context+": "+why);}
bool number_equal(double a,double b){return a==b||(std::isnan(a)&&std::isnan(b));}
template<class T> void prefix_equal(const std::vector<T>&a,const std::vector<T>&b,size_t count,const char*why){
  need(a.size()>=count&&b.size()>=count&&std::equal(a.begin(),a.begin()+count,b.begin()),why);
}
std::array<CameraCalibration,4> cameras(){
  std::array<CameraCalibration,4> result;
  for(unsigned c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);
    result[c].K={192,0,160,0,208,120,0,0,1};
    result[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};
    result[c].near_clip_z_m=.65; // Synthetic readback, never assigned to an archive.
  }
  return result;
}
HarmonicIntent intent(double a,double b,double da,double db){
  HarmonicIntent in;in.q0=sphere_point(a*direction_pi/180,b*direction_pi/180);
  in.desired={sphere_point(da*direction_pi/180,db*direction_pi/180),12,true,true};
  in.R_WB={1,0,0,0,1,0,0,0,1};return in;
}
void equal_curve(const CurveSample&a,const CurveSample&b){
  need(a.direction==b.direction&&a.gradient_B==b.gradient_B&&a.unit_B==b.unit_B,"Q vector differs");
  const double av[]={a.alpha,a.beta,a.F_alpha,a.F_beta,a.norm2,a.tau,a.zeta,a.potential,a.magnitude,a.unit_alpha,a.unit_beta};
  const double bv[]={b.alpha,b.beta,b.F_alpha,b.F_beta,b.norm2,b.tau,b.zeta,b.potential,b.magnitude,b.unit_alpha,b.unit_beta};
  for(size_t i=0;i<std::size(av);++i)need(number_equal(av[i],bv[i]),"Q scalar differs");
  need(a.cell==b.cell&&a.valid==b.valid&&a.local_chart==b.local_chart&&a.zero_field==b.zero_field,"Q status differs");
}
void topology_semantics(const HarmonicResult&r,const HarmonicPlanner&p){
  need(r.coarse_topology_only&&r.coarse.status==FieldStatus::TopologyReady,"coarse topology status missing");
  need(std::isnan(r.coarse.residual)&&r.coarse.iterations==0&&r.coarse.solve_ns==0&&r.coarse.solve_cpu_ns==0,"skipped coarse solve claims numerical evidence");
  need(r.coarse.assembly_ns==0&&r.coarse.iteration_ns==0&&r.coarse.residual_ns==0&&r.coarse.store_ns==0,"skipped coarse solve has numerical phases");
  for(double v:r.coarse.phi)need(std::isnan(v),"coarse diagnostic potential is not NaN");
  for(uint32_t i=0;i<r.adaptive.nodes;++i)need(std::isnan(r.adaptive.phi[i]),"active adaptive potential is not NaN");
  const auto&s=p.coarse_trial_stats();
  need(s.total_ns==0&&s.analysis_ns==0&&s.factor_ns==0&&s.solve_ns==0&&s.verify_ns==0&&s.unknowns==0,"skipped coarse solve called backend");
}
void equal_fields(const HarmonicResult&a,const HarmonicResult&b,bool compare_versions=true){
  need(a.status==b.status&&a.termination==b.termination&&a.curve_size==b.curve_size,"planner result differs");
  need(a.fine.status==b.fine.status&&a.fine.nodes==b.fine.nodes&&a.fine.boundary==b.fine.boundary&&a.fine.state==b.fine.state,"fine domain differs");
  need(a.fine.phi.size()==b.fine.phi.size(),"fine phi size differs");
  for(size_t i=0;i<a.fine.phi.size();++i)need(number_equal(a.fine.phi[i],b.fine.phi[i]),"fine phi differs");
  need(number_equal(a.fine.residual,b.fine.residual)&&a.fine.reused==b.fine.reused&&a.coarse.reused==b.coarse.reused&&a.reused==b.reused,"fine residual/cache decision differs");
  need(a.coarse.state==b.coarse.state&&a.coarse.boundary==b.coarse.boundary&&a.adaptive.nodes==b.adaptive.nodes&&a.adaptive.edges==b.adaptive.edges&&a.adaptive.owner==b.adaptive.owner,"coarse topology differs");
  const auto n=a.adaptive.nodes,e=a.adaptive.edges;
  prefix_equal(a.adaptive.boundary,b.adaptive.boundary,n,"adaptive labels differ");
  prefix_equal(a.adaptive.representative,b.adaptive.representative,n,"adaptive representatives differ");
  prefix_equal(a.adaptive.offset,b.adaptive.offset,n+1,"adaptive offsets differ");
  prefix_equal(a.adaptive.neighbor,b.adaptive.neighbor,e,"adaptive neighbors differ");
  prefix_equal(a.adaptive.weight,b.adaptive.weight,e,"adaptive weights differ");
  need(a.expanded==b.expanded&&a.exit_clipped==b.exit_clipped&&a.fine_attempts==b.fine_attempts&&a.angular_length==b.angular_length&&a.exit_direction==b.exit_direction,"route diagnostics differ");
  if(compare_versions)need(a.field_version==b.field_version&&a.curve_version==b.curve_version&&a.version==b.version,"version/cache lifecycle differs");
  for(uint32_t i=0;i<a.curve_size;++i)equal_curve(a.curve[i],b.curve[i]);
}
void independent_fine_checks(const HarmonicResult&r,const HarmonicDepthInput&d,const HarmonicPlanner&p){
  if(r.fine.status!=FieldStatus::Valid)return;
  const auto&s=p.fine_stencil();double maximum=0.;
  for(uint32_t i=0;i<s.nodes.size();++i){
    if(!r.fine.boundary[i])continue;
    need(d.fine_state[i]==uint8_t(CandidateStatus::Available),"fine node lacks original Available evidence");
    const double phi=r.fine.phi[i];need(std::isfinite(phi)&&phi>=-1e-7&&phi<=1+1e-7,"fine potential outside maximum principle");
    if(p.config().fine_neumann_roi)need(FieldBoundary(r.fine.boundary[i])!=FieldBoundary::Coupling,"local fine domain reads coarse potential");
    if(FieldBoundary(r.fine.boundary[i])!=FieldBoundary::Interior)continue;
    double residual=0,diagonal=0;
    for(unsigned k=0;k<4;++k){const auto j=s.nodes[i].neighbor[k];if(j==UINT32_MAX||!r.fine.boundary[j])continue;
      const double w=s.nodes[i].weight[k];residual+=w*(phi-r.fine.phi[j]);diagonal+=w;}
    need(diagonal>0,"isolated fine Interior node");maximum=std::max(maximum,std::abs(residual)/diagonal);
  }
  need(maximum<=p.config().residual_tolerance,"independent finite-volume residual failed");
  FieldLayer mask=r.fine;
  for(uint32_t i=0;i<s.nodes.size();++i)mask.boundary[i]=d.fine_state[i]==uint8_t(CandidateStatus::Available)?uint8_t(FieldBoundary::Interior):0;
  for(uint32_t k=1;k<r.curve_size;++k){
    need(FineCurveIntegrator::segment_clear(s,mask,r.curve[k-1].direction,r.curve[k].direction),"Q crosses original Unknown/obstacle mask");
    need(r.curve[k].zeta>=r.curve[k-1].zeta,"Q arc length decreased");
  }
}
void equal_spatial(const SpatialReference&a,const SpatialReference&b){
  need(a.valid()==b.valid(),"spatial acceptance differs");if(!a.valid())return;
  need(a.nodes()==b.nodes()&&a.profile().length==b.profile().length,"spatial knot/length differs");
  for(unsigned i=0;i<=10;++i){const double s=(a.profile().length+1)*i/10.;const auto x=a.query(s),y=b.query(s);
    need(x.position_W==y.position_W&&x.tangent_W==y.tangent_W&&x.curvature_W==y.curvature_W,"3D spatial geometry differs");
    need(x.zeta==y.zeta&&x.kappa==y.kappa&&x.dkappa_ds==y.dkappa_ds&&x.position_error_bound_m==y.position_error_bound_m,"spatial derivatives/certificate differ");
  }
}
void compare_spatial_build(const HarmonicResult&a,const HarmonicResult&b){
  if(a.status!=FieldStatus::Valid)return;
  std::vector<Direction3>qa,qb;for(uint32_t i=0;i<a.curve_size;++i){qa.push_back(a.curve[i].direction);qb.push_back(b.curve[i].direction);}
  SpatialReference sa,sb;const auto va=sa.build(qa,a.intent.p_W,a.intent.R_WB),vb=sb.build(qb,b.intent.p_W,b.intent.R_WB);
  need(va==vb,"continuous spatial build acceptance differs");equal_spatial(sa,sb);
}
struct PlannerPair {
  HarmonicPlanner fast,old;HarmonicDepthInput data;HarmonicResult a,b;
  PlannerPair(){configure({});fast.allocate(data,320,240);data.ready=true;data.sequence=data.configuration=data.geometry=1;
    for(auto&camera:data.coarse_image)for(auto&v:camera){v.total=v.valid=v.raw_valid=144;v.mean_z=v.min_rho=v.raw_min_rho=10.;}}
  void configure(HarmonicConfig cfg){cfg.force_coarse_solve=false;fast.configure(cameras(),320,240,cfg,4);fast.set_trial_solvers(SparseTrialMode::EigenLDLT,true);fast.allocate(a);
    cfg.force_coarse_solve=true;old.configure(cameras(),320,240,cfg,4);old.set_trial_solvers(SparseTrialMode::EigenLDLT,true);old.allocate(b);}
  void region(const std::function<bool(double,double)>&available){const auto&s=fast.fine_stencil();
    for(uint32_t i=0;i<s.nodes.size();++i){const double a=-180+(i%360+.5),b=-90+(i/360+.5);const bool ok=available(a,b);
      data.fine_state[i]=uint8_t(ok?CandidateStatus::Available:CandidateStatus::Unknown);data.cameras[i]=ok?1:0;}}
  void run(const std::string&name,const HarmonicIntent&in,bool cold=false,bool require_valid=true,bool spatial=false){
    context=name;if(cold){fast.invalidate_cache();old.invalidate_cache();}++data.sequence;fast.build(data,in,a);old.build(data,in,b);equal_fields(a,b);
    if(require_valid)need(a.status==FieldStatus::Valid&&a.termination==CurveEnd::Exit,"valid fixture rejected");
    if(a.adaptive.nodes&&a.coarse.status==FieldStatus::TopologyReady){topology_semantics(a,fast);need(b.coarse.status==FieldStatus::Valid&&std::isfinite(b.coarse.residual),"forced numerical oracle was not solved");}
    independent_fine_checks(a,data,fast);if(spatial)compare_spatial_build(a,b);++differential_cases;
  }
};
void planner_checks(const fs::path&output){
  PlannerPair pair;
  auto rectangle=[&]{pair.region([](double a,double b){return a>=-45&&a<45&&b>=-20&&b<20;});};
  rectangle();auto in=intent(-20,.5,20,.5);pair.run("open cold",in,true,true,true);need(!pair.a.coarse.reused&&!pair.a.fine.reused,"cold reused");
  fs::create_directories(output/"topology");save_harmonic(output/"topology",pair.a);
  fs::create_directories(output/"numeric_oracle");save_harmonic(output/"numeric_oracle",pair.b);
  pair.run("identical new group",in);need(pair.a.reused&&pair.a.coarse.reused&&pair.a.fine.reused,"exact cache missed");
  pair.run("explicit cold roundtrip",in,true);need(!pair.a.reused&&!pair.a.coarse.reused,"explicit cold failed");
  in.q0=sphere_point(-19.9*direction_pi/180,.5*direction_pi/180);pair.run("q0 subcell change",in);need(!pair.a.reused,"q0 change reused Q");
  in.desired.d_desired=sphere_point(20.2*direction_pi/180,.5*direction_pi/180);pair.run("desired subcell change",in);need(!pair.a.reused,"desired change reused Q");
  in.p_W={1,-2,3};pair.run("world translation",in,false,true,true);need(pair.a.reused,"translation invalidated angular shape");
  in.R_WB={0,-1,0,1,0,0,0,0,1};pair.run("rotation identity change",in,false,true,true);need(!pair.a.coarse.reused,"rotation did not invalidate topology");
  ++in.coordinate_revision;pair.run("coordinate revision",in);need(!pair.a.coarse.reused,"coordinate revision did not invalidate topology");
  ++pair.data.geometry;pair.run("geometry revision",in);need(!pair.a.coarse.reused,"geometry revision did not invalidate topology");
  ++pair.data.configuration;pair.run("input configuration revision",in);need(!pair.a.coarse.reused,"configuration did not invalidate topology");
  // Changing a cell outside the local corridor must invalidate its global
  // topology certificate even when the local solution can be reused.
  const auto remote=pair.fast.fine_stencil().cell(sphere_point(-35*direction_pi/180,15*direction_pi/180));
  pair.data.fine_state[remote]=uint8_t(CandidateStatus::Unknown);pair.run("Omega cell outside ROI removed",in);need(!pair.a.coarse.reused,"changed component hit topology cache");
  pair.data.fine_state[remote]=uint8_t(CandidateStatus::Available);pair.run("Omega repair",in);need(!pair.a.coarse.reused,"Omega repair reused old topology");
  const auto start=pair.fast.fine_stencil().cell(in.q0);pair.data.fine_state[start]=uint8_t(CandidateStatus::Unknown);
  pair.run("Unknown q0",in,false,false);need(pair.a.status==FieldStatus::NeedsRefinement&&pair.a.curve_size==0&&!pair.a.reused,"Unknown start leaked Q");
  pair.data.fine_state[start]=uint8_t(CandidateStatus::Available);pair.run("Unknown to valid",in);need(!pair.a.reused,"Unknown recovery reused stale Q");
  pair.data.ready=false;pair.run("not ready",in,false,false);need(pair.a.status==FieldStatus::NoInput&&pair.a.curve_size==0,"missing input accepted");
  pair.data.ready=true;pair.run("invalid to valid cold",in);need(!pair.a.coarse.reused&&!pair.a.fine.reused,"invalid input poisoned cold recovery");
  HarmonicConfig cfg;cfg.corridor_deg=4;cfg.revision++;pair.configure(cfg);pair.run("solver configuration reconfigure",in);need(!pair.a.coarse.reused,"new configuration reused topology");
  pair.configure({});rectangle();pair.run("aligned",intent(20,.5,20,.5),true);need(pair.a.curve_size==1,"aligned path fabricated motion");
  pair.region([](double a,double b){return a>=-45&&a<45&&b>=0&&b<1;});pair.run("one row narrow",intent(-20,.5,20,.5),true,true,true);
  for(int gap=-12;gap<=12;gap+=3){rectangle();for(uint32_t i=0;i<pair.data.fine_state.size();++i){const double a=-180+(i%360+.5),b=-90+(i/360+.5);
      if(std::abs(a)<3&&!(b>=gap&&b<gap+3))pair.data.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
    pair.run("single wall gap "+std::to_string(gap),intent(-25,.5,25,.5),true,true,gap==0);}
  for(int shift=-6;shift<=6;shift+=3){rectangle();for(uint32_t i=0;i<pair.data.fine_state.size();++i){const double a=-180+(i%360+.5),b=-90+(i/360+.5);
      if((std::abs(a+12)<2&&!(b>=shift+5&&b<shift+9))||(std::abs(a-8)<2&&!(b>=shift-9&&b<shift-5)))pair.data.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
    pair.run("double wall "+std::to_string(shift),intent(-30,.5,30,.5),true,true,shift==0);}
  pair.region([](double a,double b){return std::abs(a)>135&&std::abs(b)<20;});pair.run("azimuth seam",intent(160,.5,-160,8.5),true,true,true);
  pair.region([](double,double b){return b>65;});pair.run("polar cap",intent(-35,78,45,80),true,false,true);
  need(pair.a.fine.status==FieldStatus::Valid,"polar fine field unsolved");
  rectangle();pair.run("nonplanar climb",intent(-20,-12,20,12),true,true,true);
  need(pair.a.curve.front().direction[2]<0&&pair.a.curve[pair.a.curve_size-1].direction[2]>0,"3D climb was flattened");
  pair.region([](double a,double b){return std::abs(b)<10&&(a< -5&&a> -45||a>5&&a<45);});
  pair.run("disconnected requested exit",intent(-20,.5,20,.5),true);need(pair.a.exit_clipped,"unreachable desired exit was not clipped");
  pair.region([](double,double){return false;});pair.run("all Unknown",in,true,false);need(pair.a.curve_size==0,"all Unknown fabricated Q");
  rectangle();auto idle=in;idle.desired.valid=false;pair.run("missing desired",idle,true,false);need(pair.a.status==FieldStatus::Idle,"missing desired was not idle");
  pair.run("desired recovery",in);need(!pair.a.coarse.reused,"idle cold recovery reused topology");
  // The alternative coarse-Dirichlet model still needs its numerical field.
  cfg={};cfg.fine_neumann_roi=false;pair.configure(cfg);pair.run("legacy coarse coupling",intent(-20,.5,20,.5),true,true,true);
  need(!pair.a.coarse_topology_only&&pair.a.coarse.status==FieldStatus::Valid&&std::isfinite(pair.a.coarse.residual)&&pair.fast.coarse_trial_stats().total_ns>0&&pair.fast.coarse_trial_stats().unknowns>0,"legacy coupling skipped coarse solve");
  need(std::count(pair.a.fine.boundary.begin(),pair.a.fine.boundary.end(),uint8_t(FieldBoundary::Coupling))>0,"legacy fixture has no coarse coupling");
  pair.configure({});pair.fast.set_native_preconditioner(CoarsePreconditioner::IC0);pair.old.set_native_preconditioner(CoarsePreconditioner::IC0);
  rectangle();pair.run("native local cold",intent(-20,.5,20,.5),true,true,true);
  pair.run("native local exact reuse",intent(-20,.5,20,.5));need(pair.a.reused,"native exact cache missed");
  pair.run("native local climb",intent(-20,-12,20,12),true,true,true);
  pair.region([](double a,double b){return a>=-45&&a<45&&b>=0&&b<1;});pair.run("native one row narrow",intent(-20,.5,20,.5),true,true,true);
  cfg={};cfg.fine_neumann_roi=false;pair.configure(cfg);pair.fast.set_native_preconditioner(CoarsePreconditioner::IC0);pair.old.set_native_preconditioner(CoarsePreconditioner::IC0);
  rectangle();pair.run("native coarse coupling",intent(-20,.5,20,.5),true,true,true);
  need(pair.a.coarse.status==FieldStatus::Valid&&pair.a.coarse.iterations>0&&!pair.a.coarse_topology_only,"native coupling skipped numerical solve");
}
struct Graph {
  AdaptiveCoarseField a;std::vector<uint8_t>component{1,1,1,1,1,1,1,0};std::vector<uint32_t>active{0,1,2,3,4,5,6};
  Graph(){a.allocate(8);a.nodes=6;a.owner={0,0,1,2,3,4,5,UINT32_MAX};
    for(uint32_t i=0;i<a.nodes;++i){a.representative[i]=i?i+1:0;a.boundary[i]=uint8_t(i==0?FieldBoundary::Low:i==5?FieldBoundary::Exit:FieldBoundary::Interior);a.offset[i]=a.edges;
      if(i){a.neighbor[a.edges]=i-1;a.weight[a.edges++]=1;}
      if(i+1<a.nodes){a.neighbor[a.edges]=i+1;a.weight[a.edges++]=1;}}
    a.offset[a.nodes]=a.edges;
  }
};
void validator_checks(){
  CoarseTopologyValidator validator;validator.configure(8);Graph good;FieldLayer layer;
  auto valid=[&](const Graph&g,bool reused){need(validator.validate(g.a,g.component,g.active,layer),validator.error());
    need(layer.status==FieldStatus::TopologyReady&&layer.reused==reused&&std::isnan(layer.residual)&&layer.iterations==0&&layer.solve_ns==0&&layer.solve_cpu_ns==0,"topology certificate/cache semantics wrong");};
  context="validator initial";valid(good,false);valid(good,true);
  auto reject=[&](const char*name,const std::function<void(Graph&)>&change){context=name;auto bad=good;change(bad);
    need(!validator.validate(bad.a,bad.component,bad.active,layer),"malformed graph accepted");
    need(layer.status==FieldStatus::SolveFailed&&!layer.reused&&std::isnan(layer.residual)&&std::string(validator.error())!="none","failure retained valid certificate");
    valid(good,false);valid(good,true);++negative_cases;};
  reject("short owner",[](Graph&g){g.a.owner.pop_back();});
  reject("short offsets",[](Graph&g){g.a.offset.resize(g.a.nodes);});
  reject("offset first",[](Graph&g){g.a.offset[0]=1;});
  reject("offset last",[](Graph&g){g.a.offset[g.a.nodes]--;});
  reject("offset backwards",[](Graph&g){g.a.offset[2]=g.a.edges+1;});
  reject("short weights",[](Graph&g){g.a.weight.resize(g.a.edges-1);});
  reject("nodes exceed configured maximum",[](Graph&g){g.a.nodes=9;});
  reject("missing active owner",[](Graph&g){g.a.owner[2]=UINT32_MAX;});
  reject("owner outside component",[](Graph&g){g.a.owner[7]=0;});
  reject("invalid component bit",[](Graph&g){g.component[0]=2;});
  reject("short active list",[](Graph&g){g.active.pop_back();});
  reject("duplicate active list",[](Graph&g){g.active[1]=0;});
  reject("inactive active list",[](Graph&g){g.active.back()=7;});
  reject("representative out of bounds",[](Graph&g){g.a.representative[0]=8;});
  reject("representative wrong owner",[](Graph&g){g.a.representative[0]=2;});
  reject("missing Low",[](Graph&g){g.a.boundary[0]=uint8_t(FieldBoundary::Interior);});
  reject("missing Exit",[](Graph&g){g.a.boundary[5]=uint8_t(FieldBoundary::Interior);});
  reject("invalid boundary enum",[](Graph&g){g.a.boundary[2]=255;});
  reject("Coupling not topology boundary",[](Graph&g){g.a.boundary[2]=uint8_t(FieldBoundary::Coupling);});
  reject("neighbor out of bounds",[](Graph&g){g.a.neighbor[0]=g.a.nodes;});
  reject("self edge",[](Graph&g){g.a.neighbor[0]=0;});
  reject("duplicate edge",[](Graph&g){g.a.neighbor[g.a.offset[1]+1]=g.a.neighbor[g.a.offset[1]];});
  reject("unsorted edge",[](Graph&g){std::swap(g.a.neighbor[g.a.offset[1]],g.a.neighbor[g.a.offset[1]+1]);});
  reject("negative weight",[](Graph&g){g.a.weight[0]=-1;});
  reject("zero weight",[](Graph&g){g.a.weight[0]=0;});
  reject("NaN weight",[](Graph&g){g.a.weight[0]=std::numeric_limits<double>::quiet_NaN();});
  reject("infinite weight",[](Graph&g){g.a.weight[0]=std::numeric_limits<double>::infinity();});
  reject("finite weights overflowing degree",[](Graph&g){g.a.weight[g.a.offset[1]]=g.a.weight[g.a.offset[1]+1]=std::numeric_limits<double>::max();});
  reject("asymmetric weight",[](Graph&g){g.a.weight[0]=2;});
  reject("missing reverse edge",[](Graph&g){g.a.neighbor[g.a.offset[1]]=3;std::swap(g.a.neighbor[g.a.offset[1]],g.a.neighbor[g.a.offset[1]+1]);});
  reject("disconnected ungrounded island",[](Graph&g){
    // Three two-node components, including an Interior-only middle island.
    g.a.edges=6;for(uint32_t i=0;i<6;++i){g.a.offset[i]=i;g.a.neighbor[i]=i^1;g.a.weight[i]=1;}g.a.offset[6]=6;});
  // Valid changes remain strict misses even if the fine ROI may be unchanged.
  context="valid owner mutation";auto changed=good;changed.a.owner[1]=1;valid(changed,false);valid(changed,true);valid(good,false);
  context="valid representative mutation";changed=good;changed.a.representative[0]=1;valid(changed,false);valid(changed,true);valid(good,false);
  context="symmetric weight mutation";changed=good;changed.a.weight[0]=changed.a.weight[1]=2;valid(changed,false);valid(changed,true);valid(good,false);
  context="valid boundary mutation";changed=good;changed.a.boundary[2]=uint8_t(FieldBoundary::Low);valid(changed,false);valid(changed,true);valid(good,false);
  context="irrelevant phi cannot certify numerical field";changed=good;std::fill(changed.a.phi.begin(),changed.a.phi.end(),std::numeric_limits<double>::infinity());valid(changed,true);
  context="explicit certificate invalidation";validator.invalidate();valid(good,false);valid(good,true);
}
void pipeline_checks(const fs::path&output){
  NavigationPipeline fast,old;NavigationObservation a,b;NavigationState state;DesiredMotion desired{{1,0,0},12,true,true};auto cal=cameras();
  auto configure=[&](uint64_t revision){fast.configure(cal,320,240,revision);old.configure(cal,320,240,revision);
    HarmonicConfig cfg;cfg.force_coarse_solve=true;old.planner().configure(cal,320,240,cfg,4);old.planner().set_trial_solvers(SparseTrialMode::EigenLDLT,true);
    fast.allocate(a);old.allocate(b);for(auto*o:{&a,&b})for(auto&d:o->depth.D)std::fill(d.begin(),d.end(),10.f);};
  auto group=[&](uint64_t n,uint64_t revision=1){navigation_identity(a,n,revision);navigation_identity(b,n,revision);state.actual.t_ns=a.depth.t_ns;state.current_simulation_ns=a.depth.t_ns;};
  configure(1);group(1);state.actual_valid=true;state.actual.v_B={12,0,0};
  auto run=[&](const char*name,bool cold=false){context=name;fast.process(a,state,desired,cold,true);old.process(b,state,desired,cold,true);const auto&x=fast.output();const auto&y=old.output();
    need(x.invalid==y.invalid&&x.input_ready==y.input_ready&&x.angular_ready==y.angular_ready&&x.spatial_ready==y.spatial_ready,"navigation acceptance differs");
    need(x.field_version==y.field_version&&x.curve_version==y.curve_version&&x.geometry_version==y.geometry_version&&x.spatial_reused==y.spatial_reused&&x.occupancy_reused==y.occupancy_reused,"navigation cache/version differs");
    need(x.observed_sections==y.observed_sections&&x.unknown_sections==y.unknown_sections&&x.obstacle_sections==y.obstacle_sections&&x.outside_sections==y.outside_sections,"tube classifications differ");
    if(x.field_attempted){equal_fields(fast.field(),old.field());if(fast.field().coarse.status==FieldStatus::TopologyReady)topology_semantics(fast.field(),fast.planner());}
    if(x.spatial_ready)equal_spatial(fast.reference(),old.reference());
    const auto&ts=fast.tube().sections();const auto&us=old.tube().sections();need(ts.size()==us.size(),"tube section count differs");
    for(size_t i=0;i<ts.size();++i)need(ts[i].s0==us[i].s0&&ts[i].s1==us[i].s1&&ts[i].cover_radius_m==us[i].cover_radius_m&&ts[i].status==us[i].status&&ts[i].cover_center_W==us[i].cover_center_W,"continuous tube certificate differs");
    if(!x.angular_ready)need(!x.spatial_ready&&ts.empty(),"failed navigation retained accepted spatial curve/tube");
    // A failed spatial-domain certificate may retain Q as rejected diagnostic
    // evidence. Missing input must instead clear the previous angular result.
    if(!x.input_ready)need(fast.field().status!=FieldStatus::Valid&&fast.field().curve_size==0,"missing input retained previous Q");
    ++navigation_cases;};
  run("synthetic known-near-plane clear");need(fast.output().spatial_ready&&fast.output().observed_sections>0,"positive case has no raw-certified clear tube");
  RawDepthBallOracle raw(a.depth,fast.builder().local_geometry(),a.observation_pose);
  need(raw({2,0,.02},.30)==TubeStatus::ObservedClear,"synthetic clear ball not certified");
  need(raw({.5,0,.02},.30)==TubeStatus::Unknown,"near-plane blind region incorrectly certified");
  need(raw({4.1,0,.02},.30)==TubeStatus::OutsideObservationRange,"outside-range ball not rejected");
  fs::create_directories(output/"pipeline_clear");navigation_export(output/"pipeline_clear",fast);save_harmonic(output/"pipeline_clear",fast.field());
  run("same group exact reuse");need(fast.field().reused&&fast.output().occupancy_reused,"same group did not reuse");
  run("pipeline explicit cold",true);need(!fast.field().reused,"pipeline cold ignored");
  group(2);run("new group equal raw");need(fast.field().reused&&!fast.output().occupancy_reused,"new raw evidence reuse contract wrong");
  state.actual.p_W={.01,0,.02};run("pipeline anchor translation");need(fast.field().reused&&fast.output().spatial_reused&&!fast.output().occupancy_reused,"translation did not separate angular shape and proof");
  state.actual.v_B=ref_scale(sphere_point(2*direction_pi/180,0),12);run("pipeline q0 change");
  desired.d_desired=sphere_point(12*direction_pi/180,5*direction_pi/180);run("pipeline 3D desired change");need(fast.output().spatial_ready,"3D pipeline positive case rejected");
  state.actual.R_WB={0,-1,0,1,0,0,0,0,1};state.actual.v_B={0,-12,0};run("pipeline actual rotation");
  state.actual.R_WB={1,0,0,0,1,0,0,0,1};state.actual.v_B={12,0,0};
  ++state.configuration_version;run("pipeline wrong config");need(fast.output().invalid==NavigationInvalid::ConfigurationMismatch,"bad config accepted");
  state.configuration_version=NavigationConfig::version;run("pipeline config repair");need(!fast.field().coarse.reused,"bad config did not clear cache");
  a.identity.calibration_version=b.identity.calibration_version=2;run("pipeline wrong calibration");need(fast.output().invalid==NavigationInvalid::CalibrationMismatch,"bad calibration accepted");
  a.identity.calibration_version=b.identity.calibration_version=1;run("pipeline calibration repair");need(!fast.field().coarse.reused,"bad calibration did not clear cache");
  state.actual.R_WB[0]=std::numeric_limits<double>::quiet_NaN();run("pipeline nonfinite rotation");need(fast.output().invalid==NavigationInvalid::InvalidState,"NaN rotation accepted");
  state.actual.R_WB[0]=1;run("pipeline rotation repair");
  // Raw and filled near obstacles must override all free-space evidence.
  for(auto*o:{&a,&b})for(auto&d:o->depth.D)std::fill(d.begin(),d.end(),.8f);group(3);run("pipeline raw obstacle");
  RawDepthBallOracle obstacle(a.depth,fast.builder().local_geometry(),a.observation_pose);
  need(obstacle({2,0,.02},.30)==TubeStatus::PotentialObstacle,"raw obstacle was not preserved");
  for(auto*o:{&a,&b})for(auto&d:o->depth.D)std::fill(d.begin(),d.end(),std::numeric_limits<float>::quiet_NaN());group(4);run("pipeline all raw Unknown");
  RawDepthBallOracle unknown(a.depth,fast.builder().local_geometry(),a.observation_pose);need(unknown({2,0,.02},.30)==TubeStatus::Unknown,"missing raw pixels invented clearance");
  for(auto*o:{&a,&b})for(auto&d:o->depth.D)std::fill(d.begin(),d.end(),10.f);group(5);run("pipeline raw evidence recovery");need(fast.output().spatial_ready,"raw repair did not recover");
  cal[0].K[0]+=1;configure(2);group(6,2);run("pipeline calibration matrix reconfigure");need(fast.output().spatial_ready&&!fast.field().coarse.reused,"calibration reconfigure reused stale domain");
}
} // namespace
int main(int argc,char**argv){try{
  need(argc==2,"expected export directory");const fs::path output=argv[1];fs::create_directories(output);
  validator_checks();planner_checks(output);pipeline_checks(output);
  std::cout<<"PASS local topology: "<<differential_cases<<" same-code numerical-oracle cases, "<<negative_cases<<" invalid-domain/cache-repair cases, "<<navigation_cases<<" full-navigation comparisons; exact fine/Q/3D/tube equality, independent FV/mask checks, NaN unsolved-coarse semantics. Correctness only; no performance conclusion.\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
