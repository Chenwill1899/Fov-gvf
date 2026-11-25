#include "new_controller/harmonic_planner.hpp"
#include <iostream>
#include <cstdlib>
#include <atomic>
#include <new>
using namespace ego4p0;
static std::atomic<bool>counting{false};static std::atomic<size_t>allocations{0};
void *operator new(std::size_t n){if(counting)++allocations;if(auto *p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void *p)noexcept{std::free(p);}void operator delete(void *p,std::size_t)noexcept{std::free(p);}
void *operator new[](std::size_t n){return ::operator new(n);}void operator delete[](void *p)noexcept{::operator delete(p);}void operator delete[](void *p,std::size_t)noexcept{::operator delete(p);}
static void require(bool b,const char *why){if(!b)throw std::runtime_error(why);}
static std::array<CameraCalibration,4> cameras(uint32_t w,uint32_t h) {
  std::array<CameraCalibration,4> cal;for(uint32_t c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);cal[c].K={.6*w,0,w*.5,0,.65*w,h*.5,0,0,1};cal[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};}return cal;
}
static void fill_far(HarmonicDepthInput &d){for(auto &v:d.coarse_image)for(auto &s:v){s.mean_z=10;s.min_rho=s.raw_min_rho=10;s.valid=s.raw_valid=s.total=81;}d.ready=true;d.sequence=d.configuration=d.geometry=1;d.stamp_ns=2000000;}
static HarmonicIntent intent(double start,double target){HarmonicIntent i;i.q0=sphere_point(start*direction_pi/180,0);i.desired={sphere_point(target*direction_pi/180,0),1.,true,true};i.R_WB={1,0,0,0,1,0,0,0,1};return i;}
int main(){try{
  HarmonicConfig cfg;cfg.fine_neumann_roi=false;uint64_t checks=0;
  for(double degrees:{3.,1.}) {
    SphericalStencil s;s.configure(degrees);FieldLayer f;f.allocate(s.layout);HarmonicLinearSolver solver;solver.configure(s);
    const auto na=s.layout.azimuth_bins;
    for(uint32_t b=0;b<s.layout.elevation_bins;++b)for(uint32_t a=0;a<na;++a) {
      const double alpha=-180+(a+.5)*degrees,beta=-90+(b+.5)*degrees;const auto id=b*na+a;
      if(alpha< -30||alpha>30||std::abs(beta)>45)continue;
      f.boundary[id]=uint8_t(FieldBoundary::Interior);if(a==uint32_t(150/degrees)){f.boundary[id]=uint8_t(FieldBoundary::Low);f.phi[id]=0;}
      if(a==uint32_t(210/degrees)-1){f.boundary[id]=uint8_t(FieldBoundary::Exit);f.phi[id]=1;}
    }
    solver.solve(f,cfg);require(f.status==FieldStatus::Valid && f.residual<=cfg.residual_tolerance,"manufactured spherical solve failed");
    double error=0;for(uint32_t i=0;i<f.phi.size();++i)if(f.boundary[i]){const double exact=(double(i%na)-150/degrees)/(60/degrees-1);error=std::max(error,std::abs(exact-f.phi[i]));++checks;}
    require(error<2e-5,"manufactured affine-alpha solution inaccurate");
    solver.solve(f,cfg);require(f.reused&&f.iterations==0,"unchanged operator not reused");
    solver.invalidate();for(uint32_t i=0;i<f.phi.size();++i)if(FieldBoundary(f.boundary[i])==FieldBoundary::Interior)f.phi[i]=.5;auto fail=cfg;fail.max_iterations=1;solver.solve(f,fail);require(f.status==FieldStatus::SolveFailed,"iteration cap falsely reported convergence");
    // Derivative of phi=alpha: F_alpha=1/cos^2(beta), F_beta=0.
    for(uint32_t i=0;i<f.phi.size();++i){f.boundary[i]=uint8_t(FieldBoundary::Interior);f.state[i]=uint8_t(CandidateStatus::Available);f.phi[i]=-direction_pi+(i%na+.5)*s.layout.azimuth_step;}
    f.status=FieldStatus::Valid;auto g=FineCurveIntegrator::gradient(s,f,sphere_point(.1,.5));
    std::cout<<"metric degrees="<<degrees<<" F="<<g.F_alpha<<","<<g.F_beta<<" norm="<<g.norm2<<" expected="<<1/std::pow(std::cos(g.beta),2)<<"\n";
    require(g.valid&&std::abs(g.F_alpha*std::pow(std::cos(g.beta),2)-1)<.02 && std::abs(g.F_beta)<.003,"sphere metric gradient wrong");
  }
  // Nonconstant latitude solution exercises the -tan(beta)*phi_beta term.
  {
    SphericalStencil s;s.configure(3.);FieldLayer f;f.allocate(s.layout);HarmonicLinearSolver solver;solver.configure(s);
    const auto na=s.layout.azimuth_bins;const uint32_t lo=10,hi=49;
    for(uint32_t b=lo;b<=hi;++b)for(uint32_t a=0;a<na;++a){const auto id=b*na+a;f.boundary[id]=uint8_t(b==lo ? FieldBoundary::Low : b==hi ? FieldBoundary::Exit : FieldBoundary::Interior);if(b==lo)f.phi[id]=0;if(b==hi)f.phi[id]=1;}
    solver.solve(f,cfg);require(f.status==FieldStatus::Valid,"latitude spherical solve failed");
    auto potential=[](double b){return std::atanh(std::sin(b));};const auto bottom=potential((-90+(lo+.5)*3)*direction_pi/180),top=potential((-90+(hi+.5)*3)*direction_pi/180);
    for(uint32_t b=lo;b<=hi;++b) {const auto exact=(potential((-90+(b+.5)*3)*direction_pi/180)-bottom)/(top-bottom);require(std::abs(f.phi[b*na]-exact)<.001,"latitude metric omitted/drifted");}
    std::fill(f.boundary.begin(),f.boundary.end(),uint8_t(FieldBoundary::Interior));solver.invalidate();solver.solve(f,cfg);require(f.status==FieldStatus::SolveFailed,"pure Neumann field accepted without potential anchor");
    f.boundary[lo*na]=uint8_t(FieldBoundary::Exit);f.phi[lo*na]=1;solver.solve(f,cfg);require(f.status==FieldStatus::SolveFailed&&f.constant_boundary,"one constant exit plus Neumann accepted as nonconstant guidance");
  }
  // Weighted second image summary, including edge blocks; no original image needed.
  DepthFrame frame;DirectionGridLayout layout;layout.configure(1,1);frame.observations.allocate(10,11,layout,16,3);frame.local.allocate(layout,true);frame.local.ready=frame.observations.ready=true;frame.local.virtual_center=true;
  for(auto &v:frame.observations.blocks)for(uint32_t i=0;i<v.size();++i){auto &b=v[i];b.mean_z=float(i+1);b.valid_pixels=(i%3)+1;b.raw_valid_pixels=1;b.min_rho=i+2;b.raw_min_rho=i+3;}
  HarmonicDepthInput summary;summary.allocate(10,11,layout,3);summary.capture(frame);
  require(summary.ready&&summary.columns==2&&summary.rows==2,"edge summary shape wrong");
  double weighted=0;uint32_t valid=0;for(uint32_t y=0;y<3;++y)for(uint32_t x=0;x<3;++x){const auto &b=frame.observations.blocks[0][y*4+x];weighted+=b.mean_z*b.valid_pixels;valid+=b.valid_pixels;}
  require(summary.coarse_image[0][0].total==81&&summary.coarse_image[0][3].total==2&&summary.coarse_image[0][0].valid==valid&&summary.coarse_image[0][0].raw_valid==9&&summary.coarse_image[0][0].mean_z==weighted/valid&&summary.coarse_image[0][0].min_rho==2,"weighted/raw/min summary incorrect");
  summary.capture(frame);for(auto &v:frame.observations.blocks)for(auto &b:v)b=BlockDepth{};summary.capture(frame);require(std::isnan(summary.coarse_image[0][0].mean_z)&&summary.coarse_image[0][0].valid==0,"empty summary fabricated zero depth");
  HarmonicPlanner planner;planner.configure(cameras(320,240),320,240,cfg,3);HarmonicDepthInput d;planner.allocate(d,320,240);fill_far(d);HarmonicResult r;planner.allocate(r);
  const auto &s=planner.fine_stencil();const auto na=s.layout.azimuth_bins;
  auto region=[&](double left,double right,double bottom,double top) {std::fill(d.fine_state.begin(),d.fine_state.end(),0);for(uint32_t i=0;i<d.fine_state.size();++i){const double a=-180+(i%na+.5),b=-90+(i/na+.5);if(a>=left&&a<right&&b>=bottom&&b<top){d.fine_state[i]=uint8_t(CandidateStatus::Available);d.cameras[i]=1;}}};
  region(-45,45,-20,20);auto in=intent(-20,20);planner.build(d,in,r);
  std::cout<<"open: "<<field_status_name(r.status)<<' '<<curve_end_name(r.termination)<<" iterations="<<r.coarse.iterations<<'/'<<r.fine.iterations<<" residual="<<r.fine.residual<<" steps="<<r.integration_steps<<'\n';
  require(r.status==FieldStatus::Valid&&r.termination==CurveEnd::Exit,"open reference failed");
  for(uint32_t k=1;k<r.curve_size;++k)require(FineCurveIntegrator::segment_clear(s,r.fine,r.curve[k-1].direction,r.curve[k].direction),"curve jumped unknown");
  {
    HarmonicResult aligned;planner.allocate(aligned);auto aim=intent(20,20);planner.build(d,aim,aligned);
    require(aligned.status==FieldStatus::Valid&&aligned.coarse.status==FieldStatus::Valid&&aligned.fine.status==FieldStatus::Valid&&aligned.termination==CurveEnd::Exit&&aligned.curve_size==1&&aligned.curve[0].valid&&std::abs(aligned.curve[0].alpha-20*direction_pi/180)<1e-12,"aligned anchor fabricated a gradient or wrong coordinates");
    planner.build(d,in,r);
  }
  counting=true;planner.build(d,in,r);counting=false;
  require(allocations==0&&r.reused&&r.fine.iterations==0,"steady field/curve allocates or recomputes");
  d.sequence++;planner.build(d,in,r);require(r.reused&&r.sequence==d.sequence,"same-domain new-group cache wrong");
  in.coordinate_revision++;planner.build(d,in,r);require(!r.fine.reused,"coordinate revision reused stale field");
  {
    HarmonicConfig small;small.fine_neumann_roi=false;small.corridor_deg=4.;HarmonicPlanner local;local.configure(cameras(320,240),320,240,small,3);HarmonicResult limited;local.allocate(limited);
    auto local_intent=intent(-10,10);local.build(d,local_intent,limited);
    const auto coupled=std::count(limited.fine.boundary.begin(),limited.fine.boundary.end(),uint8_t(FieldBoundary::Coupling));
    require(limited.status==FieldStatus::Valid&&limited.termination==CurveEnd::Exit&&limited.fine.nodes<3000&&coupled>0&&!limited.expanded,"valid local coarse-fine coupling not used");
  }
  // q0 is observed in a mixed coarse cell (six available, three unknown).
  region(-45,45,1,20);auto mixed=intent(-20.5,20.5);mixed.q0=sphere_point(-20.5*direction_pi/180,1.5*direction_pi/180);mixed.desired.d_desired=sphere_point(20.5*direction_pi/180,4.5*direction_pi/180);++d.sequence;planner.invalidate_cache();planner.build(d,mixed,r);
  require(r.coarse.state[planner.coarse_stencil().cell(mixed.q0)]==uint8_t(CandidateStatus::Unknown),"mixed fixture is not mixed");
  require(r.coarse.status==FieldStatus::Valid&&r.coarse.iterations>0&&r.fine.status==FieldStatus::Valid&&r.adaptive.refined[r.adaptive.owner[s.cell(mixed.q0)]]&&r.termination==CurveEnd::Exit,"mixed anchor skipped coarse solve/coupling");
  require(std::count(r.fine.boundary.begin(),r.fine.boundary.end(),uint8_t(FieldBoundary::Coupling))>0,"mixed case has no actual coarse-fine boundary");
  const auto fv=r.field_version;const auto cv=r.curve_version;auto moved=mixed;moved.q0=sphere_point(-20.3*direction_pi/180,1.6*direction_pi/180);planner.build(d,moved,r);
  require(r.coarse.reused&&r.fine.reused&&!r.reused&&r.field_version==fv&&r.curve_version!=cv&&direction_angle(r.curve[0].direction,moved.q0)<1e-7,"anchor change rebuilt a compatible field or kept old curve");
  double length=0;for(uint32_t k=0;k<r.curve_size;++k){const auto &v=r.curve[k];if(k)length+=direction_angle(r.curve[k-1].direction,v.direction);require(std::abs(v.zeta-length)<1e-10,"curve parameter is not angular arc length");if(!v.zero_field)require(std::abs(direction_dot(v.unit_B,v.unit_B)-1)<1e-12,"unit query field not unit length");}
  require(std::abs(r.angular_length-length)<1e-10,"total angular length wrong");
  region(-30,31,0,1);in=intent(-20,20);in.q0=sphere_point(-20*direction_pi/180,.5*direction_pi/180);in.desired.d_desired=sphere_point(20*direction_pi/180,.5*direction_pi/180);d.sequence++;planner.build(d,in,r);
  std::cout<<"narrow: "<<field_status_name(r.status)<<' '<<curve_end_name(r.termination)<<" expanded="<<r.expanded<<" steps="<<r.integration_steps<<'\n';
  require(r.adaptive.refined_nodes>0&&r.coarse.status==FieldStatus::Valid&&r.fine.status==FieldStatus::Valid&&r.termination==CurveEnd::Exit,"mixed coarse block erased fine corridor");
  region(-45,45,-20,20);for(uint32_t i=0;i<d.fine_state.size();++i){const double a=-180+(i%na+.5),b=-90+(i/na+.5);if(std::abs(a)<3&&b<12&&b>=-20)d.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
  d.sequence++;in=intent(-25,25);planner.build(d,in,r);std::cout<<"wall-gap: "<<field_status_name(r.status)<<' '<<curve_end_name(r.termination)<<" steps="<<r.integration_steps<<'\n';
  require(r.fine.status==FieldStatus::Valid,"fine gap solve failed");for(uint32_t k=1;k<r.curve_size;++k)require(FineCurveIntegrator::segment_clear(s,r.fine,r.curve[k-1].direction,r.curve[k].direction),"gap reference crossed barrier");
  region(-45,45,-20,20);for(uint32_t i=0;i<d.fine_state.size();++i){const double a=-180+(i%na+.5),b=-90+(i/na+.5);if(std::abs(a)<3&&b>=-20&&b<20&&!(b>=0&&b<1))d.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
  in=intent(-25,25);d.sequence++;planner.build(d,in,r);
  require(r.adaptive.refined_nodes>0&&r.coarse.status==FieldStatus::Valid&&r.fine.status==FieldStatus::Valid,"disconnected coarse branch not refined");
  for(uint32_t i=0;i<r.fine.boundary.size();++i)if(FieldBoundary(r.fine.boundary[i])==FieldBoundary::Exit)require(-180+(i%na+.5)>15,"coarse side falsely became earlier exit");
  region(-45,-5,-20,20);in=intent(-20,20);d.sequence++;planner.build(d,in,r);require(r.exit_clipped,"disconnected requested exit not reported");
  std::fill(d.fine_state.begin(),d.fine_state.end(),0);d.sequence++;planner.build(d,in,r);require(r.status==FieldStatus::NeedsRefinement&&r.curve_size==0,"unknown start fabricated reference");
  in.desired.has_direction=false;planner.build(d,in,r);require(r.status==FieldStatus::Idle&&r.termination==CurveEnd::NoDirection,"zero intent normalized");
  // Direct integration tests for zero field, seam, poles, and artificial edge.
  r.fine.status=FieldStatus::Valid;std::fill(r.fine.boundary.begin(),r.fine.boundary.end(),uint8_t(FieldBoundary::Interior));std::fill(r.fine.phi.begin(),r.fine.phi.end(),.5);std::fill(r.fine.state.begin(),r.fine.state.end(),uint8_t(CandidateStatus::Available));
  r.intent=intent(0,20);FineCurveIntegrator{}.integrate(s,cfg,r);require(r.termination==CurveEnd::Stagnation,"zero field failed stagnation");
  r.fine.boundary[s.cell(r.intent.q0)]=uint8_t(FieldBoundary::Exit);FineCurveIntegrator{}.integrate(s,cfg,r);require(r.termination==CurveEnd::Exit,"exit should precede zero test");
  auto a=sphere_point(179.9*direction_pi/180,0),b=sphere_point(-179.9*direction_pi/180,0);require(FineCurveIntegrator::segment_clear(s,r.fine,a,b),"periodic seam blocked");r.fine.boundary[s.cell(a)]=0;require(!FineCurveIntegrator::segment_clear(s,r.fine,a,b),"unknown seam skipped");r.fine.boundary[s.cell(a)]=1;
  a=sphere_point(0,89.9*direction_pi/180);b=sphere_point(direction_pi,89.9*direction_pi/180);require(FineCurveIntegrator::segment_clear(s,r.fine,a,b),"complete polar cap blocked");r.fine.boundary[s.layout.cells()-10]=0;require(!FineCurveIntegrator::segment_clear(s,r.fine,a,b),"missing polar wedge skipped");
  std::fill(r.fine.boundary.begin(),r.fine.boundary.end(),uint8_t(FieldBoundary::Interior));
  for(uint32_t i=0;i<s.layout.cells();++i){r.fine.phi[i]=s.nodes[i].direction[0];if(direction_angle(s.nodes[i].direction,sphere_point(0,85*direction_pi/180))<direction_pi/180)r.fine.boundary[i]=uint8_t(FieldBoundary::Exit);}
  r.intent.q0={0,0,1};FineCurveIntegrator{}.integrate(s,cfg,r);require(r.termination==CurveEnd::Exit&&r.curve_size>1&&r.curve[0].local_chart,"polar tangent integration failed");
  // Shared-vertex scalar patches: continuous gradient and finite-difference
  // agreement with the reconstructed potential, not a normalized arbitrary flow.
  for(uint32_t i=0;i<s.layout.cells();++i){const auto &q=s.nodes[i].direction;r.fine.phi[i]=.5+.2*q[0]+.1*q[1]+.15*q[2];r.fine.boundary[i]=uint8_t(FieldBoundary::Interior);}
  for(auto ab:{std::pair<double,double>{.0,.2},{.1,.0},{-.2,.3}}){const double e=1e-7;const auto a=FineDirectionField::query(s,r.fine,sphere_point(ab.first-e,ab.second)),b=FineDirectionField::query(s,r.fine,sphere_point(ab.first+e,ab.second));require(std::abs(a.F_alpha-b.F_alpha)+std::abs(a.F_beta-b.F_beta)<1e-5,"gradient jumps at patch face");const auto g=FineDirectionField::query(s,r.fine,sphere_point(ab.first,ab.second));const auto u=FineDirectionField::query(s,r.fine,sphere_point(ab.first,ab.second+e)),v=FineDirectionField::query(s,r.fine,sphere_point(ab.first,ab.second-e));require(std::abs((b.potential-a.potential)/(2*e)-g.F_alpha*std::pow(std::cos(ab.second),2))<1e-6&&std::abs((u.potential-v.potential)/(2*e)-g.F_beta)<1e-6,"query is not metric gradient of queried potential");}
  // Zero-Neumann face reconstruction must vanish at the unobserved face.
  for(uint32_t i=0;i<s.layout.cells();++i){const double beta=-90+(i/360+.5);r.fine.boundary[i]=beta>=1 ? uint8_t(FieldBoundary::Interior):0;r.fine.phi[i]=-beta*direction_pi/180;}
  auto near_face=FineCurveIntegrator::gradient(s,r.fine,sphere_point(0,(1+1e-7)*direction_pi/180));require(near_face.valid&&std::abs(near_face.F_beta)<1e-6,"Neumann face gradient did not vanish");
  std::fill(r.fine.boundary.begin(),r.fine.boundary.end(),0);const auto sw=s.cell(sphere_point(-.25*direction_pi/180,-.25*direction_pi/180)),ne=s.cell(sphere_point(.25*direction_pi/180,.25*direction_pi/180));r.fine.boundary[sw]=r.fine.boundary[ne]=uint8_t(FieldBoundary::Interior);r.fine.phi[sw]=.2;r.fine.phi[ne]=.9;auto isolated=FineDirectionField::query(s,r.fine,sphere_point(-.25*direction_pi/180,-.25*direction_pi/180));require(isolated.valid&&isolated.zero_field&&std::abs(isolated.potential-.2)<1e-12,"blind diagonal sectors interpolated together");
  // Artificial ROI edges request expansion, never a coarse-gradient splice.
  std::fill(r.fine.boundary.begin(),r.fine.boundary.end(),uint8_t(FieldBoundary::Interior));std::fill(r.fine.phi.begin(),r.fine.phi.end(),.5);
  r.intent.q0={1,0,0};r.fine.boundary[s.cell(r.intent.q0)]=uint8_t(FieldBoundary::Coupling);
#if EGO4P0_CURVE_GUARD
  // A Coupling label needs an actual Available-but-outside-ROI neighbor.
  r.fine.boundary[s.nodes[s.cell(r.intent.q0)].neighbor[1]]=0;
#endif
  FineCurveIntegrator{}.integrate(s,cfg,r);require(r.termination==CurveEnd::NeedsExpansion,"coupling edge did not request refinement");
  bool expansion=false;auto qa=sphere_point(.1,0),qb=sphere_point(.11,0);r.fine.boundary[s.cell(qb)]=0;require(!FineCurveIntegrator::segment_clear(s,r.fine,qa,qb,0,&expansion)&&expansion,"artificial missing edge mislabeled physical obstacle");
  auto polar=FineCurveIntegrator::gradient(s,r.fine,{0,0,1});require(polar.valid&&polar.local_chart&&std::isfinite(polar.norm2),"polar local chart failed");
  std::cout<<"PASS: "<<checks<<" manufactured nodes; weighted/edge summary, residual/iteration rejection, metric gradient, reuse/invalidation/zero-allocation, mixed narrow corridor, gap safety, unknown/zero intent, seam/polar supercover\n";
  return 0;
}catch(const std::exception &e){counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
