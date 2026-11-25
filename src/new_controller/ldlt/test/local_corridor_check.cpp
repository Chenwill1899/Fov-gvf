#include "new_controller/harmonic_planner.hpp"
#include <iostream>
using namespace ego4p0;
static std::array<CameraCalibration,4> cameras(uint32_t w,uint32_t h){
  std::array<CameraCalibration,4> cal;
  for(uint32_t c=0;c<4;++c){const double a=c*direction_pi/2,co=std::cos(a),si=std::sin(a);cal[c].K={.6*w,0,w*.5,0,.65*w,h*.5,0,0,1};cal[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};}
  return cal;
}
static void need(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
int main(){try{
  HarmonicConfig cfg;need(cfg.fine_neumann_roi&&cfg.corridor_deg==3.&&cfg.max_step_deg==.8,"new profile defaults differ");
  HarmonicPlanner p;p.configure(cameras(320,240),320,240,cfg,4);p.set_trial_solvers(SparseTrialMode::EigenLDLT,true);
  HarmonicDepthInput d;p.allocate(d,320,240);d.ready=true;d.sequence=d.configuration=d.geometry=1;
  for(auto &camera:d.coarse_image)for(auto &v:camera){v.total=v.valid=v.raw_valid=144;v.mean_z=v.min_rho=v.raw_min_rho=10.;}
  HarmonicResult r;p.allocate(r);const auto &s=p.fine_stencil();unsigned checked=0,early=0;
  auto run=[&](double q0,double dest,bool aligned=false){
    HarmonicIntent in;in.q0=sphere_point(q0*direction_pi/180,.5*direction_pi/180);
    in.desired={sphere_point(dest*direction_pi/180,.5*direction_pi/180),12,true,true};in.R_WB={1,0,0,0,1,0,0,0,1};
    ++d.sequence;p.invalidate_cache();p.build(d,in,r);
    if(r.status!=FieldStatus::Valid||r.termination!=CurveEnd::Exit){++early;std::cout<<"REJECT case="<<checked<<" term="<<curve_end_name(r.termination)<<" nodes="<<r.fine.nodes<<'\n';return;}
    need(r.coarse.status==FieldStatus::TopologyReady&&r.coarse_topology_only&&std::isnan(r.coarse.residual)&&r.coarse.iterations==0&&r.coarse.solve_ns==0&&r.coarse.solve_cpu_ns==0&&r.fine.status==FieldStatus::Valid,"unverified topology/fine field accepted");
    need(std::count(r.fine.boundary.begin(),r.fine.boundary.end(),uint8_t(FieldBoundary::Coupling))==0,"local mode leaked coarse coupling");
    double maximum=0.;
    for(uint32_t i=0;i<s.nodes.size();++i)if(r.fine.boundary[i]){
      need(d.fine_state[i]==uint8_t(CandidateStatus::Available),"unknown node entered fine system");
      need(std::isfinite(r.fine.phi[i])&&r.fine.phi[i]>=-1e-7&&r.fine.phi[i]<=1+1e-7,"potential range invalid");
      if(FieldBoundary(r.fine.boundary[i])!=FieldBoundary::Interior)continue;
      double residual=0.,scale=0.;
      for(unsigned k=0;k<4;++k){const auto j=s.nodes[i].neighbor[k];if(j==UINT32_MAX||!r.fine.boundary[j])continue;
        const auto w=s.nodes[i].weight[k];residual+=w*(r.fine.phi[i]-r.fine.phi[j]);scale+=w*(std::abs(r.fine.phi[i])+std::abs(r.fine.phi[j]));}
      maximum=std::max(maximum,std::abs(residual)/std::max(1.,scale));
    }
    need(maximum<=cfg.residual_tolerance,"independent fine residual failed");
    FieldLayer evidence=r.fine;for(uint32_t i=0;i<s.nodes.size();++i)evidence.boundary[i]=d.fine_state[i]==uint8_t(CandidateStatus::Available)?uint8_t(FieldBoundary::Interior):0;
    for(uint32_t k=1;k<r.curve_size;++k){need(FineCurveIntegrator::segment_clear(s,evidence,r.curve[k-1].direction,r.curve[k].direction),"curve crosses original unknown/obstacle mask");need(r.curve[k].zeta>=r.curve[k-1].zeta,"arc length decreased");}
    if(aligned)need(r.curve_size==1,"aligned intent fabricated extra motion");
    p.build(d,in,r);need(r.reused&&r.fine.reused,"local exact cache missed");
    d.fine_state[s.cell(in.q0)]=uint8_t(CandidateStatus::Unknown);++d.sequence;p.build(d,in,r);
    need(r.status==FieldStatus::NeedsRefinement&&r.curve_size==0,"unknown start reused stale curve");
    ++checked;
  };
  auto region=[&](double bottom,double top){for(uint32_t i=0;i<s.nodes.size();++i){const auto a=-180+(i%360+.5),b=-90+(i/360+.5);const bool ok=a>=-45&&a<45&&b>=bottom&&b<top;d.fine_state[i]=ok?uint8_t(CandidateStatus::Available):uint8_t(CandidateStatus::Unknown);d.cameras[i]=ok?1:0;}};
  region(-20,20);run(-20,20);
  region(-20,20);run(20,20,true);
  region(0,1);run(-20,20);
  for(int gap=-12;gap<=12;gap+=3){
    region(-20,20);
    for(uint32_t i=0;i<s.nodes.size();++i){double a=-180+(i%360+.5),b=-90+(i/360+.5);if(std::abs(a)<3&&!(b>=gap&&b<gap+3))d.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
    run(-25,25);
  }
  for(int shift=-6;shift<=6;shift+=3){
    region(-20,20);
    for(uint32_t i=0;i<s.nodes.size();++i){double a=-180+(i%360+.5),b=-90+(i/360+.5);if((std::abs(a+12)<2&&!(b>=shift+5&&b<shift+9))||(std::abs(a-8)<2&&!(b>=shift-9&&b<shift-5)))d.fine_state[i]=uint8_t(CandidateStatus::Unavailable);}
    run(-30,30);
  }
  need(checked==17&&early==0,"some connected fixtures rejected");
  std::cout<<"PASS local corridor "<<checked<<" cases: aligned, narrow, shifted gaps, two barriers, original-mask supercover, independent FV residual, exact reuse and unknown invalidation\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
