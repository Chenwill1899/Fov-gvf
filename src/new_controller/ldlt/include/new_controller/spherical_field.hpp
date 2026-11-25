#pragma once
#include "new_controller/harmonic_curve_evidence.hpp"
#include "new_controller/harmonic_direction_field.hpp"

namespace ego4p0 {
// A rotationally symmetric polar chart. Fit a + b u + c v + d rho² on
// complete latitude rings, where (u,v) is log_pole(q) in a fixed orientation.
inline bool polar_field(const SphericalStencil &s,const FieldLayer &f,spherical::Bearing q,
                        double &value,spherical::Tangent &gradient) {
  const double sign=q.el<0.?-1.:1.;
  double count=0.,r2=0.,r4=0.,sum=0.,sum2=0.,bu=0.,cv=0.;
  for(uint32_t i=0;i<s.nodes.size();++i){
    const auto p=s.nodes[i].bearing;
    if(sign*p.el<84.*direction_pi/180.)continue;
    if(!f.boundary[i] || f.state[i]!=uint8_t(CandidateStatus::Available))return false;
    const double rho=direction_pi/2-sign*p.el, square=rho*rho,phi=f.phi[i];
    count+=1.;r2+=square;r4+=square*square;sum+=phi;sum2+=phi*square;
    bu+=rho*std::cos(p.az)*phi;cv+=rho*std::sin(p.az)*phi;
  }
  const double det=count*r4-r2*r2;
  if(!(det>0. && r2>0.))return false;
  const double a=(sum*r4-r2*sum2)/det,d=(count*sum2-r2*sum)/det,b=2*bu/r2,c=2*cv/r2;
  const double rho=direction_pi/2-sign*q.el, meridian=b*std::cos(q.az)+c*std::sin(q.az);
  value=a+rho*(meridian+d*rho);
  gradient={ (rho>1e-8?rho/std::sin(rho):1.)*(-b*std::sin(q.az)+c*std::cos(q.az)),
             -sign*(meridian+2*d*rho)};
  return std::isfinite(value);
}

inline CurveSample bearing_gradient(const SphericalStencil &s,const FieldLayer &f,spherical::Bearing q,
                                    double epsilon,spherical::Tangent &unit) {
  CurveSample out;out.alpha=q.az;out.beta=q.el;
  const auto cell=s.layout.cell(q.az,q.el);
  if(!cell || !f.boundary[*cell] || f.state[*cell]!=uint8_t(CandidateStatus::Available) ||
     f.status!=FieldStatus::Valid)return out;
  out.cell=*cell;
  double pa=0.,pb=0.;
  FineDirectionField::bicubic(s,f,out,pa,pb);
  const double ce=std::cos(q.el);
  spherical::Tangent g{ce>1e-9?pa/ce:0.,pb};
  if(std::abs(q.el)>84.*direction_pi/180.){
    double v; spherical::Tangent polar;
    if(!polar_field(s,f,q,v,polar))return out;
    const double t=std::clamp((std::abs(q.el)*180./direction_pi-84.)/2.,0.,1.);
    const double weight=t*t*(3.-2.*t),derivative=std::copysign(3*t*(1-t)*180./direction_pi,q.el);
    g={(1-weight)*g.east+weight*polar.east,
       (1-weight)*g.north+weight*polar.north+derivative*(v-out.potential)};
    out.potential+=(v-out.potential)*weight;out.local_chart=true;
  }
  out.magnitude=std::hypot(g.east,g.north);out.norm2=out.magnitude*out.magnitude;
  out.valid=std::isfinite(out.potential)&&std::isfinite(out.magnitude);
  out.zero_field=out.magnitude<=epsilon;
  if(!out.zero_field)unit={g.east/out.magnitude,g.north/out.magnitude};
  return out;
}

inline bool angular_cap_clear(const DirectionGridLayout &g,const FieldLayer &f,spherical::Bearing mid,
                              double radius,CurveEvidence &box) {
  const int na=int(g.azimuth_bins),nb=int(g.elevation_bins);
  auto row=[&](double el){return std::clamp(int(std::floor((el+direction_pi/2)/g.elevation_step)),0,nb-1);};
  int left=0,right=na-1;
  if(std::abs(mid.el)+radius<direction_pi/2){
    const double span=std::asin(std::clamp(std::sin(radius)/std::cos(mid.el),0.,1.))+2e-13;
    left=int(std::floor((mid.az-span+direction_pi)/g.azimuth_step));
    right=int(std::floor((mid.az+span+direction_pi)/g.azimuth_step));
  }
  for(int y=row(mid.el-radius);y<=row(mid.el+radius)&&!box.kind;++y)
    for(int x=left;x<=right;++x)if(!FineCurveEvidence::covered(f,y*na+(x%na+na)%na,box))break;
  return !box.kind;
}

// Every point of the minor arc is within half its length of its midpoint.
// Cover that spherical cap with latitude rows and periodic longitude columns.
// Subdivide uncertain enclosures; never authorize an unresolved intersection.
inline bool angular_arc_clear(const DirectionGridLayout &g,const FieldLayer &f,
                              spherical::Bearing a,spherical::Bearing b,CurveEvidence &failure,unsigned depth=0) {
  const double length=spherical::angle(a,b);
  if(!std::isfinite(length)||length>=direction_pi-1e-10){failure.add(CurveNumerical);return false;}
  for(auto p:{a,b}){
    const auto cell=g.cell(p.az,p.el);
    if(!cell || !FineCurveEvidence::covered(f,*cell,failure))return false;
  }
  const auto log_ab=spherical::log(a,b);
  const auto mid=spherical::exp(a,{log_ab.east*.5,log_ab.north*.5});
  const double radius=length*.5+2e-13;
  CurveEvidence box;
  angular_cap_clear(g,f,mid,radius,box);
  if(!box.kind)return true;
  if(depth>=12 || length<1e-10){failure=box;return false;}
  return angular_arc_clear(g,f,a,mid,failure,depth+1) && angular_arc_clear(g,f,mid,b,failure,depth+1);
}

inline void integrate_bearings(const SphericalStencil &s,const HarmonicConfig &cfg,HarmonicResult &r) {
  const auto start=field_now_ns();
  r.curve_size=r.integration_steps=r.gradient_queries=r.segment_checks=0;
  r.angular_length=0.;r.curve_ns=r.query_ns=r.segment_ns=0;r.termination=CurveEnd::NoField;
  auto q=r.intent.bearings->start;
  auto query=[&](spherical::Bearing at,spherical::Tangent &t){
    const auto begin=field_now_ns();auto sample=bearing_gradient(s,r.fine,at,cfg.gradient_epsilon,t);
    ++r.gradient_queries;r.query_ns+=field_now_ns()-begin;return sample;
  };
  for(uint32_t i=0;i<=cfg.max_curve_steps;++i){
    spherical::Tangent v{};auto sample=query(q,v);
    if(!sample.valid){r.termination=CurveEnd::Blocked;break;}
    sample.zeta=sample.tau=r.angular_length;r.curve[r.curve_size++]=sample;
    if(!i)r.anchor_potential=sample.potential;
    if(FieldBoundary(r.fine.boundary[sample.cell])==FieldBoundary::Exit){r.termination=CurveEnd::Exit;break;}
    if(sample.zero_field){r.termination=CurveEnd::Stagnation;break;}
    if(i==cfg.max_curve_steps){r.termination=CurveEnd::StepLimit;break;}
    bool accepted=false;CurveEvidence failure;
    for(double step=cfg.max_step_deg*direction_pi/180.;step>=1e-10*direction_pi/180.;step*=.5){
      ++r.curve_rejected_trials;failure={};
      const auto mid=spherical::exp(q,{step*v.east*.5,step*v.north*.5});
      spherical::Tangent vm{};auto gm=query(mid,vm);
      if(!gm.valid || gm.zero_field){
        const auto cell=s.layout.cell(mid.az,mid.el);
        if(!cell || FineCurveEvidence::covered(r.fine,*cell,failure))failure.add(CurveNoProgress);
        continue;
      }
      const auto back=spherical::transport(mid,q,vm);
      const auto next=spherical::exp(q,{step*back.east,step*back.north});
      const auto begin=field_now_ns();r.segment_checks+=3;
      const bool clear=angular_arc_clear(s.layout,r.fine,q,mid,failure) &&
                       angular_arc_clear(s.layout,r.fine,mid,next,failure) &&
                       angular_arc_clear(s.layout,r.fine,q,next,failure);
      r.segment_ns+=field_now_ns()-begin;
      if(!clear)continue;
      spherical::Tangent unused{};const auto gn=query(next,unused);
      if(!gn.valid || gn.potential<sample.potential-1e-12){failure.add(CurveNoProgress);continue;}
      const double length=spherical::angle(q,next);
      if(!(length>32*std::numeric_limits<double>::epsilon())){failure.add(CurveNoProgress);continue;}
      r.angular_length+=length;r.curve_min_accepted_step_rad=r.curve_min_accepted_step_rad?
        std::min(r.curve_min_accepted_step_rad,length):length;
      q=next;++r.integration_steps;--r.curve_rejected_trials;accepted=true;break;
    }
    if(!accepted){r.curve_failure_kind=failure.kind;r.curve_failure_cell=failure.cell;
      r.termination=failure.pure_artificial()?CurveEnd::NeedsExpansion:CurveEnd::Blocked;break;}
  }
  r.curve_ns=field_now_ns()-start;
}
} // namespace ego4p0
