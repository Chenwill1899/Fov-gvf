#pragma once
#include "new_controller/harmonic_direction_field.hpp"
#include "new_controller/harmonic_curve_evidence.hpp"
namespace ego4p0 {
// Stage 10: a single fixed angular anchor; normalized fine-field integration.
class FineCurveIntegrator {
 public:
  static CurveSample gradient(const SphericalStencil &s,const FieldLayer &f,const Direction3 &q) {
    return FineDirectionField::query(s,f,q);
  }
  static Direction3 advance(const Direction3 &q,const Direction3 &gradient,double tau) {
    const double n=std::sqrt(direction_dot(gradient,gradient));if(n<1e-30)return q;
    const double angle=tau*n;Direction3 next{};
    for(uint32_t k=0;k<3;++k)next[k]=std::cos(angle)*q[k]+std::sin(angle)*gradient[k]/n;
    unit_direction(next);return next;
  }
  // Conservative supercover of short great-circle arcs, including orthogonal
  // bridge cells for diagonal transitions. Recursive alpha subdivision handles
  // the seam; pole crossings require the entire corresponding polar row.
  static bool segment_clear(const SphericalStencil &s,const FieldLayer &f,const Direction3 &a,const Direction3 &b,uint32_t depth=0,bool *needs_expansion=nullptr) {
    auto covered=[&](uint32_t i){if(f.boundary[i])return true;if(needs_expansion&&f.state[i]==uint8_t(CandidateStatus::Available))*needs_expansion=true;return false;};
    const auto ia=s.cell(a),ib=s.cell(b);if(!covered(ia)||!covered(ib))return false;
    Direction3 mid{};for(uint32_t k=0;k<3;++k)mid[k]=a[k]+b[k];if(!unit_direction(mid))return false;
    const auto im=s.cell(mid);if(!covered(im))return false;
    const double aa=std::atan2(a[1],a[0]),ab=std::atan2(b[1],b[0]),ba=std::atan2(a[2],std::hypot(a[0],a[1])),bb=std::atan2(b[2],std::hypot(b[0],b[1]));
    const auto na=s.layout.azimuth_bins;
    if(std::hypot(mid[0],mid[1])<1e-8) {
      const uint32_t row=mid[2]>0 ? s.layout.elevation_bins-1 : 0;
      for(uint32_t j=row*na;j<(row+1)*na;++j)if(!covered(j))return false;
      return true;
    }
    if(std::abs(wrapped_angle(ab-aa))>.25*s.layout.azimuth_step || std::abs(bb-ba)>.25*s.layout.elevation_step || (ia!=im&&ib!=im)) {
      if(depth>=20)return false;
      return segment_clear(s,f,a,mid,depth+1,needs_expansion)&&segment_clear(s,f,mid,b,depth+1,needs_expansion);
    }
    const uint32_t ra=ia/na,rb=ib/na,ca=ia%na,cb=ib%na;
    if(ra!=rb&&ca!=cb && (!covered(ra*na+cb)||!covered(rb*na+ca)))return false;
    return true;
  }
  static bool expansion_allowed(const HarmonicResult &result) {
    return result.termination==CurveEnd::NeedsExpansion&&result.curve_failure_kind==CurveArtificial;
  }
  void integrate(const SphericalStencil &s,const HarmonicConfig &config,HarmonicResult &result)const {
    result.curve_failure_kind=0;result.curve_failure_cell=UINT32_MAX;result.curve_rejected_trials=0;result.curve_min_accepted_step_rad=0.;
#if EGO4P0_CURVE_GUARD
    integrate_revised(s,config,result);
#else
    integrate_legacy(s,config,result);
#endif
  }
  void integrate_legacy(const SphericalStencil &s,const HarmonicConfig &config,HarmonicResult &result)const {
    const int64_t begin=field_now_ns();result.curve_size=0;result.integration_steps=0;result.gradient_queries=0;result.termination=CurveEnd::NoField;
    result.angular_length=result.anchor_potential=0;result.exit_direction={};result.query_ns=result.segment_ns=0;result.segment_checks=0;
    if(result.fine.status!=FieldStatus::Valid) {result.curve_ns=field_now_ns()-begin;return;}
    Direction3 q=result.intent.q0;if(!unit_direction(q)) {result.termination=CurveEnd::NoDirection;result.curve_ns=field_now_ns()-begin;return;}
    const double maximum=config.max_step_deg*direction_pi/180,minimum=config.min_step_deg*direction_pi/180;
    double length=0.;result.angular_length=0;result.query_ns=result.segment_ns=0;result.segment_checks=0;
    auto query=[&](const Direction3 &p){const auto t=field_now_ns();auto v=FineDirectionField::query(s,result.fine,p,config.gradient_epsilon);result.query_ns+=field_now_ns()-t;++result.gradient_queries;return v;};
    for(uint32_t step=0;step<=config.max_curve_steps;++step) {
      auto g=query(q);g.zeta=g.tau=length;result.curve[result.curve_size++]=g;
      if(!step)result.anchor_potential=g.potential;
      if(!g.valid) {result.termination=CurveEnd::NeedsExpansion;break;}
      if(FieldBoundary(result.fine.boundary[g.cell])==FieldBoundary::Exit) {result.termination=CurveEnd::Exit;result.exit_direction=q;break;}
      if(FieldBoundary(result.fine.boundary[g.cell])==FieldBoundary::Coupling){result.termination=CurveEnd::NeedsExpansion;break;}
      if(g.norm2<=config.gradient_epsilon*config.gradient_epsilon) {result.termination=CurveEnd::Stagnation;break;}
      if(step==config.max_curve_steps) {result.termination=CurveEnd::StepLimit;break;}
      double theta=maximum;bool accepted=false,artificial=false;Direction3 next{};
      while(theta>=minimum) {
        const auto mid=advance(q,g.unit_B,theta*.5);auto gm=query(mid);
        if(!gm.valid) {artificial|=result.fine.state[s.cell(mid)]==uint8_t(CandidateStatus::Available);theta*=.5;continue;}
        auto v=gm.unit_B;const double radial=direction_dot(v,q);for(uint32_t k=0;k<3;++k)v[k]-=radial*q[k];
        if(!unit_direction(v)){theta*=.5;continue;}next=advance(q,v,theta);
        const auto check_begin=field_now_ns();result.segment_checks+=3;
        const bool clear=segment_clear(s,result.fine,q,mid,0,&artificial)&&segment_clear(s,result.fine,mid,next,0,&artificial)&&segment_clear(s,result.fine,q,next,0,&artificial);
        result.segment_ns+=field_now_ns()-check_begin;
        if(clear) {const auto gn=query(next);if(gn.valid&&gn.potential>g.potential-1e-12){accepted=true;break;}}
        artificial|=result.fine.state[s.cell(next)]==uint8_t(CandidateStatus::Available)&&!result.fine.boundary[s.cell(next)];theta*=.5;
      }
      if(!accepted) {result.termination=artificial ? CurveEnd::NeedsExpansion : CurveEnd::Blocked;break;}
      length+=direction_angle(q,next);q=next;result.angular_length=length;++result.integration_steps;
    }
    result.curve_ns=field_now_ns()-begin;
  }
  void integrate_revised(const SphericalStencil &s,const HarmonicConfig &config,HarmonicResult &result)const {
    const int64_t begin=field_now_ns();result.curve_size=result.integration_steps=result.gradient_queries=0;result.termination=CurveEnd::NoField;
    result.angular_length=result.anchor_potential=0;result.exit_direction={};result.query_ns=result.segment_ns=0;result.segment_checks=0;
    result.curve_failure_kind=0;result.curve_failure_cell=UINT32_MAX;result.curve_rejected_trials=0;result.curve_min_accepted_step_rad=0.;
    if(result.fine.status!=FieldStatus::Valid){result.curve_ns=field_now_ns()-begin;return;}
    Direction3 q=result.intent.q0;if(!unit_direction(q)){result.termination=CurveEnd::NoDirection;result.curve_ns=field_now_ns()-begin;return;}
    const double maximum=config.max_step_deg*direction_pi/180;
    // Experimental geometry substep floor, never a reduction in coverage or
    // solver tolerance. Both modes use the same midpoint and fine gradient.
    const double minimum=(EGO4P0_CORNER_SMALL_STEP ? std::min(config.min_step_deg,1e-10) : config.min_step_deg)*direction_pi/180;
    const double progress_floor=32*std::numeric_limits<double>::epsilon();
    double length=0.;
    auto query=[&](const Direction3 &p){const auto t=field_now_ns();auto v=FineDirectionField::query(s,result.fine,p,config.gradient_epsilon);result.query_ns+=field_now_ns()-t;++result.gradient_queries;return v;};
    auto stop=[&](const CurveEvidence &e,CurveEnd otherwise=CurveEnd::Blocked){result.curve_failure_kind=e.kind;result.curve_failure_cell=e.cell;result.termination=e.pure_artificial() ? CurveEnd::NeedsExpansion : otherwise;};
    for(uint32_t step=0;step<=config.max_curve_steps;++step) {
      auto g=query(q);
      if(!g.valid){stop(FineCurveEvidence::failed_query(s,result.fine,q));break;}
      CurveEvidence current;if(!FineCurveEvidence::covered(result.fine,g.cell,current)){stop(current);break;}
      g.zeta=g.tau=length;result.curve[result.curve_size++]=g;if(!step)result.anchor_potential=g.potential;
      if(FieldBoundary(result.fine.boundary[g.cell])==FieldBoundary::Exit){result.termination=CurveEnd::Exit;result.exit_direction=q;break;}
      if(FieldBoundary(result.fine.boundary[g.cell])==FieldBoundary::Coupling){stop(FineCurveEvidence::coupling(s,result.fine,g.cell));break;}
      if(g.norm2<=config.gradient_epsilon*config.gradient_epsilon){result.termination=CurveEnd::Stagnation;result.curve_failure_kind=CurveNoProgress;break;}
      if(step==config.max_curve_steps){result.termination=CurveEnd::StepLimit;break;}
      bool accepted=false;Direction3 next{};CurveEvidence failure;double accepted_length=0.;
      double theta=maximum;
      for(uint32_t trial=0;trial<64&&theta>=minimum;++trial,theta*=.5) {
        // Evidence belongs only to this attempted substep. A rejected large
        // artificial crossing cannot justify expansion after a later real hit.
        failure={};
        const auto mid=advance(q,g.unit_B,theta*.5);const auto gm=query(mid);
        if(!gm.valid){failure=FineCurveEvidence::failed_query(s,result.fine,mid);++result.curve_rejected_trials;continue;}
        if(!FineCurveEvidence::covered(result.fine,gm.cell,failure)){++result.curve_rejected_trials;continue;}
        auto v=gm.unit_B;const double radial=direction_dot(v,q);for(uint32_t k=0;k<3;++k)v[k]-=radial*q[k];
        if(!unit_direction(v)){failure.add(CurveNoProgress,gm.cell);++result.curve_rejected_trials;continue;}next=advance(q,v,theta);
        const auto check_begin=field_now_ns();result.segment_checks+=3;
        const bool clear=FineCurveEvidence::segment(s,result.fine,q,mid,failure)&&FineCurveEvidence::segment(s,result.fine,mid,next,failure)&&FineCurveEvidence::segment(s,result.fine,q,next,failure);
        result.segment_ns+=field_now_ns()-check_begin;
        if(!clear){++result.curve_rejected_trials;continue;}
        const auto gn=query(next);
        if(!gn.valid){failure=FineCurveEvidence::failed_query(s,result.fine,next);++result.curve_rejected_trials;continue;}
        if(!FineCurveEvidence::covered(result.fine,gn.cell,failure)){++result.curve_rejected_trials;continue;}
        accepted_length=FineCurveEvidence::arc_length(q,next);
        if(!(accepted_length>progress_floor)||!std::isfinite(accepted_length)){failure.add(CurveNoProgress,g.cell);++result.curve_rejected_trials;continue;}
        if(!(gn.potential>g.potential-1e-12)){failure.add(CurveNoProgress,gn.cell);++result.curve_rejected_trials;continue;}
        accepted=true;break;
      }
      if(!accepted){stop(failure,(failure.kind&(CurveUnknown|CurveObstacle|CurveNumerical)) ? CurveEnd::Blocked : CurveEnd::Stagnation);break;}
      result.curve_min_accepted_step_rad=result.curve_min_accepted_step_rad>0 ? std::min(result.curve_min_accepted_step_rad,accepted_length) : accepted_length;
      length+=accepted_length;q=next;result.angular_length=length;++result.integration_steps;
    }
    result.curve_ns=field_now_ns()-begin;
  }

};
} // namespace ego4p0
