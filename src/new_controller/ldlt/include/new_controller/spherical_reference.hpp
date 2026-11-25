#pragma once

#include "new_controller/spherical_field.hpp"
#include "new_controller/spatial_reference.hpp" // shared scalar curvature ramp

namespace ego4p0 {
class SphericalReference {
  using Bearing=spherical::Bearing;
  using Radial=spherical::Radial;
  using Tangent=spherical::Tangent;
  using Controls=std::array<Tangent,4>;
  struct Piece {
    double start=0.,h=0.,speed_bound=1.,second_bound=1.;
    Bearing entry{};Tangent forward{};bool great_circle=false;
    Controls controls{};
    std::array<Tangent,3> derivative{};
  };
 public:
  struct Sample {Radial position;Bearing heading;Radial curvature;double error;};
  bool build(const HarmonicResult &field,Radial anchor,double yaw) {
    pieces_.clear();nodes_.clear();chart_length_=0.;max_speed_=1.;double max_second=1.;
    if(!field.curve_size || field.termination!=CurveEnd::Exit || field.curve_size>field.curve.size())return false;
    if(field.fine.state.size()!=field.fine.layout.cells() || field.fine.boundary.size()!=field.fine.layout.cells())return false;
    std::vector<Bearing> points;points.reserve(field.curve_size);
    for(uint32_t i=0;i<field.curve_size;++i){
      Bearing p{field.curve[i].alpha,field.curve[i].beta};
      if(!field.fine.layout.cell(p.az,p.el))return false;
      if(points.empty() || spherical::angle(points.back(),p)>1e-11)points.push_back(p);
    }
    CurveEvidence first;
    if(!angular_arc_clear(field.fine.layout,field.fine,points.front(),points.front(),first))return false;
    yaw_=yaw;tail_=points.back();
    std::vector<Tangent> tangents(points.size());
    for(size_t i=0;i<points.size() && points.size()>1;++i){
      const auto p=points[i];Tangent acc{};
      if(i){auto incoming=unit(spherical::log(p,points[i-1]));acc={-incoming.east,-incoming.north};}
      if(i+1<points.size()){const auto outgoing=unit(spherical::log(p,points[i+1]));acc.east+=outgoing.east;acc.north+=outgoing.north;}
      if(std::hypot(acc.east,acc.north)<1e-7)return false;
      tangents[i]=unit(acc);
    }
    for(size_t i=1;i<points.size();++i){
      Piece p;p.start=chart_length_;p.entry=points[i-1];
      const auto end=points[i];const auto chord=spherical::log(p.entry,end);
      p.h=std::hypot(chord.east,chord.north);if(!(p.h>0.) || p.h>direction_pi-1e-6)return false;
      p.forward=unit(chord);
      const auto end_tangent=spherical::transport(p.entry,end,p.forward);
      auto mismatch=[](Tangent a,Tangent b){return std::hypot(a.east-b.east,a.north-b.north);};
      p.great_circle=mismatch(tangents[i-1],p.forward)<2e-12 && mismatch(tangents[i],end_tangent)<2e-12;
      if(!p.great_circle){
        // Cubic Hermite in the normal chart x=Log_entry(q). Invert the
        // endpoint differential, so adjoining charts agree on the same
        // physical unit tangent when differentiated by knot angle z.
        const auto back=spherical::transport(end,p.entry,tangents[i]);
        const auto endpoint_rate=scale_transverse(back,p.forward,p.h/std::sin(p.h));
        p.controls={Tangent{},multiply(tangents[i-1],p.h/3.),
          plus(chord,endpoint_rate,-p.h/3.),chord};
        for(auto x:p.controls)if(norm(x)>direction_pi/2.)return false;
        double d1=0.,d2=0.;
        for(int j=0;j<3;++j){
          p.derivative[j]=multiply(plus(p.controls[j+1],p.controls[j],-1.),3./p.h);
          const auto &v=p.derivative[j];
          // All derivative control points must progress along the chord.
          // The minimum singular value of dExp in this chart is >= 2/pi.
          if(v.east*p.forward.east+v.north*p.forward.north<=direction_pi*.5e-4)return false;
          d1=std::max(d1,norm(v));
        }
        for(int j=0;j<2;++j)d2=std::max(d2,2.*norm(plus(p.derivative[j+1],p.derivative[j],-1.))/p.h);
        // Singular values of dExp are 1 and sinc(r). On r<=pi/2 its
        // ambient Hessian is bounded by 2+pi/6 < 3: use sinc(r) =
        // integral_0^1 cos(t*r)dt. The gradient/Hessian of cos(t*r) have
        // norms <=t and t², so its product with x contributes <=2t+r*t².
        // The axial cos(r) term contributes at most one more.
        p.speed_bound=d1;
        p.second_bound=d2+3.*d1*d1;
        if(!certify_piece(field.fine,p.entry,p.controls,0))return false;
      }else{
        CurveEvidence proof;
        if(!angular_arc_clear(field.fine.layout,field.fine,p.entry,end,proof))return false;
      }
      chart_length_+=p.h;max_speed_=std::max(max_speed_,p.speed_bound);max_second=std::max(max_second,p.second_bound);
      pieces_.push_back(p);
    }
    // sigma = M * cumulative knot angle, with |dQ/dsigma| <= 1.
    // One triangular/trapezoidal ramp spans the entire interpolant.
    profile_=CurvatureSchedule::make(max_speed_*chart_length_);
    if(!std::isfinite(profile_.length) || profile_.length>500.)return false;
    const double k=NavigationConfig::reference_max_curvature_per_m;
    second_bound_=k/SpatialReferenceConfig::ramp_length_m+k*k*max_second/(max_speed_*max_speed_);
    if(!std::isfinite(second_bound_))return false;
    nodes_.push_back({0.,anchor,0.});
    while(nodes_.back().s<profile_.length){
      const double next=std::min(profile_.length,nodes_.back().s+SpatialReferenceConfig::lookup_step_m);
      nodes_.push_back(integral(nodes_.back(),next));
    }
    return true;
  }
  double length()const{return profile_.length;}
  Bearing heading(double s)const{return oriented(s).first;}
  Sample sample(double s)const{
    const auto p=position_at(s);const auto at=oriented(s);return {p.position,at.first,at.second,p.error};
  }
  double project(Radial point,double lo,double hi)const {
    double best=lo,min_distance=std::numeric_limits<double>::infinity();
    auto left=position_at(lo).position;
    for(double a=lo;a<hi;){
      const double b=std::min(hi,a+SpatialReferenceConfig::lookup_step_m);
      const auto right=position_at(b).position,delta=spherical::add(right,left,-1.);
      if(delta.r>0.){
        const auto offset=spherical::add(point,left,-1.);
        const double u=std::clamp(offset.r*spherical::cosine(offset.dir,delta.dir)/delta.r,0.,1.);
        const double cost=spherical::distance(point,spherical::add(left,delta,u));
        if(cost<min_distance){min_distance=cost;best=a+u*(b-a);}
      }
      a=b;left=right;
    }
    return best;
  }
 private:
  static double norm(Tangent t){return std::hypot(t.east,t.north);}
  static Tangent multiply(Tangent v,double k){return {v.east*k,v.north*k};}
  static Tangent unit(Tangent t){return multiply(t,1./norm(t));}
  static Tangent plus(Tangent a,Tangent b,double k=1.){return {a.east+k*b.east,a.north+k*b.north};}
  static Tangent blend(Tangent a,Tangent b,double u){return plus(a,plus(b,a,-1.),u);}
  static Tangent scale_transverse(Tangent v,Tangent along,double factor){
    const Tangent ortho{-along.north,along.east};
    return plus(v,ortho,(factor-1.)*(v.east*ortho.east+v.north*ortho.north));
  }
  static bool certify_piece(const FieldLayer &f,Bearing base,const Controls &c,unsigned depth){
    const auto e=blend(c[0],c[1],.5),v=blend(c[1],c[2],.5),w=blend(c[2],c[3],.5);
    const auto j=blend(e,v,.5),k=blend(v,w,.5),m=blend(j,k,.5);
    // The planar polynomial lies in its control hull; Exp on the normal
    // chart is 1-Lipschitz. This cap therefore encloses every curve direction.
    double radius=0.;for(auto x:c)radius=std::max(radius,norm(plus(x,m,-1.)));
    CurveEvidence box;
    if(radius<direction_pi/2.-1e-10 && angular_cap_clear(f.layout,f,spherical::exp(base,m),radius+2e-13,box))return true;
    if(depth>=14 || radius<1e-12)return false;
    return certify_piece(f,base,{c[0],e,j,m},depth+1) && certify_piece(f,base,{m,k,w,c[3]},depth+1);
  }
  Bearing world(Bearing dir)const{return {spherical::wrap(dir.az+yaw_),dir.el};}
  std::pair<Bearing,Radial> oriented(double s)const{
    if(pieces_.empty() || s>=profile_.length)return {world(tail_),{}};
    const auto sched=profile_.query(s);
    const double z=sched[0]/max_speed_,rate=sched[1]/max_speed_;
    auto after=std::upper_bound(pieces_.begin(),pieces_.end(),z,[](double at,const Piece &p){return at<p.start;});
    const auto &p=*(after==pieces_.begin()?after:std::prev(after));
    const double u=std::clamp((z-p.start)/p.h,0.,1.);
    if(p.great_circle){
      return {world(spherical::advance(p.entry,p.forward,u*p.h)),
        {world(spherical::advance(p.entry,p.forward,u*p.h+direction_pi/2)),rate}};
    }
    const auto &c=p.controls;
    const auto a=blend(c[0],c[1],u),b=blend(c[1],c[2],u),d=blend(c[2],c[3],u);
    const auto x=blend(blend(a,b,u),blend(b,d,u),u);
    const auto dx=blend(blend(p.derivative[0],p.derivative[1],u),blend(p.derivative[1],p.derivative[2],u),u);
    const double r=norm(x);
    const auto q=spherical::exp(p.entry,x);
    // The derivative in the radial chart direction is unchanged. In the
    // angular direction it is contracted, then parallel transported to q.
    const auto local=r>1e-12?scale_transverse(dx,multiply(x,1./r),std::sin(r)/r):dx;
    const auto t=spherical::transport(p.entry,q,local);
    const double magnitude=norm(t);
    const auto normal=magnitude>0.?spherical::advance(q,multiply(t,1./magnitude),direction_pi/2):q;
    return {world(q),{world(normal),magnitude*rate}};
  }
  struct Node {double s;Radial position;double error;};
  Node integral(const Node &left,double s)const{
    const double h=s-left.s;const auto position=spherical::add(left.position,{heading(left.s+h*.5),h});
    // T' is continuous, and T'' is bounded piecewise, including across knots.
    return {s,position,left.error+second_bound_*h*h*h/24.+roundoff(position.r)};
  }
  static double roundoff(double scale){return 128*std::numeric_limits<double>::epsilon()*(1.+scale);}
  Node position_at(double s)const{
    if(s>=profile_.length){
      const auto &last=nodes_.back();const auto p=spherical::add(last.position,{world(tail_),s-last.s});
      return {s,p,last.error+roundoff(p.r)};
    }
    const auto right=std::lower_bound(nodes_.begin(),nodes_.end(),s,[](const Node &n,double target){return n.s<target;});
    if(right->s==s)return *right;
    return integral(*std::prev(right),s);
  }
  Bearing tail_{};double yaw_=0.,chart_length_=0.,max_speed_=1.,second_bound_=0.;
  CurvatureSchedule profile_;
  std::vector<Piece> pieces_;
  std::vector<Node> nodes_;
};
} // namespace ego4p0
