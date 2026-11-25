#pragma once
#include "new_controller/harmonic_types.hpp"
#include "new_controller/navigation_config.hpp"
#include <algorithm>
#include <functional>

namespace ego4p0 {
inline Direction3 ref_add(Direction3 a,const Direction3 &b,double k=1.){for(int i=0;i<3;++i)a[i]+=k*b[i];return a;}
inline Direction3 ref_scale(Direction3 a,double k){for(double &v:a)v*=k;return a;}
inline double ref_norm(const Direction3 &a){return std::hypot(a[0],a[1],a[2]);}
inline Direction3 ref_cross(const Direction3&a,const Direction3&b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline Direction3 ref_rotate(const std::array<double,9>&R,const Direction3&q){Direction3 a{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)a[i]+=R[3*i+j]*q[j];return a;}
inline Direction3 ref_unrotate(const std::array<double,9>&R,const Direction3&q){Direction3 a{};for(int i=0;i<3;++i)for(int j=0;j<3;++j)a[i]+=R[3*j+i]*q[j];return a;}
inline double ref_angle(const Direction3&a,const Direction3&b){return std::atan2(ref_norm(ref_cross(a,b)),direction_dot(a,b));}
inline double ref_rotation_norm_bound(const std::array<double,9>&R){double bound=0;for(int i=0;i<3;++i){double row=0;for(int j=0;j<3;++j){double x=0;for(int k=0;k<3;++k)x+=R[3*k+i]*R[3*k+j];row+=std::abs(x);}bound=std::max(bound,row);}return std::sqrt(bound)*(1+2e-15);}
struct SpatialReferenceConfig {
  static constexpr double ramp_time_s=.10;
  static constexpr double ramp_length_m=BaseConfig::max_speed_mps*ramp_time_s;
  static constexpr double lookup_step_m=.05;
  static constexpr double fast_inverse_local_error_rad=5e-8;
};
struct AngularQuery {Direction3 q{},dq_dzeta{};double arc_error=0;};
// A C1 spherical interpolant of Q, not an independently chosen exit arc.
// Normalized cubic Hermite segments share tangent directions at every knot;
// numerical arc-length reparametrization makes |dq/dzeta|=1. Cusps are rejected.
class ContinuousAngularPath {
 public:
  struct Piece {
    std::array<Direction3,4> b;
    // Local gnomonic chart: b_j=w_j*(normal+x_j*east+y_j*north).
    // x/y are the two intrinsic coordinates. Positive w_j are fixed rational
    // Bezier weights, not another curve degree of freedom. Preserve b for the
    // original conservative cone certificate and derivative/error bounds.
    Direction3 normal{},east{},north{};
    std::array<std::array<double,4>,3> chart_polynomial{}; // w*x, w*y, w
    std::array<Direction3,9> knots{};
    std::array<double,9> arc{},arc_error{};
    double begin=0,begin_error=0,length=0,second_bound=0,speed_bound=0;
  };
  ContinuousAngularPath(){points_.reserve(8192);pieces_.reserve(8192);tangents_.reserve(8192);}
  bool build(const std::vector<Direction3>&input){
    points_.clear();pieces_.clear();tangents_.clear();length_=0;length_error_=0;second_bound_=1;error_.clear();
    for(auto q:input){
      if(!unit_direction(q))return fail("invalid_angular_sample");
      if(points_.empty()){points_.push_back(q);continue;}
      const auto a=points_.back();const double angle=ref_angle(a,q);
      if(angle<1e-11)continue;
      if(angle>direction_pi-1e-6)return fail("ambiguous_antipodal_segment");
      const int count=std::max(1,int(std::ceil(angle/.08)));
      const double sine=std::sin(angle);
      for(int j=1;j<=count;++j){const double t=double(j)/count;
        auto v=ref_add(ref_scale(a,std::sin((1-t)*angle)/sine),q,std::sin(t*angle)/sine);
        if(!unit_direction(v))return fail("invalid_angular_interpolation");
        points_.push_back(v);
      }
      if(points_.size()>8192)return fail("angular_capacity_exceeded");
    }
    if(points_.empty())return fail("missing_angular_curve");
    tangents_.resize(points_.size());
    for(size_t i=0;i<points_.size();++i){
      if(points_.size()==1)break;
      Direction3 t{};
      if(i)t=ref_add(t,ref_scale(ref_add(points_[i],points_[i-1],-1),1/ref_angle(points_[i-1],points_[i])));
      if(i+1<points_.size())t=ref_add(t,ref_scale(ref_add(points_[i+1],points_[i],-1),1/ref_angle(points_[i],points_[i+1])));
      t=ref_add(t,points_[i],-direction_dot(t,points_[i]));
      if(!unit_direction(t))return fail("angular_cusp");
      tangents_[i]=t;
    }
    for(size_t i=1;i<points_.size();++i){
      const double h=ref_angle(points_[i-1],points_[i]);Piece p;
      p.b={points_[i-1],ref_add(points_[i-1],tangents_[i-1],h/3),ref_add(points_[i],tangents_[i],-h/3),points_[i]};
      auto chord=ref_add(p.b[3],p.b[0],-1);const double chord_length=ref_norm(chord);
      if(!(chord_length>0.)||!std::isfinite(chord_length))return fail("zero_angular_chord");
      chord=ref_scale(chord,1/chord_length); // A validated small arc is not a missing direction.
      double nmin=1e9,nmax=0,projmax=0,M1=0,M2=0,minproj=1e9;
      for(auto &b:p.b){nmin=std::min(nmin,direction_dot(b,p.b[0]));nmax=std::max(nmax,ref_norm(b));projmax=std::max(projmax,std::abs(direction_dot(b,chord)));}
      for(int j=0;j<3;++j){auto d=ref_scale(ref_add(p.b[j+1],p.b[j],-1),3);M1=std::max(M1,ref_norm(d));minproj=std::min(minproj,direction_dot(d,chord));}
      for(int j=0;j<2;++j)M2=std::max(M2,ref_norm(ref_scale(ref_add(ref_add(p.b[j+2],p.b[j+1],-2),p.b[j]),6)));
      if(!(nmin>.5))return fail("angular_normalization_bound");
      const double vmin=minproj/nmax-projmax*M1/(nmin*nmin);
      if(!(vmin>h*1e-4))return fail("nonregular_angular_interpolant");
      const double Q2=M2*(1/nmin+nmax/(nmin*nmin))+2*M1*M1/(nmin*nmin)+4*nmax*M1*M1/(nmin*nmin*nmin);
      p.second_bound=2*Q2/(vmin*vmin);second_bound_=std::max(second_bound_,p.second_bound);
      p.speed_bound=M1/nmin;
      prepare_chart(p);
      p.begin=length_;
      p.begin_error=length_error_;
      p.knots[0]=p.b[0];
      for(int j=0;j<8;++j){
        p.knots[j+1]=evaluate(p,double(j+1)/8).q;
        const double nominal=integral_speed(p,double(j)/8,double(j+1)/8);
        const auto bound=arc_bounds(p,p.knots[j],p.knots[j+1],1./8.);
        p.arc[j+1]=p.arc[j]+nominal;
        p.arc_error[j+1]=p.arc_error[j]+std::max(std::abs(nominal-bound[0]),std::abs(bound[1]-nominal))+roundoff;
      }
      p.length=p.arc[8];if(!(p.length>0)||!std::isfinite(p.length))return fail("zero_angular_length");
      length_+=p.length;
      length_error_+=p.arc_error.back()+roundoff;
      pieces_.push_back(p);
    }
    return true;
  }
  AngularQuery query(double zeta,bool fast=false)const{
    if(points_.empty())return {};
    if(pieces_.empty())return {points_.front(),{}};
    if(zeta<=0)return {points_.front(),tangents_.front()};
    if(zeta>=length_)return {points_.back(),tangents_.back(),length_error_};
    zeta=std::clamp(zeta,0.,length_);
    auto it=std::upper_bound(pieces_.begin(),pieces_.end(),zeta,[](double z,const Piece&p){return z<p.begin;});
    if(it!=pieces_.begin())--it;
    const auto &p=*it;const double target=std::clamp(zeta-p.begin,0.,p.length);
    int cell=int(std::upper_bound(p.arc.begin(),p.arc.end(),target)-p.arc.begin())-1;cell=std::clamp(cell,0,7);
    double lo=double(cell)/8,hi=double(cell+1)/8,u=lo+(hi-lo)*(target-p.arc[cell])/(p.arc[cell+1]-p.arc[cell]);
    const double cell_start=lo;
    const double cell_target=target-p.arc[cell];
    auto local_bounds=[&](const AngularQuery &q,double parameter){
      return arc_bounds(p,p.knots[cell],q.q,parameter-cell_start);
    };
    auto result=[&](AngularQuery q,const std::array<double,2> &bound){
      unit_direction(q.dq_dzeta);
      q.arc_error=p.begin_error+p.arc_error[cell]+
        std::max(std::abs(bound[0]-cell_target),std::abs(bound[1]-cell_target))+roundoff;
      return q;
    };
    if(fast){
      // The local chord gives a cheap enclosed arc residual. Its correction
      // is only a proposal. Every accepted result is checked against both bounds.
      for(int attempt=0;attempt<2;++attempt){
        const auto q=evaluate(p,u);
        const auto bound=local_bounds(q,u);
        const double residual=std::max(std::abs(bound[0]-cell_target),std::abs(bound[1]-cell_target));
        if(residual<=SpatialReferenceConfig::fast_inverse_local_error_rad)return result(q,bound);
        const double next=u+(cell_target-(bound[0]+bound[1])*.5)/ref_norm(q.dq_dzeta);
        if(!(next>lo&&next<hi))break;
        u=next;
      }
    }
    for(int j=0;j<14;++j){auto q=evaluate(p,u);const double f=p.arc[cell]+integral_speed(p,double(cell)/8,u)-target;
      if(std::abs(f)<1e-13)break;
      if(f>0)hi=u;else lo=u;
      const double n=u-f/ref_norm(q.dq_dzeta);u=n>lo&&n<hi?n:(lo+hi)/2;
    }
    const auto out=evaluate(p,u);
    return result(out,local_bounds(out,u));
  }
  double length()const{return length_;} double second_bound()const{return second_bound_;}
  double length_error()const{return length_error_;}
  const std::vector<Piece>&pieces()const{return pieces_;}const std::string&error()const{return error_;}
  static AngularQuery evaluate(const Piece&p,double u){
    const auto c=chart_at(p,u);
    const double square=c.x*c.x+c.y*c.y+c.w*c.w,inverse=1/std::sqrt(square);
    const double radial=(c.x*c.dx+c.y*c.dy+c.w*c.dw)/square;
    // Lift only the final direction/derivative back into the observation's
    // three-dimensional frame. This is the identical normalized 3D cubic.
    Direction3 q{},derivative{};
    for(int k=0;k<3;++k){
      q[k]=(p.east[k]*c.x+p.north[k]*c.y+p.normal[k]*c.w)*inverse;
      derivative[k]=(p.east[k]*(c.dx-radial*c.x)+p.north[k]*(c.dy-radial*c.y)+
                     p.normal[k]*(c.dw-radial*c.w))*inverse;
    }
    return {q,derivative};
  }
 private:
  struct ChartValue {double x=0,y=0,w=0,dx=0,dy=0,dw=0;};
  static void prepare_chart(Piece&p){
    p.normal=p.b[0];unit_direction(p.normal);
    // Choose a well-conditioned chart even for almost coincident endpoints
    // and at either world pole. Dividing a tiny endpoint chord is unstable.
    int least=0;for(int k=1;k<3;++k)if(std::abs(p.normal[k])<std::abs(p.normal[least]))least=k;
    Direction3 axis{};axis[least]=1.;p.east=ref_cross(axis,p.normal);unit_direction(p.east);
    p.north=ref_cross(p.normal,p.east);unit_direction(p.north);
    for(int coordinate=0;coordinate<3;++coordinate){
      const auto &axis=coordinate==0?p.east:coordinate==1?p.north:p.normal;
      std::array<double,4> c{};
      for(int j=0;j<4;++j)c[j]=direction_dot(p.b[j],axis);
      // Homogeneous coefficients evaluate the rational 2D chart without
      // repeatedly dividing by its positive scalar Bezier denominator.
      p.chart_polynomial[coordinate]={c[0],3*(c[1]-c[0]),
        3*(c[2]-2*c[1]+c[0]),c[3]-3*c[2]+3*c[1]-c[0]};
    }
  }
  static ChartValue chart_at(const Piece&p,double u){
    ChartValue out;
    double *value[3]={&out.x,&out.y,&out.w},*derivative[3]={&out.dx,&out.dy,&out.dw};
    for(int k=0;k<3;++k){const auto &c=p.chart_polynomial[k];
      *value[k]=((c[3]*u+c[2])*u+c[1])*u+c[0];
      *derivative[k]=(3*c[3]*u+2*c[2])*u+c[1];
    }
    return out;
  }
  static double chart_speed(const Piece&p,double u){
    const auto c=chart_at(p,u);
    // For x=X/W,y=Y/W, the spherical metric is
    // sqrt(x'^2+y'^2+(x*y'-y*x')^2)/(1+x^2+y^2).
    // This homogeneous form cancels W exactly and avoids every 3D lift in
    // Gauss quadrature and angular-arclength inversion.
    const double a=c.y*c.dw-c.w*c.dy,b=c.w*c.dx-c.x*c.dw,d=c.x*c.dy-c.y*c.dx;
    return std::sqrt(a*a+b*b+d*d)/(c.x*c.x+c.y*c.y+c.w*c.w);
  }
  static constexpr double roundoff=64*std::numeric_limits<double>::epsilon();
  static std::array<double,2> arc_bounds(const Piece &p,const Direction3 &a,const Direction3 &b,double du){
    const double chord=ref_norm(ref_add(b,a,-1));
    const double upper_speed=p.speed_bound*std::max(0.,du)*(1+roundoff);
    // |chord|² >= arc² - K² arc⁴/12, from the integrated dot product of
    // unit tangents whose distance is bounded by K times arc separation.
    const double k_arc=p.second_bound*upper_speed;
    const double loss=k_arc*k_arc/12;
    const double upper=loss<.5?std::min(upper_speed,(chord+roundoff)/std::sqrt(1-loss)):upper_speed;
    return {std::max(0.,chord-roundoff),upper+roundoff};
  }
  static double integral_speed(const Piece&p,double a,double b){
    static constexpr double x[4]={.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363};
    static constexpr double w[4]={.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763};
    const double m=(a+b)/2,h=(b-a)/2;double sum=0;for(int j=0;j<4;++j)sum+=w[j]*(chart_speed(p,m-h*x[j])+chart_speed(p,m+h*x[j]));return h*sum;
  }
  bool fail(const char*s){error_=s;return false;}
  std::vector<Direction3> points_,tangents_;std::vector<Piece> pieces_;double length_=0,length_error_=0,second_bound_=1;std::string error_;
};
struct CurvatureSchedule {
  double zeta_end=0,ramp=0,plateau=0,peak=0,length=0;
  static CurvatureSchedule make(double zeta){
    if(!std::isfinite(zeta)||zeta<0)throw std::invalid_argument("invalid accumulated angle");
    CurvatureSchedule p;p.zeta_end=zeta;if(zeta<1e-12)return p;
    const double k=NavigationConfig::reference_max_curvature_per_m,l=SpatialReferenceConfig::ramp_length_m;
    p.ramp=zeta<k*l?std::sqrt(zeta*l/k):l;p.peak=k*p.ramp/l;p.plateau=std::max(0.,zeta/p.peak-p.ramp);p.length=2*p.ramp+p.plateau;return p;
  }
  std::array<double,3> query(double s)const{ // zeta, kappa, d_kappa/d_s
    if(length==0||s>=length)return {zeta_end,0,0};
    if(s<=0)return {0,0,length?peak/ramp:0};
    const double slope=peak/ramp;
    if(s<ramp)return {.5*slope*s*s,slope*s,slope};
    if(s<ramp+plateau)return {.5*peak*ramp+peak*(s-ramp),peak,0};
    const double left=length-s;return {zeta_end-.5*slope*left*left,slope*left,-slope};
  }
};
struct SpatialPosition {double s=0,error_bound_m=0;Direction3 position{};};
struct SpatialSample {double s=0,zeta=0,kappa=0,dkappa_ds=0,position_error_bound_m=0;Direction3 position_W{},tangent_W{},curvature_W{};};
class SpatialReference {
 public:
  SpatialReference(){canonical_.reserve(16384);nodes_.reserve(16384);}
  bool build(const std::vector<Direction3>&Q,const Direction3&p,const std::array<double,9>&R){
    valid_=false;canonical_.clear();nodes_.clear();
    if(!place(p,R))return false;
    if(!path_.build(Q))return false;
    profile_=CurvatureSchedule::make(path_.length());
    // T' is continuous; this bounds its piecewise second derivative. Positive
    // midpoint quadrature has a global error <= M2*h^3/(24*N^2) per panel.
    const double k=NavigationConfig::reference_max_curvature_per_m;
    second_bound_=profile_.length?k/SpatialReferenceConfig::ramp_length_m+k*k*path_.second_bound():0;
    if(profile_.length>500||!std::isfinite(second_bound_))return false;
    canonical_.push_back({});
    tail_direction_=path_.query(path_.length()).q;
    valid_=true;
    const double step=SpatialReferenceConfig::lookup_step_m;
    const std::array<double,3> joins{profile_.ramp,profile_.ramp+profile_.plateau,profile_.length};
    size_t grid=1,join=0;
    while(canonical_.back().s<profile_.length){
      const auto &last=canonical_.back();
      while(join<joins.size() && joins[join]<=last.s)++join;
      const double next_grid=grid*step;
      const double end=std::min(next_grid,joins[join]);
      if(end==next_grid)++grid;
      const auto integral=integrate(last.s,end);
      canonical_.push_back({end,last.error_bound_m+integral.second,ref_add(last.position,integral.first)});
    }
    for(const auto &sample:canonical_)nodes_.push_back(sample.s);
    return true;
  }
  // Shape stays in the observation body coordinates. Change its placement
  // independently of the angular interpolant and integrated positions.
  bool place(const Direction3 &p,const std::array<double,9> &R){
    for(double x:p)if(!std::isfinite(x))return false;
    for(double x:R)if(!std::isfinite(x))return false;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
      double dot=0;
      for(int k=0;k<3;++k)dot+=R[3*k+i]*R[3*k+j];
      if(std::abs(dot-(i==j?1.:0.))>1e-10)return false;
    }
    const double det=R[0]*(R[4]*R[8]-R[5]*R[7])-R[1]*(R[3]*R[8]-R[5]*R[6])+R[2]*(R[3]*R[7]-R[4]*R[6]);
    if(std::abs(det-1)>1e-10)return false;
    anchor_=p;
    rotation_=R;
    rotation_norm_bound_=ref_rotation_norm_bound(R);
    return true;
  }
  SpatialPosition position(double s)const{
    if(!valid_||!std::isfinite(s)||s<0)throw std::invalid_argument("invalid spatial query");
    SpatialPosition result;
    if(s>=profile_.length){
      const auto &end=canonical_.back();
      result={s,end.error_bound_m+path_.length_error()*(s-end.s),
        ref_add(end.position,tail_direction_,s-end.s)};
    }else{
      const auto right=std::lower_bound(canonical_.begin(),canonical_.end(),s,
        [](const SpatialPosition &sample,double at){return sample.s<at;});
      if(right->s==s)result=*right;
      else{
        const auto &left=*std::prev(right);
        const auto integral=integrate(left.s,s);
        result={s,left.error_bound_m+integral.second,ref_add(left.position,integral.first)};
      }
    }
    result.position=ref_add(anchor_,ref_rotate(rotation_,result.position));
    result.error_bound_m*=rotation_norm_bound_;
    result.error_bound_m+=64*std::numeric_limits<double>::epsilon()*(1+ref_norm(anchor_)+s);
    return result;
  }
  SpatialSample query(double s)const{
    const auto p=position(s);
    const auto profile=profile_.query(s);
    const auto q=path_.query(profile[0]);
    return {s,profile[0],profile[1],profile[2],p.error_bound_m,p.position,
      ref_rotate(rotation_,q.q),ref_rotate(rotation_,ref_scale(q.dq_dzeta,profile[1]))};
  }
  // Project onto the existing position-table chords in a bounded arc window.
  // Refine the returned parameter against the continuous curve in the caller.
  double project_chords(const Direction3 &world,double lo,double hi)const{
    const auto p=ref_unrotate(rotation_,ref_add(world,anchor_,-1));
    double best=lo,distance=std::numeric_limits<double>::infinity();
    auto consider=[&](const SpatialPosition &a,const SpatialPosition &b){
      const auto delta=ref_add(b.position,a.position,-1);
      const auto offset=ref_add(p,a.position,-1);
      const double square=direction_dot(delta,delta),width=b.s-a.s;
      if(!(square>0.)||!(width>0.))return;
      const double u=std::clamp(direction_dot(offset,delta)/square,
        (std::max(lo,a.s)-a.s)/width,(std::min(hi,b.s)-a.s)/width);
      const auto error=ref_add(offset,delta,-u);
      const double cost=direction_dot(error,error);
      if(cost<distance){distance=cost;best=a.s+u*width;}
    };
    const auto after=std::upper_bound(canonical_.begin(),canonical_.end(),lo,
      [](double at,const SpatialPosition &sample){return at<sample.s;});
    for(auto right=after;right!=canonical_.end()&&std::prev(right)->s<hi;++right)
      consider(*std::prev(right),*right);
    const auto &end=canonical_.back();
    if(hi>end.s)consider(end,{hi,0.,ref_add(end.position,tail_direction_,hi-end.s)});
    return best;
  }
  void insert_node(double s){
    if(!valid_||s<0||!std::isfinite(s))return;
    auto it=std::lower_bound(nodes_.begin(),nodes_.end(),s,[](double a,double b){return a<b;});
    if(it==nodes_.end()||std::abs(*it-s)>1e-12)nodes_.insert(it,s);
  }
  bool normal_attachment(const Direction3&p,const Direction3&direction,double s)const{if(!valid_||!std::isfinite(s)||s<0)return false;const auto q=query(s);auto d=direction;if(!unit_direction(d))return false;return ref_norm(ref_add(p,q.position_W,-1))<=NavigationConfig::normal_attach_position_error_m+1e-12 && ref_angle(d,q.tangent_W)<=NavigationConfig::normal_attach_angle_deg*direction_pi/180+1e-12;}
  bool continuous_connection(const SpatialSample&previous)const{if(!valid_)return false;const auto q=query(0);return ref_norm(ref_add(previous.position_W,q.position_W,-1))<1e-9&&ref_angle(previous.tangent_W,q.tangent_W)<1e-9&&ref_norm(previous.curvature_W)<1e-9;}
  void invalidate(){valid_=false;}
  bool valid()const{return valid_;}const CurvatureSchedule&profile()const{return profile_;}
  const ContinuousAngularPath&angular_path()const{return path_;}
  const std::vector<double>&nodes()const{return nodes_;}
  const Direction3&anchor()const{return anchor_;}const std::array<double,9>&rotation()const{return rotation_;}
  double tangent_norm_bound()const{return rotation_norm_bound_;}
  double normal_speed_mps()const{return BaseConfig::max_speed_mps;} // never a forced terminal stop
 private:
  std::pair<Direction3,double> integrate(double a,double b)const{
    Direction3 out{};
    double angular_error=0;
    constexpr int N=16;
    const double width=b-a,h=width/N;
    for(int j=0;j<N;++j){
      const auto q=path_.query(profile_.query(a+(j+.5)*h)[0],true);
      out=ref_add(out,q.q,h);
      angular_error+=q.arc_error*h;
    }
    // Arc residual bounds tangent error in Euclidean norm. At fixed s,
    // the triangular/trapezoidal schedule's accumulated angle is
    // 1-Lipschitz with respect to its total angle.
    return {out,second_bound_*width*width*width/(24*N*N)+2e-10*width+
      angular_error+path_.length_error()*width};
  }
  ContinuousAngularPath path_;
  CurvatureSchedule profile_;
  Direction3 anchor_{};
  std::array<double,9>rotation_{};
  Direction3 tail_direction_{};
  std::vector<SpatialPosition>canonical_;
  std::vector<double>nodes_;
  double second_bound_=0,rotation_norm_bound_=1;
  bool valid_=false;
};
} // namespace ego4p0
