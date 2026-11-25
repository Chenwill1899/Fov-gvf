#pragma once
#include "new_controller/harmonic_types.hpp"

namespace ego4p0 {
// Cell-centred finite-volume Laplace-Beltrami operator. Multiplying by
// cos(beta) d_alpha d_beta yields symmetric positive edge conductances.
// Periodic alpha; polar faces have zero measure. No pole division in solver.
class SphericalStencil {
 public:
  struct Node {std::array<uint32_t,4> neighbor{};std::array<double,4> weight{};Direction3 direction{}; spherical::Bearing bearing{};};
  void configure(double degrees,bool angles_only=false) {
    angular_storage=angles_only;layout.configure(degrees,degrees);nodes.resize(layout.cells());
    const uint32_t na=layout.azimuth_bins,nb=layout.elevation_bins;
    for(uint32_t b=0;b<nb;++b)for(uint32_t a=0;a<na;++a) {
      auto &n=nodes[b*na+a];const double beta=-direction_pi/2+(b+.5)*layout.elevation_step;
      n.bearing={-direction_pi+(a+.5)*layout.azimuth_step,beta};
      if(!angles_only)n.direction=sphere_point(n.bearing.az,beta);
      n.neighbor={b*na+(a+na-1)%na,b*na+(a+1)%na,b ? (b-1)*na+a : UINT32_MAX,b+1<nb ? (b+1)*na+a : UINT32_MAX};
      n.weight={layout.elevation_step/(std::cos(beta)*layout.azimuth_step),layout.elevation_step/(std::cos(beta)*layout.azimuth_step),
        b ? std::cos(-direction_pi/2+b*layout.elevation_step)*layout.azimuth_step/layout.elevation_step : 0.,
        b+1<nb ? std::cos(-direction_pi/2+(b+1)*layout.elevation_step)*layout.azimuth_step/layout.elevation_step : 0.};
    }
    if(angular_storage){
      latitude_sine.resize(nb);latitude_cosine.resize(nb);
      for(uint32_t row=0;row<nb;++row){
        const auto latitude=nodes[row*na].bearing.el;
        latitude_sine[row]=std::sin(latitude);
        latitude_cosine[row]=spherical::cos_lat(latitude);
      }
      // Subtract the actual grid longitudes, including across the seam. A
      // rounded index difference can otherwise change exact extremum ties.
      longitude_cosine.resize(size_t(na)*na);
      for(uint32_t target=0;target<na;++target)
        for(uint32_t column=0;column<na;++column)
          longitude_cosine[size_t(target)*na+column]=std::cos(nodes[target].bearing.az-nodes[column].bearing.az);
    }
  }
  uint32_t cell(const Direction3 &q)const {return *layout.cell(std::atan2(q[1],q[0]),std::atan2(q[2],std::hypot(q[0],q[1])));}
  void prepare_goal(spherical::Bearing goal) {
    goal_sine=std::sin(goal.el);goal_cosine=spherical::cos_lat(goal.el);
    goal_longitudes.resize(layout.azimuth_bins);
    for(uint32_t column=0;column<layout.azimuth_bins;++column)
      goal_longitudes[column]=std::cos(goal.az-nodes[column].bearing.az);
  }
  double goal_score(uint32_t cell)const {
    const auto columns=layout.azimuth_bins,row=cell/columns;
    return std::clamp(latitude_sine[row]*goal_sine+
                      latitude_cosine[row]*goal_cosine*goal_longitudes[cell%columns],-1.,1.);
  }
  double node_cosine(uint32_t a,uint32_t b)const {
    if(!angular_storage)return direction_dot(nodes[a].direction,nodes[b].direction);
    const auto columns=layout.azimuth_bins;
    const auto row=a/columns,target_row=b/columns;
    const double delta=longitude_cosine[size_t(b%columns)*columns+a%columns];
    return std::clamp(latitude_sine[row]*latitude_sine[target_row]+
                      latitude_cosine[row]*latitude_cosine[target_row]*delta,-1.,1.);
  }
  bool angular_storage=false;
  DirectionGridLayout layout;std::vector<Node> nodes;
 private:
  std::vector<double> latitude_sine,latitude_cosine,longitude_cosine;
  std::vector<double> goal_longitudes;
  double goal_sine=0.,goal_cosine=0.;
};
class HarmonicLinearSolver {
 public:
  void configure(const SphericalStencil &s) {
    stencil_=&s;const auto n=s.nodes.size();diagonal_.resize(n);rhs_.resize(n);r_.resize(n);z_.resize(n);p_.resize(n);ap_.resize(n);
    cached_phi_.assign(n,.5);cached_value_.resize(n);cached_boundary_.resize(n);active_.reserve(n);valid_=false;
  }
  void invalidate() {valid_=false;}
  void solve(FieldLayer &l,const HarmonicConfig &config) {
    const int64_t begin=field_now_ns();l.iterations=0;l.reused=false;l.warm_started=valid_;l.nodes=0;l.residual=0.;
    l.cache_ns=l.assembly_ns=l.iteration_ns=l.residual_ns=l.store_ns=0;
    bool same=valid_;
    for(uint32_t i=0;i<l.phi.size();++i) {
      const auto b=FieldBoundary(l.boundary[i]);if(b!=FieldBoundary::Outside)++l.nodes;
      if(l.boundary[i]!=cached_boundary_[i] || (b!=FieldBoundary::Interior && b!=FieldBoundary::Outside && l.phi[i]!=cached_value_[i]))same=false;
    }
    l.cache_ns=field_now_ns()-begin;
    if(same) {std::copy(cached_phi_.begin(),cached_phi_.end(),l.phi.begin());l.status=FieldStatus::Valid;l.reused=true;l.residual=cached_residual_;l.store_ns=field_now_ns()-begin-l.cache_ns;l.solve_ns=field_now_ns()-begin;return;}
    const auto assembly=field_now_ns();
    active_.clear();uint32_t fixed=0;double minimum=std::numeric_limits<double>::infinity(),maximum=-minimum;
    for(uint32_t i=0;i<l.phi.size();++i) {
      cached_value_[i]=l.phi[i];cached_boundary_[i]=l.boundary[i];const auto b=FieldBoundary(l.boundary[i]);
      if(b==FieldBoundary::Outside) {l.phi[i]=std::numeric_limits<double>::quiet_NaN();continue;}
      if(b!=FieldBoundary::Interior) {++fixed;minimum=std::min(minimum,l.phi[i]);maximum=std::max(maximum,l.phi[i]);continue;}
      l.phi[i]=valid_ && std::isfinite(cached_phi_[i]) ? cached_phi_[i] : std::isfinite(l.phi[i]) ? l.phi[i] : .5;active_.push_back(i);
      double d=0,rhs=0;const auto &n=stencil_->nodes[i];
      for(uint32_t k=0;k<4;++k) {
        const auto j=n.neighbor[k];if(j==UINT32_MAX||!l.boundary[j])continue;
        d+=n.weight[k];if(FieldBoundary(l.boundary[j])!=FieldBoundary::Interior)rhs+=n.weight[k]*l.phi[j];
      }
      diagonal_[i]=d;rhs_[i]=rhs;
    }
    l.constant_boundary=fixed>0&&maximum-minimum<1e-12;
    bool ok=fixed>0&&!l.constant_boundary;
    for(auto i:active_)if(diagonal_[i]<=0)ok=false;
    l.assembly_ns=field_now_ns()-assembly;
    auto dot=[&](const auto &a,const auto &b){double out=0;for(auto i:active_)out+=a[i]*b[i];return out;};
    auto apply=[&](const std::vector<double> &v,std::vector<double> &out){
      for(auto i:active_) {double sum=diagonal_[i]*v[i];const auto &n=stencil_->nodes[i];for(uint32_t k=0;k<4;++k) {const auto j=n.neighbor[k];if(j!=UINT32_MAX&&FieldBoundary(l.boundary[j])==FieldBoundary::Interior)sum-=n.weight[k]*v[j];}out[i]=sum;}
    };
    const auto iteration=field_now_ns();
    if(ok) {
      apply(l.phi,ap_);l.residual=0.;
      for(auto i:active_) {r_[i]=rhs_[i]-ap_[i];z_[i]=r_[i]/diagonal_[i];p_[i]=z_[i];l.residual=std::max(l.residual,std::abs(z_[i]));}
      double rz=dot(r_,z_);
      while(l.residual>config.residual_tolerance && l.iterations<config.max_iterations) {
        apply(p_,ap_);const double denominator=dot(p_,ap_);
        if(!std::isfinite(denominator)||denominator<=0||!std::isfinite(rz)) {ok=false;break;}
        const double alpha=rz/denominator;
        for(auto i:active_) {l.phi[i]+=alpha*p_[i];r_[i]-=alpha*ap_[i];z_[i]=r_[i]/diagonal_[i];}
        const double next=dot(r_,z_),beta=next/rz;rz=next;++l.iterations;l.residual=0.;
        for(auto i:active_) {l.residual=std::max(l.residual,std::abs(z_[i]));p_[i]=z_[i]+beta*p_[i];}
      }
    }
    l.iteration_ns=field_now_ns()-iteration;const auto residual=field_now_ns();
    if(ok) {
      // Recompute a true residual; recursive CG residual alone is not evidence.
      apply(l.phi,ap_);l.residual=0.;for(auto i:active_)l.residual=std::max(l.residual,std::abs(rhs_[i]-ap_[i])/diagonal_[i]);
      if(l.residual>config.residual_tolerance||!std::isfinite(l.residual))ok=false;
      for(uint32_t i=0;i<l.phi.size();++i)if(l.boundary[i] && (!std::isfinite(l.phi[i])||l.phi[i]<-1e-5||l.phi[i]>1+1e-5))ok=false;
    }
    l.residual_ns=field_now_ns()-residual;const auto save=field_now_ns();
    l.status=ok ? FieldStatus::Valid : FieldStatus::SolveFailed;valid_=ok;
    if(ok){cached_phi_=l.phi;cached_residual_=l.residual;}
    l.store_ns=field_now_ns()-save;
    l.solve_ns=field_now_ns()-begin;
  }
 private:
  const SphericalStencil *stencil_=nullptr;
  std::vector<double>diagonal_,rhs_,r_,z_,p_,ap_,cached_phi_,cached_value_;
  std::vector<uint8_t>cached_boundary_;std::vector<uint32_t>active_;bool valid_=false;double cached_residual_=0;
};
} // namespace ego4p0
