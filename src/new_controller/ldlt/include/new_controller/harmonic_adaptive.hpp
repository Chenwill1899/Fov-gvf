#pragma once
#include "new_controller/harmonic_solver.hpp"
#include "new_controller/harmonic_edge_merge.hpp"
#include "new_controller/harmonic_preconditioner.hpp"
#include "new_controller/trial_field_solver.hpp"
namespace ego4p0 {
// Agglomerate observed fine FV cells. Whole 3x3 cells share one unknown;
// mixed or Dirichlet-cut cells retain only their observed 1-degree children.
// Summed fine-face conductances preserve symmetry and missing-face Neumann.
class AdaptiveCoarseSolver {
 public:
  void configure(const SphericalStencil &f,const SphericalStencil &c) {
    f_=&f;c_=&c;const auto n=f.nodes.size();
    for(auto *v:{&diag_,&rhs_,&r_,&z_,&p_,&ap_,&cache_phi_})v->resize(n);
    active_nodes_.reserve(n);cache_owner_.resize(n);cache_bc_.resize(n);cursor_.resize(n);labels_.resize(n);valid_=false;
    preconditioner_.configure(static_cast<uint32_t>(n),static_cast<uint32_t>(4*n));
    if(external_)external_->configure(static_cast<uint32_t>(n));
  }
  void invalidate(){valid_=false;preconditioner_.invalidate();if(external_)external_->invalidate();}
  void set_trial_solver(SparseTrialMode mode){external_=std::make_unique<TrialFieldSolver>(mode);if(f_)external_->configure(static_cast<uint32_t>(f_->nodes.size()));valid_=false;preconditioner_.invalidate();}
  void clear_trial_stats(){preconditioner_.begin_solve();if(external_)external_->clear_reported_stats();}
  void clear_trial_solver(){external_.reset();valid_=false;preconditioner_.invalidate();}
  const SparseTrialStats &trial_stats()const{static const SparseTrialStats empty;return external_ ? external_->stats() : empty;}
  void set_preconditioner(CoarsePreconditioner mode) {if(mode!=preconditioner_.mode()){preconditioner_.set_mode(mode);valid_=false;}}
  CoarsePreconditioner preconditioner_mode() const {return preconditioner_.mode();}
  const CoarsePreconditionerStats &preconditioner_stats() const {return preconditioner_.stats();}
  void build(const std::vector<uint8_t> &component,const FieldLayer &regular,uint32_t low,uint32_t target,double cap,AdaptiveCoarseField &a) {
    active_nodes_.clear();
    for(uint32_t i=0;i<component.size();++i)if(component[i])active_nodes_.push_back(i);
    build(component,active_nodes_,regular,low,target,cap,a);
  }
  void build(const std::vector<uint8_t> &component,const std::vector<uint32_t> &active,
             const FieldLayer &regular,uint32_t low,uint32_t target,double cap,AdaptiveCoarseField &a) {
    const auto nf=f_->layout.azimuth_bins,nc=c_->layout.azimuth_bins;
    std::fill(a.owner.begin(),a.owner.end(),UINT32_MAX);a.nodes=a.edges=a.refined_nodes=0;
    const double threshold=std::cos(std::min(cap,std::acos(std::clamp(f_->node_cosine(low,target),-1.,1.))*.24));
    std::fill(labels_.begin(),labels_.end(),uint8_t(FieldBoundary::Outside));
    for(uint32_t i:active) {
      labels_[i]=uint8_t(i==target||f_->node_cosine(i,target)>=threshold ? FieldBoundary::Exit : i==low||f_->node_cosine(i,low)>=threshold ? FieldBoundary::Low : FieldBoundary::Interior);
    }
    for(uint32_t c=0;c<c_->nodes.size();++c) {
      const auto x=(c%nc)*3,y=(c/nc)*3,first=y*nf+x;const auto label=labels_[first];
      bool aggregate=regular.state[c]==uint8_t(CandidateStatus::Available)&&component[first];
      for(uint32_t dy=0;dy<3;++dy)for(uint32_t dx=0;dx<3;++dx)aggregate&=component[(y+dy)*nf+x+dx]&&labels_[(y+dy)*nf+x+dx]==label;
      if(aggregate) {
        const auto node=a.nodes++;a.representative[node]=(y+1)*nf+x+1;a.boundary[node]=label;a.refined[node]=0;
        for(uint32_t dy=0;dy<3;++dy)for(uint32_t dx=0;dx<3;++dx)a.owner[(y+dy)*nf+x+dx]=node;
      }else for(uint32_t dy=0;dy<3;++dy)for(uint32_t dx=0;dx<3;++dx) {
        const auto i=(y+dy)*nf+x+dx;if(!component[i])continue;const auto node=a.nodes++;a.owner[i]=node;a.representative[node]=i;a.boundary[node]=labels_[i];a.refined[node]=1;++a.refined_nodes;
      }
    }
    std::fill_n(a.offset.begin(),a.nodes+1,0);
    for(uint32_t i:active)for(auto j:f_->nodes[i].neighbor)if(j!=UINT32_MAX&&component[j]&&a.owner[i]!=a.owner[j])++a.offset[a.owner[i]+1];
    for(uint32_t i=0;i<a.nodes;++i)a.offset[i+1]+=a.offset[i];
    std::copy_n(a.offset.begin(),a.nodes,cursor_.begin());
    for(uint32_t i:active)for(uint32_t k=0;k<4;++k) {
      const auto j=f_->nodes[i].neighbor[k];if(j==UINT32_MAX||!component[j]||a.owner[i]==a.owner[j])continue;
      const auto e=cursor_[a.owner[i]]++;a.neighbor[e]=a.owner[j];a.weight[e]=f_->nodes[i].weight[k];
    }
    a.edges=a.offset[a.nodes];a.raw_edges=a.edges;const auto merge_begin=field_now_ns();
    collapse_parallel_faces(a);a.edge_merge_ns=field_now_ns()-merge_begin;
  }
  void solve(AdaptiveCoarseField &a,FieldLayer &l,const HarmonicConfig &cfg) {
    preconditioner_.begin_solve();
    if(external_){external_->solve_adaptive(a,l,cfg);return;}
    const auto begin=field_now_ns();l.nodes=a.nodes;l.iterations=0;l.reused=false;l.warm_started=valid_;l.residual=0;l.cache_ns=l.assembly_ns=l.iteration_ns=l.residual_ns=l.store_ns=0;
    bool same=valid_&&cache_nodes_==a.nodes&&cache_residual_<=cfg.residual_tolerance&&preconditioner_.matches_structure(a);
    for(uint32_t i=0;i<a.owner.size();++i)if(a.owner[i]!=cache_owner_[i])same=false;
    for(uint32_t i=0;i<a.nodes;++i)if(a.boundary[i]!=cache_bc_[i])same=false;
    l.cache_ns=field_now_ns()-begin;
    if(same){for(uint32_t i=0;i<a.nodes;++i)a.phi[i]=cache_phi_[a.representative[i]];l.residual=cache_residual_;l.status=FieldStatus::Valid;l.reused=true;l.store_ns=field_now_ns()-begin-l.cache_ns;l.solve_ns=field_now_ns()-begin;return;}
    const auto assembly=field_now_ns();bool low=false,exit=false,ok=a.nodes>1;
    for(uint32_t i=0;i<a.nodes;++i) {
      const auto bc=FieldBoundary(a.boundary[i]);low|=bc==FieldBoundary::Low;exit|=bc==FieldBoundary::Exit;
      a.phi[i]=bc==FieldBoundary::Low ? 0. : bc==FieldBoundary::Exit ? 1. : valid_&&std::isfinite(cache_phi_[a.representative[i]]) ? cache_phi_[a.representative[i]] : .5;diag_[i]=rhs_[i]=0;
      for(uint32_t e=a.offset[i];e<a.offset[i+1];++e){diag_[i]+=a.weight[e];if(FieldBoundary(a.boundary[a.neighbor[e]])==FieldBoundary::Exit)rhs_[i]+=a.weight[e];}
      if(bc==FieldBoundary::Interior&&diag_[i]<=0)ok=false;
    }
    ok&=low&&exit;l.assembly_ns=field_now_ns()-assembly;
    auto apply=[&](const auto &v,auto &out){for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior){double sum=diag_[i]*v[i];for(uint32_t e=a.offset[i];e<a.offset[i+1];++e)if(FieldBoundary(a.boundary[a.neighbor[e]])==FieldBoundary::Interior)sum-=a.weight[e]*v[a.neighbor[e]];out[i]=sum;}};
    auto dot=[&](const auto &x,const auto &y){double sum=0;for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior)sum+=x[i]*y[i];return sum;};
    if(ok)ok=preconditioner_.prepare(a,diag_);
    const auto iteration=field_now_ns();
    if(ok) {
      apply(a.phi,ap_);
      for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior) {
        r_[i]=rhs_[i]-ap_[i];l.residual=std::max(l.residual,std::abs(r_[i])/diag_[i]);
      }
      ok=preconditioner_.apply(r_,z_);std::copy_n(z_.begin(),a.nodes,p_.begin());double rz=dot(r_,z_);
      while(ok&&l.residual>cfg.residual_tolerance&&l.iterations<cfg.max_iterations) {
        apply(p_,ap_);const double denom=dot(p_,ap_);
        if(!(denom>0)||!std::isfinite(denom)||!(rz>0)||!std::isfinite(rz)){ok=false;break;}
        const double alpha=rz/denom;
        for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior){a.phi[i]+=alpha*p_[i];r_[i]-=alpha*ap_[i];}
        ++l.iterations;l.residual=0;
        for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior)l.residual=std::max(l.residual,std::abs(r_[i])/diag_[i]);
        if(!std::isfinite(l.residual)){ok=false;break;}
        if(l.residual<=cfg.residual_tolerance)break;
        if(!preconditioner_.apply(r_,z_)){ok=false;break;}
        const double next=dot(r_,z_);if(!(next>0)||!std::isfinite(next)){ok=false;break;}
        const double beta=next/rz;rz=next;
        for(uint32_t i=0;i<a.nodes;++i)if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior)p_[i]=z_[i]+beta*p_[i];
      }
    }
    l.iteration_ns=field_now_ns()-iteration;const auto residual=field_now_ns();
    if(ok){apply(a.phi,ap_);l.residual=0;for(uint32_t i=0;i<a.nodes;++i){if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior){const double error=std::abs(rhs_[i]-ap_[i])/diag_[i];if(!std::isfinite(error))ok=false;else l.residual=std::max(l.residual,error);}if(!std::isfinite(a.phi[i])||a.phi[i]<-1e-5||a.phi[i]>1+1e-5)ok=false;}ok&=l.residual<=cfg.residual_tolerance;}
    l.residual_ns=field_now_ns()-residual;const auto save=field_now_ns();l.status=ok ? FieldStatus::Valid : FieldStatus::SolveFailed;valid_=ok;
    if(ok){cache_nodes_=a.nodes;cache_residual_=l.residual;std::copy(a.owner.begin(),a.owner.end(),cache_owner_.begin());std::copy_n(a.boundary.begin(),a.nodes,cache_bc_.begin());for(uint32_t i=0;i<a.owner.size();++i)cache_phi_[i]=a.owner[i]==UINT32_MAX ? std::numeric_limits<double>::quiet_NaN() : a.phi[a.owner[i]];}
    l.store_ns=field_now_ns()-save;l.solve_ns=field_now_ns()-begin;
  }
 private:
  std::unique_ptr<TrialFieldSolver> external_;
  AdaptiveCoarsePreconditioner preconditioner_;
  const SphericalStencil *f_=nullptr,*c_=nullptr;
  std::vector<double>diag_,rhs_,r_,z_,p_,ap_,cache_phi_;std::vector<uint32_t>cache_owner_,cursor_,active_nodes_;std::vector<uint8_t>cache_bc_,labels_;
  bool valid_=false;uint32_t cache_nodes_=0;double cache_residual_=0;
};
} // namespace ego4p0
