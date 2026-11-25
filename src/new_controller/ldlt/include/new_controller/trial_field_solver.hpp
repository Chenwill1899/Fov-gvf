#pragma once
#include "new_controller/grounded_system.hpp"
#include "new_controller/sparse_trial_backend.hpp"

namespace ego4p0 {
// Field adapter for the selected standalone coarse/fine Eigen LDLT backend.
// All solve/caching work is charged to FieldLayer::solve_ns. A phi cache hit
// is distinct from backend symbolic/numeric reuse and reports zero backend time.
class TrialFieldSolver {
 public:
  explicit TrialFieldSolver(SparseTrialMode mode):backend_(mode){}
  void configure(uint32_t maximum_nodes) {
    system_.reserve(maximum_nodes);cache_phi_.reserve(maximum_nodes);cache_boundary_.reserve(maximum_nodes);cache_values_.reserve(maximum_nodes);
    cache_owner_.reserve(maximum_nodes);cache_representative_.reserve(maximum_nodes);cache_offset_.reserve(maximum_nodes+1);
    cache_neighbor_.reserve(4*maximum_nodes);cache_weight_.reserve(4*maximum_nodes);
    cache_stencil_node_.reserve(maximum_nodes);cache_stencil_neighbor_.reserve(maximum_nodes);cache_stencil_weight_.reserve(maximum_nodes);
    boundary_values_.reserve(maximum_nodes);invalidate();
  }
  void invalidate(){valid_=false;backend_.reset();stats_={};}
  void clear_reported_stats(){stats_={};}
  const SparseTrialStats &stats()const{return stats_;}
  void solve_adaptive(AdaptiveCoarseField &a,FieldLayer &l,const HarmonicConfig &cfg) {
    const auto begin=field_now_ns();const auto count=a.nodes;
    if(a.boundary.size()<count||a.phi.size()<count||a.representative.size()<count||a.offset.size()<count+1||a.neighbor.size()<a.edges||a.weight.size()<a.edges||a.offset[0]!=0||a.offset[count]!=a.edges){fail_input(l,"invalid adaptive CSR",begin);return;}
    bool low=false,exit=false;
    for(uint32_t i=0;i<count;++i){if(a.offset[i]>a.offset[i+1]||a.offset[i+1]>a.edges){fail_input(l,"invalid adaptive offsets",begin);return;}low|=FieldBoundary(a.boundary[i])==FieldBoundary::Low;exit|=FieldBoundary(a.boundary[i])==FieldBoundary::Exit;}
    for(uint32_t e=0;e<a.edges;++e)if(a.neighbor[e]>=count||!std::isfinite(a.weight[e])||!(a.weight[e]>0)){fail_input(l,"invalid adaptive edge",begin);return;}
    if(!low||!exit){fail_input(l,"adaptive graph lacks Low/Exit",begin);return;}
    solve(count,a.boundary,a.phi,l,cfg,
      [&](uint32_t i,auto add){for(uint32_t e=a.offset[i];e<a.offset[i+1];++e)add(a.neighbor[e],a.weight[e]);},
      [&]{return system_.adaptive(a);},
      [&]{return cache_kind_==OperatorKind::Adaptive&&cache_owner_==a.owner&&cache_representative_.size()==count&&std::equal(cache_representative_.begin(),cache_representative_.end(),a.representative.begin())&&cache_offset_.size()==count+1&&std::equal(cache_offset_.begin(),cache_offset_.end(),a.offset.begin())&&cache_neighbor_.size()==a.edges&&cache_weight_.size()==a.edges&&std::equal(cache_neighbor_.begin(),cache_neighbor_.end(),a.neighbor.begin())&&std::equal(cache_weight_.begin(),cache_weight_.end(),a.weight.begin());},
      [&]{cache_kind_=OperatorKind::Adaptive;cache_owner_=a.owner;cache_representative_.assign(a.representative.begin(),a.representative.begin()+count);cache_offset_.assign(a.offset.begin(),a.offset.begin()+count+1);cache_neighbor_.assign(a.neighbor.begin(),a.neighbor.begin()+a.edges);cache_weight_.assign(a.weight.begin(),a.weight.begin()+a.edges);},
      [&](uint32_t i){const auto representative=a.representative[i];return representative<cache_owner_.size()&&cache_owner_[representative]<cache_phi_.size() ? cache_phi_[cache_owner_[representative]] : .5;},begin);
  }
  void solve_layer(const SphericalStencil &s,FieldLayer &l,const HarmonicConfig &cfg) {
    const auto begin=field_now_ns();
    if(l.phi.size()!=s.nodes.size()||l.boundary.size()!=s.nodes.size()){fail_input(l,"fine layer/stencil dimension mismatch",begin);return;}
    solve(static_cast<uint32_t>(l.phi.size()),l.boundary,l.phi,l,cfg,
      [&](uint32_t i,auto add){for(uint32_t k=0;k<4;++k)add(s.nodes[i].neighbor[k],s.nodes[i].weight[k]);},
      [&]{return system_.layer(s,l);},
      [&]{
        if(cache_kind_!=OperatorKind::Layer||cache_stencil_count_!=s.nodes.size())return false;
        for(uint32_t k=0;k<cache_stencil_node_.size();++k){const auto &n=s.nodes[cache_stencil_node_[k]];if(n.neighbor!=cache_stencil_neighbor_[k]||n.weight!=cache_stencil_weight_[k])return false;}
        return true;
      },
      [&]{
        cache_kind_=OperatorKind::Layer;cache_stencil_count_=static_cast<uint32_t>(s.nodes.size());cache_stencil_node_=system_.node;
        cache_stencil_neighbor_.resize(cache_stencil_node_.size());cache_stencil_weight_.resize(cache_stencil_node_.size());
        for(uint32_t k=0;k<cache_stencil_node_.size();++k){const auto &n=s.nodes[cache_stencil_node_[k]];cache_stencil_neighbor_[k]=n.neighbor;cache_stencil_weight_[k]=n.weight;}
      },
      [&](uint32_t i){return i<cache_phi_.size() ? cache_phi_[i] : .5;},begin);
  }
 private:
  void fail_input(FieldLayer &l,const std::string &error,int64_t begin){stats_={};stats_.error=error;valid_=false;l.status=FieldStatus::SolveFailed;l.reused=false;l.iterations=0;l.residual=std::numeric_limits<double>::infinity();stats_.residual=l.residual;l.cache_ns=l.assembly_ns=l.iteration_ns=l.residual_ns=l.store_ns=0;l.solve_ns=field_now_ns()-begin;}
  template<class Edges,class Assemble,class ExtraSame,class SaveExtra,class Warm>
  void solve(uint32_t count,const std::vector<uint8_t> &boundary,std::vector<double> &phi,FieldLayer &l,const HarmonicConfig &cfg,
             Edges edges,Assemble assemble,ExtraSame extra_same,SaveExtra save_extra,Warm warm,int64_t begin) {
    stats_={};l.iterations=0;l.nodes=0;l.residual=std::numeric_limits<double>::infinity();l.reused=false;l.warm_started=valid_;l.constant_boundary=false;
    l.cache_ns=l.assembly_ns=l.iteration_ns=l.residual_ns=l.store_ns=0;
    auto finish=[&](bool ok,const std::string &error=std::string{}){l.status=ok ? FieldStatus::Valid : FieldStatus::SolveFailed;stats_.ok=ok;stats_.residual=l.residual;if(!error.empty())stats_.error=error;valid_=ok;l.solve_ns=field_now_ns()-begin;};
    const auto assembly=field_now_ns();
    if(boundary.size()<count||phi.size()<count||!(cfg.residual_tolerance>0)||!std::isfinite(cfg.residual_tolerance)){finish(false,"invalid field adapter input");return;}
    boundary_values_.assign(count,0.);uint32_t fixed=0,unknowns=0;double minimum=std::numeric_limits<double>::infinity(),maximum=-minimum;bool ok=true;
    for(uint32_t i=0;i<count;++i) {
      const auto bc=FieldBoundary(boundary[i]);
      if(bc==FieldBoundary::Outside){phi[i]=std::numeric_limits<double>::quiet_NaN();continue;}
      ++l.nodes;
      if(bc==FieldBoundary::Interior){++unknowns;continue;}
      if(bc==FieldBoundary::Low)phi[i]=0.;else if(bc==FieldBoundary::Exit)phi[i]=1.;else if(bc!=FieldBoundary::Coupling)ok=false;
      boundary_values_[i]=phi[i];++fixed;minimum=std::min(minimum,phi[i]);maximum=std::max(maximum,phi[i]);
      if(!std::isfinite(phi[i])||phi[i]<-1e-5||phi[i]>1+1e-5)ok=false;
    }
    l.constant_boundary=fixed>0&&maximum-minimum<1e-12;
    ok&=fixed>0&&!l.constant_boundary;
    if(!ok){l.assembly_ns=field_now_ns()-assembly;finish(false,"missing/constant/nonfinite Dirichlet boundary");return;}
    l.assembly_ns=field_now_ns()-assembly;
    const auto cache=field_now_ns();
    // The canonical boundary types/values and exact operator inputs determine
    // the same grounded CSR and RHS. Check them before assembling that CSR.
    // Fine snapshots include every Interior row's four neighbors/weights;
    // fixed/Outside rows have no outgoing contribution to A, b or FV residual.
    // Adaptive snapshots retain the complete owner map and active graph.
    const bool same=valid_&&cache_tolerance_==cfg.residual_tolerance&&cache_phi_.size()==count&&cache_boundary_.size()==count&&std::equal(cache_boundary_.begin(),cache_boundary_.end(),boundary.begin())&&cache_values_==boundary_values_&&extra_same();
    l.cache_ns=field_now_ns()-cache;
    if(same){const auto store=field_now_ns();std::copy(cache_phi_.begin(),cache_phi_.end(),phi.begin());l.reused=true;l.residual=cache_residual_;stats_.unknowns=unknowns;stats_.nonzeros=static_cast<uint32_t>(system_.val.size());l.store_ns=field_now_ns()-store;finish(true);return;}
    const auto assemble_begin=field_now_ns();
    if(unknowns&&!assemble()){l.assembly_ns+=field_now_ns()-assemble_begin;finish(false,system_.error);return;}
    if(!unknowns){system_.ptr.clear();system_.col.clear();system_.val.clear();system_.rhs.clear();system_.node.clear();system_.x.clear();}
    l.assembly_ns+=field_now_ns()-assemble_begin;
    for(uint32_t k=0;k<system_.node.size();++k){const double value=valid_ ? warm(system_.node[k]) : .5;system_.x[k]=std::isfinite(value) ? value : .5;}
    if(unknowns) {
      ok=backend_.solve(system_.ptr,system_.col,system_.val,system_.rhs,system_.x,cfg.residual_tolerance);stats_=backend_.stats();
      l.iterations=stats_.iterations;l.iteration_ns=stats_.import_ns+stats_.analysis_ns+stats_.factor_ns+stats_.solve_ns;l.residual_ns=stats_.verify_ns;
      if(!ok){if(stats_.verify_ns>0)l.residual=stats_.residual;finish(false,stats_.error);return;}
      for(uint32_t k=0;k<system_.node.size();++k)phi[system_.node[k]]=system_.x[k];
    }
    const auto verify=field_now_ns();l.residual=0.;
    for(uint32_t i=0;i<count;++i) {
      const auto bc=FieldBoundary(boundary[i]);if(bc==FieldBoundary::Outside)continue;
      if(!std::isfinite(phi[i])||phi[i]<-1e-5||phi[i]>1+1e-5)ok=false;
      if(bc!=FieldBoundary::Interior)continue;
      double diagonal=0,ap=0,rhs=0;
      edges(i,[&](uint32_t j,double w){if(j==UINT32_MAX||j>=count||FieldBoundary(boundary[j])==FieldBoundary::Outside)return;diagonal+=w;if(FieldBoundary(boundary[j])==FieldBoundary::Interior)ap-=w*phi[j];else rhs+=w*phi[j];});
      ap+=diagonal*phi[i];const double residual=std::abs(rhs-ap)/diagonal;
      if(!(diagonal>0)||!std::isfinite(residual))ok=false;else l.residual=std::max(l.residual,residual);
    }
    ok&=std::isfinite(l.residual)&&l.residual<=cfg.residual_tolerance;l.residual_ns+=field_now_ns()-verify;
    const auto store=field_now_ns();
    if(ok){cache_phi_.assign(phi.begin(),phi.begin()+count);cache_boundary_.assign(boundary.begin(),boundary.begin()+count);cache_values_=boundary_values_;cache_residual_=l.residual;cache_tolerance_=cfg.residual_tolerance;save_extra();}
    l.store_ns=field_now_ns()-store;finish(ok,ok ? std::string{} : "FV residual/range verification failed");
  }
  SparseTrialBackend backend_;SparseTrialStats stats_;GroundedSystem system_;
  enum class OperatorKind:uint8_t {None,Adaptive,Layer};OperatorKind cache_kind_=OperatorKind::None;
  uint32_t cache_stencil_count_=0;std::vector<uint32_t>cache_stencil_node_;
  std::vector<std::array<uint32_t,4>>cache_stencil_neighbor_;std::vector<std::array<double,4>>cache_stencil_weight_;
  std::vector<double> cache_phi_,cache_values_,boundary_values_,cache_weight_;
  std::vector<uint8_t>cache_boundary_;std::vector<uint32_t>cache_owner_,cache_representative_,cache_offset_,cache_neighbor_;
  double cache_tolerance_=0.,cache_residual_=0.;bool valid_=false;
};
} // namespace ego4p0
