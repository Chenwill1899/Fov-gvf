#pragma once
#include "new_controller/harmonic_types.hpp"

#ifndef EGO4P0_COARSE_PC_MODE
#define EGO4P0_COARSE_PC_MODE 0
#endif
static_assert(EGO4P0_COARSE_PC_MODE>=0&&EGO4P0_COARSE_PC_MODE<=2,"invalid coarse preconditioner mode");

namespace ego4p0 {
enum class CoarsePreconditioner : uint8_t {Jacobi,SymmetricGaussSeidel,IC0};
inline const char *coarse_preconditioner_name(CoarsePreconditioner p) {
  switch(p) {case CoarsePreconditioner::Jacobi:return "jacobi";
    case CoarsePreconditioner::SymmetricGaussSeidel:return "sgs";
    case CoarsePreconditioner::IC0:return "ic0";}
  return "invalid";
}
struct CoarsePreconditionerStats {
  int64_t setup_ns=0,apply_ns=0;
  uint32_t applications=0,failure_node=UINT32_MAX;
  double failure_pivot=std::numeric_limits<double>::quiet_NaN();
  bool setup_reused=false,ready=false;
};

// The matrix is the Interior principal submatrix of the symmetric graph
// Laplacian, with the supplied diagonal retaining Dirichlet conductances.
// SGS is SSOR with omega=1. IC(0) stores an unshifted unit-L / D / L^T
// factorization and explicitly rejects a nonpositive/nonfinite pivot.
// No diagonal shift, fallback, equation change, or tolerance relaxation.
class AdaptiveCoarsePreconditioner {
 public:
  void configure(uint32_t maximum_nodes,uint32_t maximum_edges) {
    row_.resize(maximum_nodes+1);upper_row_.resize(maximum_nodes+1);
    lower_.resize(maximum_edges);upper_.resize(maximum_edges);cursor_.resize(maximum_nodes);
    pivot_.resize(maximum_nodes);work_.resize(maximum_nodes);
    cache_offset_.resize(maximum_nodes+1);cache_neighbor_.resize(maximum_edges);
    cache_weight_.resize(maximum_edges);cache_diagonal_.resize(maximum_nodes);cache_boundary_.resize(maximum_nodes);
    valid_=false;begin_solve();
  }
  void set_mode(CoarsePreconditioner mode) {if(mode!=mode_){mode_=mode;valid_=false;}}
  CoarsePreconditioner mode() const {return mode_;}
  void invalidate(){valid_=false;}
  void begin_solve(){stats_={};}
  const CoarsePreconditionerStats &stats() const {return stats_;}
  bool matches_structure(const AdaptiveCoarseField &a) const {
    bool same=valid_&&nodes_==a.nodes&&edges_==a.edges&&a.offset.size()>a.nodes&&a.neighbor.size()>=a.edges&&a.weight.size()>=a.edges&&a.boundary.size()>=a.nodes;
    for(uint32_t i=0;same&&i<=a.nodes;++i)same=a.offset[i]==cache_offset_[i];
    for(uint32_t i=0;same&&i<a.nodes;++i)same=a.boundary[i]==cache_boundary_[i];
    for(uint32_t e=0;same&&e<a.edges;++e)same=a.neighbor[e]==cache_neighbor_[e]&&a.weight[e]==cache_weight_[e];
    return same;
  }
  bool prepare(const AdaptiveCoarseField &a,const std::vector<double> &diagonal) {
    const auto begin=field_now_ns();
    auto done=[&](bool ok){stats_.setup_ns+=field_now_ns()-begin;stats_.ready=ok;return ok;};
    if(a.nodes>pivot_.size()||a.nodes+1>a.offset.size()||diagonal.size()<a.nodes||a.boundary.size()<a.nodes||
       a.offset[0]!=0||a.offset[a.nodes]!=a.edges||a.edges>a.neighbor.size()||a.edges>a.weight.size()||a.edges>lower_.size()) {
      valid_=false;return done(false);
    }
    if(mode_!=CoarsePreconditioner::Jacobi&&mode_!=CoarsePreconditioner::SymmetricGaussSeidel&&mode_!=CoarsePreconditioner::IC0){valid_=false;return done(false);}
    for(uint32_t i=0;i<a.nodes;++i)if(a.offset[i]>a.offset[i+1]||a.offset[i+1]>a.edges){valid_=false;return done(false);}
    bool same=valid_&&nodes_==a.nodes&&edges_==a.edges;
    for(uint32_t i=0;same&&i<=a.nodes;++i)same=a.offset[i]==cache_offset_[i];
    for(uint32_t i=0;same&&i<a.nodes;++i)same=a.boundary[i]==cache_boundary_[i]&&diagonal[i]==cache_diagonal_[i];
    for(uint32_t e=0;same&&e<a.edges;++e)same=a.neighbor[e]==cache_neighbor_[e]&&a.weight[e]==cache_weight_[e];
    if(same){stats_.setup_reused=true;return done(true);}
    valid_=false;nodes_=a.nodes;edges_=a.edges;
    if(mode_==CoarsePreconditioner::Jacobi) {
      for(uint32_t i=0;i<nodes_;++i) {
        if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior&&(!(diagonal[i]>0)||!std::isfinite(diagonal[i])))return pivot_failure(i,diagonal[i],begin);
        pivot_[i]=diagonal[i];
      }
      save_matrix(a,diagonal);valid_=true;return done(true);
    }
    std::fill_n(row_.begin(),nodes_+1,0);std::fill_n(upper_row_.begin(),nodes_+1,0);
    uint32_t count=0;
    for(uint32_t i=0;i<nodes_;++i) {
      if(a.offset[i]>a.offset[i+1]||a.offset[i+1]>a.edges)return done(false);
      if(FieldBoundary(a.boundary[i])==FieldBoundary::Interior) {
        if(!(diagonal[i]>0)||!std::isfinite(diagonal[i]))return pivot_failure(i,diagonal[i],begin);
        for(uint32_t e=a.offset[i];e<a.offset[i+1];++e) {
          const auto j=a.neighbor[e];const auto w=a.weight[e];
          if(j>=nodes_||j==i||!(w>0)||!std::isfinite(w))return done(false);
          if(FieldBoundary(a.boundary[j])!=FieldBoundary::Interior)continue;
          // A symmetric preconditioner is required by PCG. Compare each
          // reverse conductance instead of silently averaging a bad graph.
          double reverse=0.;
          for(uint32_t r=a.offset[j];r<a.offset[j+1];++r)if(a.neighbor[r]==i)reverse+=a.weight[r];
          if(!std::isfinite(reverse)||std::abs(reverse-w)>1e-12*std::max({1.,reverse,w}))return done(false);
          if(j<i)lower_[count++]={j,-w};
        }
      }
      row_[i+1]=count;
      std::sort(lower_.begin()+row_[i],lower_.begin()+row_[i+1],[](const Entry &x,const Entry &y){return x.column<y.column;});
      for(uint32_t k=row_[i]+1;k<row_[i+1];++k)if(lower_[k-1].column==lower_[k].column)return done(false);
    }
    for(uint32_t i=0;i<nodes_;++i)for(uint32_t k=row_[i];k<row_[i+1];++k)++upper_row_[lower_[k].column+1];
    for(uint32_t i=0;i<nodes_;++i)upper_row_[i+1]+=upper_row_[i];
    std::copy_n(upper_row_.begin(),nodes_,cursor_.begin());
    for(uint32_t i=0;i<nodes_;++i)for(uint32_t k=row_[i];k<row_[i+1];++k)upper_[cursor_[lower_[k].column]++]={i,k};
    for(uint32_t i=0;i<nodes_;++i) {
      pivot_[i]=diagonal[i];
      if(FieldBoundary(a.boundary[i])!=FieldBoundary::Interior||mode_!=CoarsePreconditioner::IC0)continue;
      for(uint32_t k=row_[i];k<row_[i+1];++k) {
        const uint32_t j=lower_[k].column;double sum=lower_[k].value;
        uint32_t x=row_[i],y=row_[j];
        while(x<k&&y<row_[j+1]) {
          const auto cx=lower_[x].column,cy=lower_[y].column;
          if(cx==cy){sum-=lower_[x].value*pivot_[cx]*lower_[y].value;++x;++y;}
          else if(cx<cy)++x;else ++y;
        }
        lower_[k].value=sum/pivot_[j];
        if(!std::isfinite(lower_[k].value))return pivot_failure(i,lower_[k].value,begin);
        pivot_[i]-=lower_[k].value*lower_[k].value*pivot_[j];
      }
      if(!(pivot_[i]>0)||!std::isfinite(pivot_[i]))return pivot_failure(i,pivot_[i],begin);
    }
    save_matrix(a,diagonal);valid_=true;return done(true);
  }
  bool apply(const std::vector<double> &r,std::vector<double> &z) {
    const auto begin=field_now_ns();++stats_.applications;
    auto done=[&](bool ok){stats_.apply_ns+=field_now_ns()-begin;return ok;};
    if(!valid_||r.size()<nodes_||z.size()<nodes_)return done(false);
    for(uint32_t i=0;i<nodes_;++i)z[i]=0.;
    if(mode_==CoarsePreconditioner::Jacobi) {
      for(uint32_t i=0;i<nodes_;++i)if(FieldBoundary(cache_boundary_[i])==FieldBoundary::Interior) {
        z[i]=r[i]/pivot_[i];if(!std::isfinite(z[i]))return done(false);
      }
      return done(true);
    }
    for(uint32_t i=0;i<nodes_;++i)if(FieldBoundary(cache_boundary_[i])==FieldBoundary::Interior) {
      double sum=r[i];for(uint32_t k=row_[i];k<row_[i+1];++k)sum-=lower_[k].value*work_[lower_[k].column];
      work_[i]=mode_==CoarsePreconditioner::IC0 ? sum : sum/pivot_[i];
      if(!std::isfinite(work_[i]))return done(false);
    }
    for(uint32_t i=nodes_;i-->0;)if(FieldBoundary(cache_boundary_[i])==FieldBoundary::Interior) {
      double sum=0;for(uint32_t k=upper_row_[i];k<upper_row_[i+1];++k)sum+=lower_[upper_[k].index].value*z[upper_[k].row];
      z[i]=mode_==CoarsePreconditioner::IC0 ? work_[i]/pivot_[i]-sum : work_[i]-sum/pivot_[i];
      if(!std::isfinite(z[i]))return done(false);
    }
    return done(true);
  }
 private:
  struct Entry {uint32_t column=0;double value=0.;};
  struct Upper {uint32_t row=0,index=0;};
  bool pivot_failure(uint32_t node,double pivot,int64_t begin) {
    stats_.failure_node=node;stats_.failure_pivot=pivot;stats_.setup_ns+=field_now_ns()-begin;stats_.ready=false;return false;
  }
  void save_matrix(const AdaptiveCoarseField &a,const std::vector<double> &diagonal) {
    std::copy_n(a.offset.begin(),nodes_+1,cache_offset_.begin());std::copy_n(a.neighbor.begin(),edges_,cache_neighbor_.begin());
    std::copy_n(a.weight.begin(),edges_,cache_weight_.begin());std::copy_n(diagonal.begin(),nodes_,cache_diagonal_.begin());
    std::copy_n(a.boundary.begin(),nodes_,cache_boundary_.begin());
  }
  CoarsePreconditioner mode_=static_cast<CoarsePreconditioner>(EGO4P0_COARSE_PC_MODE);
  CoarsePreconditionerStats stats_;
  std::vector<uint32_t> row_,upper_row_,cursor_,cache_offset_,cache_neighbor_;
  std::vector<Entry> lower_;std::vector<Upper> upper_;
  std::vector<double> pivot_,work_,cache_diagonal_,cache_weight_;
  std::vector<uint8_t> cache_boundary_;
  uint32_t nodes_=0,edges_=0;bool valid_=false;
};
} // namespace ego4p0
