#pragma once
#include "new_controller/harmonic_types.hpp"
#include <algorithm>

namespace ego4p0 {
// Validate the graph and boundary labels used by the local-Neumann fine solve.
// No matrix is assembled, factored or solved here, and no numerical residual
// is claimed. Exact snapshots make reuse independent of a coarse phi cache.
class CoarseTopologyValidator {
 public:
  void configure(uint32_t maximum_nodes){
    maximum_=maximum_nodes;
    owner_.reserve(maximum_);representative_.reserve(maximum_);boundary_.reserve(maximum_);
    offset_.reserve(size_t(maximum_)+1);neighbor_.reserve(size_t(maximum_)*4);weight_.reserve(size_t(maximum_)*4);
    visited_.resize(maximum_);queue_.resize(maximum_);invalidate();
  }
  void invalidate(){valid_=false;error_="none";}
  const char *error()const{return error_;}

  bool validate(const AdaptiveCoarseField &a,const std::vector<uint8_t>&component,
                const std::vector<uint32_t>&active,FieldLayer &layer){
    error_="none";layer.status=FieldStatus::NoInput;layer.nodes=a.nodes;
    layer.iterations=0;layer.reused=layer.warm_started=layer.constant_boundary=false;
    layer.residual=std::numeric_limits<double>::quiet_NaN();
    layer.solve_ns=layer.solve_cpu_ns=layer.cache_ns=layer.assembly_ns=layer.iteration_ns=layer.residual_ns=layer.store_ns=0;
    auto fail=[&](const char *why){error_=why;valid_=false;layer.status=FieldStatus::SolveFailed;return false;};
    const uint32_t count=a.nodes;
    if(count<2||count>maximum_||a.owner.size()!=component.size()||component.size()>maximum_||
       a.representative.size()<count||a.boundary.size()<count||a.offset.size()<size_t(count)+1||
       a.neighbor.size()<a.edges||a.weight.size()<a.edges||a.offset[0]!=0||a.offset[count]!=a.edges)
      return fail("invalid_topology_dimensions");
    size_t active_count=0;
    for(size_t i=0;i<component.size();++i){
      if(component[i]>1)return fail("invalid_component_mask");
      if(component[i]){++active_count;if(a.owner[i]>=count)return fail("missing_active_owner");}
      else if(a.owner[i]!=UINT32_MAX)return fail("owner_outside_component");
    }
    if(active.size()!=active_count)return fail("active_component_size_mismatch");
    for(size_t i=0;i<active.size();++i)
      if(active[i]>=component.size()||!component[active[i]]||(i&&active[i]<=active[i-1]))
        return fail("invalid_active_component_list");

    const bool same=valid_&&owner_==a.owner&&representative_.size()==count&&
      std::equal(representative_.begin(),representative_.end(),a.representative.begin())&&
      std::equal(boundary_.begin(),boundary_.end(),a.boundary.begin())&&offset_.size()==size_t(count)+1&&
      std::equal(offset_.begin(),offset_.end(),a.offset.begin())&&neighbor_.size()==a.edges&&
      std::equal(neighbor_.begin(),neighbor_.end(),a.neighbor.begin())&&
      std::equal(weight_.begin(),weight_.end(),a.weight.begin());
    if(same){layer.status=FieldStatus::TopologyReady;layer.reused=true;return true;}

    bool low=false,exit=false;
    for(uint32_t i=0;i<count;++i){
      const auto representative=a.representative[i];
      if(representative>=a.owner.size()||!component[representative]||a.owner[representative]!=i)
        return fail("invalid_topology_representative");
      const auto label=FieldBoundary(a.boundary[i]);
      if(label!=FieldBoundary::Interior&&label!=FieldBoundary::Low&&label!=FieldBoundary::Exit)
        return fail("invalid_topology_boundary");
      low|=label==FieldBoundary::Low;exit|=label==FieldBoundary::Exit;
      if(a.offset[i]>a.offset[i+1]||a.offset[i+1]>a.edges)return fail("invalid_topology_offsets");
      double degree=0.;
      for(uint32_t e=a.offset[i];e<a.offset[i+1];++e){
        if(a.neighbor[e]>=count||a.neighbor[e]==i||
           (e>a.offset[i]&&a.neighbor[e]<=a.neighbor[e-1])||
           !(a.weight[e]>0.)||!std::isfinite(a.weight[e]))return fail("invalid_topology_edge");
        degree+=a.weight[e];
      }
      if(!(degree>0.)||!std::isfinite(degree))return fail("invalid_topology_degree");
    }
    if(!low||!exit)return fail("missing_topology_low_or_exit");
    // Preserve the FV symmetry tolerance used by the numerical backend.
    for(uint32_t i=0;i<count;++i)for(uint32_t e=a.offset[i];e<a.offset[i+1];++e){
      const auto j=a.neighbor[e];
      const auto first=a.neighbor.begin()+a.offset[j],last=a.neighbor.begin()+a.offset[j+1];
      const auto reverse=std::lower_bound(first,last,i);
      if(reverse==last||*reverse!=i||
         std::abs(a.weight[e]-a.weight[size_t(reverse-a.neighbor.begin())])>1e-12*std::max(1.,std::abs(a.weight[e])))
        return fail("asymmetric_topology_edge");
    }
    std::fill_n(visited_.begin(),count,0);uint32_t head=0,tail=1;queue_[0]=0;visited_[0]=1;
    while(head<tail){const auto i=queue_[head++];for(uint32_t e=a.offset[i];e<a.offset[i+1];++e){
      const auto j=a.neighbor[e];if(!visited_[j]){visited_[j]=1;queue_[tail++]=j;}
    }}
    if(tail!=count)return fail("disconnected_topology");
    owner_=a.owner;representative_.assign(a.representative.begin(),a.representative.begin()+count);
    boundary_.assign(a.boundary.begin(),a.boundary.begin()+count);
    offset_.assign(a.offset.begin(),a.offset.begin()+count+1);
    neighbor_.assign(a.neighbor.begin(),a.neighbor.begin()+a.edges);weight_.assign(a.weight.begin(),a.weight.begin()+a.edges);
    valid_=true;layer.status=FieldStatus::TopologyReady;return true;
  }
 private:
  uint32_t maximum_=0;
  std::vector<uint32_t>owner_,representative_,offset_,neighbor_,queue_;
  std::vector<uint8_t>boundary_,visited_;
  std::vector<double>weight_;
  bool valid_=false;
  const char *error_="none";
};
} // namespace ego4p0
