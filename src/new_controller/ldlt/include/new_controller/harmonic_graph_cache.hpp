#pragma once
#include "new_controller/harmonic_types.hpp"
#include <algorithm>

namespace ego4p0 {
// Only the planner may commit a graph, immediately after topology validation.
// The private snapshot is authoritative; caller-owned output buffers are never
// evidence that a graph is unchanged or that it previously passed validation.
class ValidatedCoarseGraphCache {
  friend class HarmonicPlanner;
  void configure(uint32_t maximum_nodes,uint32_t coarse_nodes){
    component_.reserve(maximum_nodes);active_.reserve(maximum_nodes);state_.reserve(coarse_nodes);
    owner_.reserve(maximum_nodes);representative_.reserve(maximum_nodes);boundary_.reserve(maximum_nodes);
    refined_.reserve(maximum_nodes);offset_.reserve(size_t(maximum_nodes)+1);
    neighbor_.reserve(size_t(maximum_nodes)*4);weight_.reserve(size_t(maximum_nodes)*4);
    invalidate();
  }
  void invalidate(){valid_=false;}
  bool restore(const std::vector<uint8_t>&component,const std::vector<uint32_t>&active,
               const std::vector<uint8_t>&state,uint32_t low,uint32_t target,double cap,
               AdaptiveCoarseField&a)const{
    if(!valid_||low!=low_||target!=target_||cap!=cap_||component!=component_||
       active!=active_||state!=state_)return false;
    // Keep the full allocated sizes: a later miss writes the maximum graph
    // with operator[]. Only the snapshot and copies use compact prefixes.
    std::copy(owner_.begin(),owner_.end(),a.owner.begin());
    std::copy(representative_.begin(),representative_.end(),a.representative.begin());
    std::copy(boundary_.begin(),boundary_.end(),a.boundary.begin());
    std::copy(refined_.begin(),refined_.end(),a.refined.begin());
    std::copy(offset_.begin(),offset_.end(),a.offset.begin());
    std::copy(neighbor_.begin(),neighbor_.end(),a.neighbor.begin());
    std::copy(weight_.begin(),weight_.end(),a.weight.begin());
    a.nodes=static_cast<uint32_t>(representative_.size());
    a.edges=static_cast<uint32_t>(neighbor_.size());
    a.refined_nodes=refined_nodes_;a.raw_edges=raw_edges_;a.edge_merge_ns=0;
    std::fill_n(a.phi.begin(),a.nodes,std::numeric_limits<double>::quiet_NaN());
    return true;
  }
  void remember_validated(const std::vector<uint8_t>&component,const std::vector<uint32_t>&active,
                          const std::vector<uint8_t>&state,uint32_t low,uint32_t target,double cap,
                          const AdaptiveCoarseField&a){
    valid_=false;
    component_=component;active_=active;state_=state;low_=low;target_=target;cap_=cap;
    owner_=a.owner;
    representative_.assign(a.representative.begin(),a.representative.begin()+a.nodes);
    boundary_.assign(a.boundary.begin(),a.boundary.begin()+a.nodes);
    refined_.assign(a.refined.begin(),a.refined.begin()+a.nodes);
    offset_.assign(a.offset.begin(),a.offset.begin()+a.nodes+1);
    neighbor_.assign(a.neighbor.begin(),a.neighbor.begin()+a.edges);
    weight_.assign(a.weight.begin(),a.weight.begin()+a.edges);
    refined_nodes_=a.refined_nodes;raw_edges_=a.raw_edges;
    valid_=true;
  }
  std::vector<uint8_t>component_,state_,boundary_,refined_;
  std::vector<uint32_t>active_,owner_,representative_,offset_,neighbor_;
  std::vector<double>weight_;
  uint32_t low_=0,target_=0,refined_nodes_=0,raw_edges_=0;
  double cap_=0.;
  bool valid_=false;
};
} // namespace ego4p0
