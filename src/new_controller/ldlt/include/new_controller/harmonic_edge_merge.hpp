#pragma once
#include "new_controller/harmonic_types.hpp"
namespace ego4p0 {
// Stable in-place row ordering and coalescing. Summing parallel conductances
// preserves the FV operator, including diagonals and Dirichlet RHS.
inline void collapse_parallel_faces(AdaptiveCoarseField &a) {
  uint32_t write=0;
  for(uint32_t i=0;i<a.nodes;++i) {
    const uint32_t begin=a.offset[i],end=a.offset[i+1];
    for(uint32_t k=begin+1;k<end;++k) {
      const auto neighbor=a.neighbor[k];const double weight=a.weight[k];uint32_t j=k;
      while(j>begin&&a.neighbor[j-1]>neighbor) {a.neighbor[j]=a.neighbor[j-1];a.weight[j]=a.weight[j-1];--j;}
      a.neighbor[j]=neighbor;a.weight[j]=weight;
    }
    a.offset[i]=write;const uint32_t row=write;
    for(uint32_t k=begin;k<end;++k) {
      const auto neighbor=a.neighbor[k];const double weight=a.weight[k];
      if(write>row&&a.neighbor[write-1]==neighbor)a.weight[write-1]+=weight;
      else {a.neighbor[write]=neighbor;a.weight[write]=weight;++write;}
    }
  }
  a.offset[a.nodes]=write;a.edges=write;
}
} // namespace ego4p0
