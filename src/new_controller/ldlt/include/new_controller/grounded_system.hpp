#pragma once
#include "new_controller/harmonic_solver.hpp"
#include <numeric>
namespace ego4p0 {
// Fixed geometry matrix assembly, with the existing missing-face Neumann rule.
// Low/Exit/Coupling values enter only the eliminated Dirichlet RHS.
struct GroundedSystem {
  std::vector<int> ptr,col,index;
  std::vector<uint32_t> node;
  std::vector<double> val,rhs,x,diagonal;
  bool valid=false;std::string error;
  void reserve(uint32_t n) {ptr.reserve(n+1);col.reserve(5*n);val.reserve(5*n);index.reserve(n);node.reserve(n);rhs.reserve(n);x.reserve(n);diagonal.reserve(n);}
  template<class Boundary,class Value,class Edges>
  bool assemble(uint32_t count,Boundary boundary,Value value,Edges edges) {
    valid=false;error.clear();index.assign(count,-1);node.clear();ptr.clear();col.clear();val.clear();
    for(uint32_t i=0;i<count;++i)if(FieldBoundary(boundary(i))==FieldBoundary::Interior){index[i]=static_cast<int>(node.size());node.push_back(i);}
    rhs.assign(node.size(),0);diagonal.assign(node.size(),0);x.assign(node.size(),.5);ptr.push_back(0);
    std::vector<std::pair<int,double>> row;row.reserve(16);bool fixed=false;
    for(uint32_t k=0;k<node.size();++k) {
      const auto i=node[k];row.clear();double d=0,b=0;
      edges(i,[&](uint32_t j,double w){
        if(j==UINT32_MAX||j>=count||FieldBoundary(boundary(j))==FieldBoundary::Outside)return;
        if(!(w>0)||!std::isfinite(w)){error="invalid FV conductance";return;}
        d+=w;
        if(FieldBoundary(boundary(j))==FieldBoundary::Interior)row.push_back({index[j],-w});
        else {const double v=value(j);if(!std::isfinite(v)){error="nonfinite Dirichlet value";return;}b+=w*v;fixed=true;}
      });
      if(!(d>0)||!std::isfinite(d)||!std::isfinite(b)){error="invalid grounded row";return false;}
      row.push_back({static_cast<int>(k),d});std::sort(row.begin(),row.end(),[](const auto&a,const auto&b){return a.first<b.first;});
      const auto begin=col.size();
      for(const auto &e:row){if(col.size()>begin&&col.back()==e.first)val.back()+=e.second;else{col.push_back(e.first);val.push_back(e.second);}}
      ptr.push_back(static_cast<int>(col.size()));diagonal[k]=d;rhs[k]=b;
    }
    if(!error.empty())return false;
    if(node.empty()||!fixed){error="no grounded unknowns/Dirichlet anchor";return false;}
    valid=true;return true;
  }
  bool adaptive(const AdaptiveCoarseField &a) {
    return assemble(a.nodes,[&](uint32_t i){return a.boundary[i];},[&](uint32_t i){return a.phi[i];},
      [&](uint32_t i,auto add){for(uint32_t e=a.offset[i];e<a.offset[i+1];++e)add(a.neighbor[e],a.weight[e]);});
  }
  bool layer(const SphericalStencil &s,const FieldLayer &l) {
    return assemble(static_cast<uint32_t>(l.phi.size()),[&](uint32_t i){return l.boundary[i];},[&](uint32_t i){return l.phi[i];},
      [&](uint32_t i,auto add){for(uint32_t k=0;k<4;++k)add(s.nodes[i].neighbor[k],s.nodes[i].weight[k]);});
  }
};
}
