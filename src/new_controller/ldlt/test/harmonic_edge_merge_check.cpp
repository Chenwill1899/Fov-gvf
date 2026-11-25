#include "new_controller/harmonic_adaptive.hpp"
#include "new_controller/harmonic_edge_merge.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <map>
#include <new>
#include <numeric>

using namespace ego4p0;
static std::atomic<bool> counting{false};
static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n){if(counting)++allocations;if(auto *p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void *p)noexcept{std::free(p);}
void operator delete(void *p,std::size_t)noexcept{std::free(p);}
void *operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void *p)noexcept{::operator delete(p);}
void operator delete[](void *p,std::size_t)noexcept{::operator delete(p);}
static void require(bool ok,const char *why){if(!ok)throw std::runtime_error(why);}
static bool close(double a,double b){return std::abs(a-b)<=2e-12*std::max({1.,std::abs(a),std::abs(b)});}
using Key=std::pair<uint32_t,uint32_t>;
using Edges=std::map<Key,double>;

static Edges directed_edges(const AdaptiveCoarseField &a) {
  Edges out;
  for(uint32_t i=0;i<a.nodes;++i)for(uint32_t e=a.offset[i];e<a.offset[i+1];++e)out[{i,a.neighbor[e]}]+=a.weight[e];
  return out;
}
static bool agrees(const AdaptiveCoarseField &a,const Edges &expected) {
  if(a.offset[0]||a.offset[a.nodes]!=a.edges||a.edges!=expected.size())return false;
  auto wanted=expected.begin();
  for(uint32_t i=0;i<a.nodes;++i) {
    if(a.offset[i]>a.offset[i+1])return false;
    for(uint32_t e=a.offset[i];e<a.offset[i+1];++e) {
      if(a.neighbor[e]>=a.nodes||a.neighbor[e]==i||!(a.weight[e]>0)||!std::isfinite(a.weight[e])||(e>a.offset[i]&&a.neighbor[e-1]>=a.neighbor[e]))return false;
      if(wanted==expected.end()||wanted->first!=Key{i,a.neighbor[e]}||!close(wanted->second,a.weight[e]))return false;
      ++wanted;
    }
  }
  return wanted==expected.end();
}
static std::vector<double> action(const AdaptiveCoarseField &a,const std::vector<double> &x,bool dirichlet) {
  std::vector<double> out(a.nodes);
  for(uint32_t i=0;i<a.nodes;++i) {
    if(dirichlet&&FieldBoundary(a.boundary[i])!=FieldBoundary::Interior)continue;
    for(uint32_t e=a.offset[i];e<a.offset[i+1];++e) {
      const auto j=a.neighbor[e];
      out[i]+=a.weight[e]*x[i];
      if(!dirichlet||FieldBoundary(a.boundary[j])==FieldBoundary::Interior)out[i]-=a.weight[e]*x[j];
    }
  }
  return out;
}
template<class T> static void keep_storage(const std::vector<T> &before,const std::vector<T> &after,const T *pointer) {
  require(before.size()==after.size()&&before.capacity()==after.capacity()&&after.data()==pointer,"merge resized or reallocated preallocated storage");
}

static void validate_merge(AdaptiveCoarseField &a,const Edges &expected,const char *label,bool require_reduction) {
  const auto before=a;const auto original_edges=a.edges;
  std::vector<double> x(a.nodes);for(uint32_t i=0;i<a.nodes;++i)x[i]=std::sin(.37*i)+.003*i;
  const auto full_action=action(a,x,false),interior_action=action(a,x,true);
  const auto *offset=a.offset.data(),*neighbor=a.neighbor.data();const auto *weight=a.weight.data();
  const auto *owner=a.owner.data(),*representative=a.representative.data();const auto *phi=a.phi.data();
  const auto *boundary=a.boundary.data(),*refined=a.refined.data();
  allocations=0;counting=true;collapse_parallel_faces(a);counting=false;
  require(allocations==0,"edge collapse dynamically allocated memory");
  keep_storage(before.offset,a.offset,offset);keep_storage(before.neighbor,a.neighbor,neighbor);keep_storage(before.weight,a.weight,weight);
  keep_storage(before.owner,a.owner,owner);keep_storage(before.representative,a.representative,representative);keep_storage(before.phi,a.phi,phi);keep_storage(before.boundary,a.boundary,boundary);keep_storage(before.refined,a.refined,refined);
  require(a.nodes==before.nodes&&a.refined_nodes==before.refined_nodes&&a.owner==before.owner&&a.representative==before.representative&&a.boundary==before.boundary&&a.refined==before.refined&&a.phi==before.phi,"merge changed ownership, node labels, potential or node metadata");
  require(agrees(a,expected),"coalesced CSR differs from independent edge sums, is not sorted, or has duplicate/self edges");
  if(require_reduction)require(a.edges<original_edges,"fixture contained no parallel faces to merge");
  for(const auto &e:expected){const auto reverse=expected.find({e.first.second,e.first.first});require(reverse!=expected.end()&&close(e.second,reverse->second),"coalesced CSR is not symmetric");}
  const auto full_after=action(a,x,false),interior_after=action(a,x,true);
  for(uint32_t i=0;i<a.nodes;++i){require(close(full_after[i],full_action[i]),"full graph Laplacian action changed");require(close(interior_after[i],interior_action[i]),"Dirichlet-eliminated interior operator action changed");}
  // Compare the independent full-map action, so a malformed input/build cannot
  // silently make both sides of the before/after comparison agree.
  std::vector<double> independent(a.nodes);
  for(const auto &e:expected)independent[e.first.first]+=e.second*(x[e.first.first]-x[e.first.second]);
  for(uint32_t i=0;i<a.nodes;++i)require(close(full_after[i],independent[i]),"operator action does not match independently accumulated fine faces");
  const auto once=a;
  allocations=0;counting=true;collapse_parallel_faces(a);counting=false;
  require(allocations==0&&a.edges==once.edges&&a.offset==once.offset&&a.neighbor==once.neighbor&&a.weight==once.weight,"second merge was not allocation-free and exactly idempotent");
  // The validator must actually reject dropped or incorrectly merged weights.
  if(a.edges){a.weight[0]*=1.01;require(!agrees(a,expected),"independent validator accepted a deliberately wrong weight");a.weight[0]=once.weight[0];}
  std::cout<<label<<": nodes="<<a.nodes<<" edges="<<original_edges<<" -> "<<a.edges<<" refined="<<a.refined_nodes<<"\n";
}

static void manual_csr() {
  AdaptiveCoarseField a;a.allocate(6);a.nodes=4;a.edges=12;a.refined_nodes=2;
  a.offset={0,4,8,12,12,12,12};
  const uint32_t neighbor[]={2,1,2,1,2,0,0,2,1,0,1,0};
  const double weight[]={.25,1.25,.75,.5,2.5,.5,1.25,.5,.5,.75,2.5,.25};
  std::copy_n(neighbor,12,a.neighbor.begin());std::copy_n(weight,12,a.weight.begin());
  for(uint32_t i=0;i<a.owner.size();++i){a.owner[i]=i%4;a.representative[i]=i;a.phi[i]=i*.1;a.boundary[i]=uint8_t(i==0?FieldBoundary::Low:i==2?FieldBoundary::Exit:FieldBoundary::Interior);a.refined[i]=i%2;}
  const auto expected=directed_edges(a);
  validate_merge(a,expected,"unsorted fractional multigraph with empty final row",true);
  AdaptiveCoarseField empty;empty.allocate(1);validate_merge(empty,{},"zero nodes",false);
  empty.nodes=1;empty.offset[0]=empty.offset[1]=0;validate_merge(empty,{},"one isolated node",false);
}

static void spherical_fixture(bool seam_polar) {
  SphericalStencil fine,coarse;fine.configure(1.);coarse.configure(3.);
  const auto na=fine.layout.azimuth_bins,nc=coarse.layout.azimuth_bins;
  std::vector<uint8_t> component(fine.nodes.size());
  for(uint32_t i=0;i<component.size();++i) {
    const double alpha=-180+(i%na+.5),beta=-90+(i/na+.5);
    component[i]=seam_polar ? std::abs(alpha)>=159&&beta>=69&&beta<89 : alpha>=-33&&alpha<33&&beta>=-16&&beta<17;
    // Unobserved holes and cut coarse cells must retain their fine children.
    if(seam_polar ? (std::abs(alpha)>172&&std::abs(alpha)<175&&beta<77) : (std::abs(alpha)<1&&beta<8))component[i]=0;
  }
  FieldLayer regular;regular.allocate(coarse.layout);
  for(uint32_t c=0;c<coarse.nodes.size();++c) {
    bool whole=true;const auto x=c%nc*3,y=c/nc*3;
    for(uint32_t dy=0;dy<3;++dy)for(uint32_t dx=0;dx<3;++dx)whole&=component[(y+dy)*na+x+dx];
    regular.state[c]=uint8_t(whole?CandidateStatus::Available:CandidateStatus::Unknown);
  }
  auto point=[](double alpha,double beta){return sphere_point(alpha*direction_pi/180,beta*direction_pi/180);};
  const auto low=fine.cell(seam_polar?point(166.5,81.5):point(-22.5,4.5));
  const auto target=fine.cell(seam_polar?point(-166.5,81.5):point(22.5,7.5));
  require(component[low]&&component[target],"spherical test anchor outside fixture");
  AdaptiveCoarseSolver solver;solver.configure(fine,coarse);AdaptiveCoarseField a;a.allocate(fine.nodes.size());
  solver.build(component,regular,low,target,3*direction_pi/180,a);
  require(a.refined_nodes>0&&a.nodes>a.refined_nodes,"spherical fixture must include aggregate and mixed child nodes");
  Edges expected;
  uint32_t directed_faces=0;
  for(uint32_t i=0;i<component.size();++i) {
    require(component[i]?(a.owner[i]<a.nodes):(a.owner[i]==UINT32_MAX),"ownership lost unknown cells or included unavailable cells");
    if(!component[i])continue;
    for(uint32_t k=0;k<4;++k) {
      const auto j=fine.nodes[i].neighbor[k];
      if(j==UINT32_MAX||!component[j]||a.owner[i]==a.owner[j])continue;
      expected[{a.owner[i],a.owner[j]}]+=fine.nodes[i].weight[k];++directed_faces;
    }
  }
  require(directed_faces>expected.size(),"spherical fixture lacks parallel fine faces");
  const auto built=directed_edges(a);
  require(built.size()==expected.size(),"actual adaptive build lost or added directed adjacency");
  for(const auto &e:expected){const auto found=built.find(e.first);require(found!=built.end()&&close(found->second,e.second),"actual adaptive build differs from independently accumulated fine faces");}
  // Recreate raw CSR from independent fine-face enumeration. This deliberately
  // avoids depending on whether AdaptiveCoarseSolver::build already coalesces.
  std::fill(a.offset.begin(),a.offset.end(),0);
  for(uint32_t i=0;i<component.size();++i)if(component[i])for(auto j:fine.nodes[i].neighbor)if(j!=UINT32_MAX&&component[j]&&a.owner[i]!=a.owner[j])++a.offset[a.owner[i]+1];
  for(uint32_t i=0;i<a.nodes;++i)a.offset[i+1]+=a.offset[i];
  auto cursor=a.offset;
  // Reverse the fine cell/face order to exercise arbitrary unsorted row order.
  for(uint32_t ii=component.size();ii>0;--ii) {
    const auto i=ii-1;if(!component[i])continue;
    for(uint32_t kk=4;kk>0;--kk) {
      const auto k=kk-1,j=fine.nodes[i].neighbor[k];
      if(j==UINT32_MAX||!component[j]||a.owner[i]==a.owner[j])continue;
      const auto e=cursor[a.owner[i]]++;a.neighbor[e]=a.owner[j];a.weight[e]=fine.nodes[i].weight[k];
    }
  }
  a.edges=a.offset[a.nodes];
  validate_merge(a,expected,seam_polar?"spherical periodic seam / polar mixed aggregation":"spherical obstacle / mixed aggregation",true);
}

int main(){try{manual_csr();spherical_fixture(false);spherical_fixture(true);std::cout<<"PASS: independent fine-face sums, sorted unique symmetric CSR, operator action, empty rows, storage/no-allocation, idempotence, negative control\n";return 0;}catch(const std::exception &e){counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
