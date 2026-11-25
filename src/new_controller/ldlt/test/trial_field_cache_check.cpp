#include "new_controller/trial_field_solver.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>
#include <stdexcept>

using namespace ego4p0;
static std::atomic<bool> counting{false};
static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n){if(counting)++allocations;if(auto *p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void *p)noexcept{std::free(p);}
void operator delete(void *p,std::size_t)noexcept{std::free(p);}
void *operator new[](std::size_t n){return ::operator new(n);}
void operator delete[](void *p)noexcept{::operator delete(p);}
void operator delete[](void *p,std::size_t)noexcept{::operator delete(p);}

static void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
static void equal_result(const FieldLayer &a,const FieldLayer &b){
  require(a.status==b.status&&a.boundary==b.boundary&&a.nodes==b.nodes,"cached/fresh layer metadata differ");
  require(a.phi.size()==b.phi.size(),"cached/fresh layer dimensions differ");
  for(size_t i=0;i<a.phi.size();++i)require(a.phi[i]==b.phi[i]||(std::isnan(a.phi[i])&&std::isnan(b.phi[i])),"cached/fresh potential differs");
  require(a.residual==b.residual,"cached/fresh true residual differs");
}
static SphericalStencil chain(){
  SphericalStencil s;s.nodes.resize(7);
  for(auto &n:s.nodes){n.neighbor.fill(UINT32_MAX);n.weight.fill(0);}
  for(uint32_t i=0;i<6;++i){
    if(i){s.nodes[i].neighbor[0]=i-1;s.nodes[i].weight[0]=1.;}
    if(i+1<6){s.nodes[i].neighbor[1]=i+1;s.nodes[i].weight[1]=1.;}
  }
  return s;
}
static FieldLayer layer(){
  FieldLayer f;f.phi.assign(7,.5);f.boundary.assign(7,uint8_t(FieldBoundary::Interior));
  f.boundary[0]=uint8_t(FieldBoundary::Low);f.boundary[5]=uint8_t(FieldBoundary::Exit);
  f.boundary[3]=uint8_t(FieldBoundary::Coupling);f.phi[3]=.4;
  f.boundary[6]=uint8_t(FieldBoundary::Outside);return f;
}
static AdaptiveCoarseField adaptive(const SphericalStencil &s,const FieldLayer &f){
  AdaptiveCoarseField a;a.allocate(7);a.nodes=6;a.owner={0,1,2,3,4,5,UINT32_MAX};
  for(uint32_t i=0;i<a.nodes;++i){a.representative[i]=i;a.boundary[i]=f.boundary[i];a.phi[i]=f.phi[i];a.offset[i]=a.edges;
    for(uint32_t k=0;k<4;++k)if(s.nodes[i].neighbor[k]!=UINT32_MAX){a.neighbor[a.edges]=s.nodes[i].neighbor[k];a.weight[a.edges++]=s.nodes[i].weight[k];}}
  a.offset[a.nodes]=a.edges;return a;
}
static void fine_case(TrialFieldSolver &solver,const SphericalStencil &s,FieldLayer &f,const HarmonicConfig &cfg,bool reused){
  auto reference=f;TrialFieldSolver fresh(SparseTrialMode::EigenLDLT);fresh.configure(static_cast<uint32_t>(s.nodes.size()));
  solver.solve_layer(s,f,cfg);fresh.solve_layer(s,reference,cfg);
  require(f.status==FieldStatus::Valid,"fine test solve failed");require(f.reused==reused,"fine exact operator cache decision wrong");equal_result(f,reference);
  if(reused)require(solver.stats().total_ns==0&&solver.stats().verify_ns==0,"fine potential hit called backend");
}
static void adaptive_case(TrialFieldSolver &solver,AdaptiveCoarseField &a,const HarmonicConfig &cfg,bool reused){
  auto reference=a;FieldLayer f,expected;TrialFieldSolver fresh(SparseTrialMode::EigenLDLT);fresh.configure(static_cast<uint32_t>(a.owner.size()));
  solver.solve_adaptive(a,f,cfg);fresh.solve_adaptive(reference,expected,cfg);
  require(f.status==FieldStatus::Valid&&expected.status==FieldStatus::Valid,"adaptive test solve failed");
  require(f.reused==reused,"adaptive exact operator cache decision wrong");
  require(f.residual==expected.residual&&f.nodes==expected.nodes,"adaptive cached/fresh residual or node count differs");
  for(uint32_t i=0;i<a.nodes;++i)require(a.phi[i]==reference.phi[i],"adaptive cached/fresh potential differs");
  if(reused)require(solver.stats().total_ns==0&&solver.stats().verify_ns==0,"adaptive potential hit called backend");
}
int main(){try{
  HarmonicConfig cfg;auto s=chain();auto f=layer();TrialFieldSolver fine(SparseTrialMode::EigenLDLT);fine.configure(7);
  fine_case(fine,s,f,cfg,false);
  require(std::abs(f.phi[1]-.4/3)<1e-14&&std::abs(f.phi[2]-.8/3)<1e-14&&std::abs(f.phi[4]-.7)<1e-14,"manufactured chain potential wrong");
  const auto unknowns=fine.stats().unknowns,nonzeros=fine.stats().nonzeros;
  const auto saved=f.phi;const double residual=f.residual;
  // Canonical fixed values and irrelevant initial guesses must still be
  // normalized/ignored before deciding a hit, exactly as on a fresh solve.
  f.phi[0]=55.;f.phi[5]=-55.;f.phi[1]=.9;f.phi[6]=123.;
  allocations=0;counting=true;fine.solve_layer(s,f,cfg);counting=false;
  require(f.status==FieldStatus::Valid&&f.reused&&allocations==0,"fine hit assembled/allocated or failed normalization");
  require(f.phi[0]==0.&&f.phi[5]==1.&&f.phi[1]==saved[1]&&std::isnan(f.phi[6])&&f.residual==residual,"fine hit lost canonical values or certified residual");
  require(fine.stats().unknowns==unknowns&&fine.stats().nonzeros==nonzeros,"fine hit lost system counts");
  auto equal_stencil=s;fine_case(fine,equal_stencil,f,cfg,true); // value identity, not address
  f.phi[3]=.6;fine_case(fine,s,f,cfg,false);require(fine.stats().numeric_reused,"new coupling RHS discarded equal matrix factor");
  auto changed=s;changed.nodes[1].weight[0]=2.; // Interior -> Low contribution
  fine_case(fine,changed,f,cfg,false);fine_case(fine,changed,f,cfg,true);
  changed.nodes[1].neighbor[2]=4;changed.nodes[1].weight[2]=.5;
  changed.nodes[4].neighbor[2]=1;changed.nodes[4].weight[2]=.5;
  fine_case(fine,changed,f,cfg,false);fine_case(fine,changed,f,cfg,true);
  cfg.residual_tolerance=1e-8;fine_case(fine,changed,f,cfg,false);fine_case(fine,changed,f,cfg,true);
  // These outgoing rows never participate in the grounded system/residual.
  changed.nodes[0].weight[0]=-7.;changed.nodes[6].neighbor[0]=123;changed.nodes[6].weight[0]=9.;
  fine_case(fine,changed,f,cfg,true);
  // Out-of-range Interior neighbors retain the existing assembler semantics:
  // the missing face is skipped, but its changed row cannot hit the old cache.
  changed.nodes[1].neighbor[0]=99;fine_case(fine,changed,f,cfg,false);
  auto invalid=changed;invalid.nodes[1].weight[1]=-1.;auto rejected=f;
  fine.solve_layer(invalid,rejected,cfg);require(rejected.status==FieldStatus::SolveFailed&&!rejected.reused,"invalid Interior weight hidden by fine cache");
  fine_case(fine,changed,f,cfg,false); // failure invalidates the potential cache
  rejected=f;rejected.phi[3]=std::numeric_limits<double>::quiet_NaN();fine.solve_layer(changed,rejected,cfg);
  require(rejected.status==FieldStatus::SolveFailed&&!rejected.reused,"nonfinite coupling value accepted");
  fine_case(fine,changed,f,cfg,false);
  rejected=f;rejected.boundary[2]=255;fine.solve_layer(changed,rejected,cfg);
  require(rejected.status==FieldStatus::SolveFailed&&!rejected.reused,"invalid fine boundary accepted");
  fine_case(fine,changed,f,cfg,false);
  auto bad_cfg=cfg;bad_cfg.residual_tolerance=0;rejected=f;fine.solve_layer(changed,rejected,bad_cfg);
  require(rejected.status==FieldStatus::SolveFailed&&!rejected.reused,"invalid tolerance accepted");
  fine_case(fine,changed,f,cfg,false);
  rejected=f;rejected.phi.pop_back();fine.solve_layer(changed,rejected,cfg);
  require(rejected.status==FieldStatus::SolveFailed&&!rejected.reused,"fine size mismatch accepted");
  fine_case(fine,changed,f,cfg,false);fine.invalidate();fine_case(fine,changed,f,cfg,false);
  auto fixed_only=layer();for(auto &bc:fixed_only.boundary)if(FieldBoundary(bc)==FieldBoundary::Interior)bc=uint8_t(FieldBoundary::Low);
  fine_case(fine,s,fixed_only,cfg,false);fine_case(fine,s,fixed_only,cfg,true);
  require(fine.stats().unknowns==0&&fine.stats().nonzeros==0,"fixed-only hit retained old grounded system counts");

  auto a=adaptive(s,layer());TrialFieldSolver coarse(SparseTrialMode::EigenLDLT);coarse.configure(7);
  adaptive_case(coarse,a,cfg,false);adaptive_case(coarse,a,cfg,true);
  FieldLayer coarse_hit;allocations=0;counting=true;coarse.solve_adaptive(a,coarse_hit,cfg);counting=false;
  require(coarse_hit.reused&&allocations==0,"adaptive hit assembled/allocated");
  a.phi[3]=.6;adaptive_case(coarse,a,cfg,false);require(coarse.stats().numeric_reused,"adaptive new RHS discarded equal matrix factor");
  a.owner[6]=0;adaptive_case(coarse,a,cfg,false);adaptive_case(coarse,a,cfg,true);
  a.representative[0]=6;adaptive_case(coarse,a,cfg,false);adaptive_case(coarse,a,cfg,true);
  // Change the fixed-side conductance, then add a symmetric active edge.
  a.weight[a.offset[1]]=2.;adaptive_case(coarse,a,cfg,false);
  auto graph=s;graph.nodes[1].neighbor[2]=4;graph.nodes[1].weight[2]=.5;graph.nodes[4].neighbor[2]=1;graph.nodes[4].weight[2]=.5;
  a=adaptive(graph,layer());adaptive_case(coarse,a,cfg,false);adaptive_case(coarse,a,cfg,true);
  // Edge order is part of the exact adaptive input even when CSR is equivalent.
  std::swap(a.neighbor[a.offset[1]],a.neighbor[a.offset[1]+1]);std::swap(a.weight[a.offset[1]],a.weight[a.offset[1]+1]);
  adaptive_case(coarse,a,cfg,false);
  cfg.residual_tolerance=1e-9;adaptive_case(coarse,a,cfg,false);adaptive_case(coarse,a,cfg,true);
  auto bad_a=a;bad_a.weight[0]=-1.;FieldLayer failed;coarse.solve_adaptive(bad_a,failed,cfg);
  require(failed.status==FieldStatus::SolveFailed&&!failed.reused,"invalid adaptive weight hidden by cache");
  adaptive_case(coarse,a,cfg,false);
  bad_a=a;bad_a.neighbor[0]=a.nodes;coarse.solve_adaptive(bad_a,failed,cfg);
  require(failed.status==FieldStatus::SolveFailed&&!failed.reused,"invalid adaptive neighbor accepted");
  adaptive_case(coarse,a,cfg,false);
  bad_a=a;bad_a.offset[2]=a.edges+1;coarse.solve_adaptive(bad_a,failed,cfg);
  require(failed.status==FieldStatus::SolveFailed&&!failed.reused,"invalid adaptive offsets accepted");
  adaptive_case(coarse,a,cfg,false);
  bad_a=a;bad_a.boundary[0]=uint8_t(FieldBoundary::Interior);coarse.solve_adaptive(bad_a,failed,cfg);
  require(failed.status==FieldStatus::SolveFailed&&!failed.reused,"adaptive missing Low accepted");
  adaptive_case(coarse,a,cfg,false);coarse.invalidate();adaptive_case(coarse,a,cfg,false);
  // A solver may be used with both adapters: identities must never leak across
  // representations, even when the grounded matrices happen to be identical.
  auto six=s;six.nodes.resize(6);auto six_layer=layer();six_layer.phi.resize(6);six_layer.boundary.resize(6);
  fine_case(coarse,six,six_layer,cfg,false);fine_case(coarse,six,six_layer,cfg,true);
  adaptive_case(coarse,a,cfg,false);
  std::cout<<"PASS: allocation-free exact cache, canonical BC/RHS, independent stencil identity, topology/weight/owner/representative/order/tolerance invalidation, invalid input rejection, fresh solve equivalence\n";
  return 0;
}catch(const std::exception &e){counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
