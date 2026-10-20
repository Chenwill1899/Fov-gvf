#include "pc_gvf/proof_query_memo.hpp"
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>
using namespace pc_gvf::depth_angular;using V=Eigen::Vector3d;
void check(bool x,const char*m){if(!x){std::cerr<<m<<'\n';std::exit(1);}}
int main(){CapsuleProofMemo memo(2);CapsuleProofMemo::Revision revision{{0,0,0,0}};int calls=0;bool result=false;auto version=[&]{return revision;};auto query=[&]{++calls;return result;};
check(!memo.evaluate(V::Zero(),V::Zero(),.615,version,query),"initial unknown capsule");
check(!memo.evaluate(V::Zero(),V::Zero(),.615,version,query)&&calls==1,"unchanged repeated query recomputed");
++revision[2];result=true;check(memo.evaluate(V::Zero(),V::Zero(),.615,version,query)&&calls==2,"new positive evidence failed to invalidate negative accounting");
CapsuleProofMemo changed;int step=0;auto changing=[&]{++revision[0];return ++step>1;};
check(!changed.evaluate(V::Zero(),V::UnitX(),.6,version,changing)&&changed.size()==0,"query that grows positive proofs cached its own stale negative");
check(changed.evaluate(V::Zero(),V::UnitX(),.6,version,changing)&&step==2,"proof expansion failed retry");
result=false;memo.evaluate(V::Zero(),V::UnitY(),.6,version,query);memo.evaluate(V::Zero(),V::UnitZ(),.6,version,query);check(memo.size()==2,"capacity exceeded");
const int before=calls;memo.evaluate(V::Zero(),V::UnitZ(),.6,version,query);check(calls==before+1,"over-capacity query was skipped");
V invalid(NAN,0,0);memo.evaluate(invalid,V::Zero(),.6,version,query);check(memo.size()==2,"invalid geometry cached");
memo.clear();check(memo.size()==0,"observation scope did not clear capsule proofs");
result=false;check(!memo.evaluate(V::Zero(),V::Zero(),.615,version,query),"old positive survived new observation scope");
struct Segment {V a,b;double radius;};
std::mt19937 generator(240619);std::uniform_real_distribution<double> uniform(0.,1.);std::size_t covered=0;
for(int trial=0;trial<200;++trial) {
 std::vector<Segment> path;V at(300,45,19);
 for(int i=0;i<12;++i) {
  V next=at+4.*V(uniform(generator)-.5,uniform(generator)-.5,uniform(generator)-.5);
  if(trial%7==0&&i==11)next=path.front().a;
  path.push_back({at,next,.1+.6*uniform(generator)});at=next;
 }
 const auto bounds=capsulePathBounds(path);check(bounds.valid,"finite path enclosure invalid");
 for(const auto& s:path)for(double extra:{0.,.08})for(int sample=0;sample<10;++sample) {
  V offset(uniform(generator)-.5,uniform(generator)-.5,uniform(generator)-.5);offset.normalize();
  const double t=uniform(generator);
  const V inside=s.a+t*(s.b-s.a)+(s.radius+extra)*uniform(generator)*offset;
  check(bounds.mayTouch(inside,extra),"path enclosure culled a constructed capsule collision");++covered;
 }
 check(!bounds.mayTouch(V::Constant(1e6)),"path enclosure failed to cull a distant obstacle");
}
std::vector<Segment> stationary{{V::Zero(),V::Zero(),.615}};
check(capsulePathBounds(stationary).mayTouch(V(0,0,.615)),"stationary end cap omitted");
stationary.front().a.x()=NAN;
check(capsulePathBounds(stationary).mayTouch(V::Zero()),"invalid path enclosure culled an obstacle");
std::cout<<"piecewise path broad-phase: "<<covered<<" constructed collisions retained, including closed paths and margins\n";
std::cout<<"unchanged capsule reuse, proof expansion invalidation, mutation during query, capacity and invalid geometry: PASS\n";
}
