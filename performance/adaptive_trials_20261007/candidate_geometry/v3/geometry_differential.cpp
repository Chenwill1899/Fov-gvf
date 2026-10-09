#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <vector>
using namespace pc_gvf::depth_angular;
#include "geometry_functions.inc"
struct Counts {std::uint64_t roots=0,nodes=0,queries=0,mismatches=0,pruned=0,fallback=0;};
int main(){
    std::mt19937_64 rng(441207);std::uniform_real_distribution<double> unit(-1.,1.),positive(0.,1.);
    Counts c;
    auto exercise=[&](const std::vector<CertifiedTube>& tubes,const Eigen::Vector3d& root,double radius,int budget){
        ++c.roots;
        std::vector<RecursiveTubeBounds> bounds;bounds.reserve(tubes.size());
        for(auto it=tubes.rbegin();it!=tubes.rend();++it)bounds.push_back(recursiveTubeBounds(*it,root,radius));
        std::vector<const RecursiveTubeBounds*> all;all.reserve(bounds.size());
        for(auto& b:bounds){all.push_back(&b);c.fallback+=!b.filterable;}
        std::array<std::vector<const RecursiveTubeBounds*>,8> levels;
        std::function<void(const Eigen::Vector3d&,double,int,const std::vector<const RecursiveTubeBounds*>*)> visit;
        visit=[&](const Eigen::Vector3d& p,double half,int level,const std::vector<const RecursiveTubeBounds*>* parent){
            if(--budget<0)return;
            ++c.nodes;const auto* selected=parent;
            if(parent&&parent->size()>8){
                const Eigen::Vector3d lo=p-Eigen::Vector3d::Constant(half),hi=p+Eigen::Vector3d::Constant(half);
                if(lo.allFinite()&&hi.allFinite()){
                    auto& local=levels[level];local.clear();local.reserve(parent->size());
                    for(const auto* t:*parent)if(!tubeBoundsDisjointCube(*t,lo,hi))local.push_back(t);
                    c.pruned+=parent->size()-local.size();
                    if(local.size()<parent->size())selected=&local;
                }
            }
            for(double qr:{0.,std::sqrt(3.)*half}){
                // Match the first accepting tube, not merely the union boolean:
                // source-relative precedence survives every filter and sibling.
                const CertifiedTube *before=nullptr,*after=nullptr;
                for(auto it=tubes.rbegin();it!=tubes.rend();++it)if(tubeContains(*it,p,qr)){before=&*it;break;}
                if(selected){for(const auto* t:*selected)if(tubeContains(*t->tube,p,qr)){after=t->tube;break;}}
                else for(auto it=tubes.rbegin();it!=tubes.rend();++it)if(tubeContains(*it,p,qr)){after=&*it;break;}
                ++c.queries;if(before!=after){++c.mismatches;if(c.mismatches<5)std::cerr<<"mismatch root "<<c.roots<<" level "<<level<<" p "<<p.transpose()<<" half "<<half<<'\n';}
            }
            if(level>=7)return;
            const double h=.5*half;
            for(int z:{-1,1})for(int y:{-1,1})for(int x:{-1,1})visit(p+h*Eigen::Vector3d(x,y,z),h,level+1,selected);
        };
        visit(root,radius,0,tubes.size()>8?&all:nullptr);
    };
    for(double magnitude:{0.,1e-12,1.,1e3,1e6,1e12,1e100,1e300})for(int sample=0;sample<160;++sample){
        const Eigen::Vector3d root=Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*magnitude;
        const double radius=sample%17?positive(rng):0.;
        const int count=sample%4==0?8:sample%4==1?9:sample%4==2?32:256;
        std::vector<CertifiedTube> tubes;
        for(int t=0;t<count;++t){
            Eigen::Vector3d a=root+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(t%3?3.:1e6);
            Eigen::Vector3d b=a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(t%7?.5:1e-12);
            tubes.push_back({a,b,positive(rng)});
        }
        // Full deep prefixes, deliberately visiting siblings after depth-owned
        // scratch storage is reused. This is not a performance benchmark.
        exercise(tubes,root,radius,1024);
    }
    // Capsule walls at, below and above each dyadic descendant boundary; large
    // coordinate cancellation, degenerate tubes and invalid evidence retained.
    const double nan=std::numeric_limits<double>::quiet_NaN(),inf=std::numeric_limits<double>::infinity();
    for(double origin:{0.,1.,1e6,1e12,1e100,1e300})for(double radius:{0.,1e-12,.58,1.,1e100}){
        Eigen::Vector3d root(origin,-origin,origin);std::vector<CertifiedTube> tubes;
        for(int axis=0;axis<3;++axis)for(double side:{-1.,1.})for(double nudge:{-1.,0.,1.}){
            auto p=root;p[axis]+=side*radius;
            if(nudge!=0.)p[axis]=std::nextafter(p[axis],nudge*inf);
            tubes.push_back({p,p,radius});
        }
        tubes.push_back({Eigen::Vector3d(nan,0.,0.),root,.58});
        tubes.push_back({root,Eigen::Vector3d(inf,0.,0.),.58});
        tubes.push_back({root,root,nan});tubes.push_back({root,root,inf});tubes.push_back({root,root,-1.});
        exercise(tubes,root,radius,1024);
    }
    const double special[]={0.,-1.,nan,inf,-inf,std::numeric_limits<double>::max()};
    for(double a:special)for(double b:special)for(double tr:special){
        std::vector<CertifiedTube> tubes(9,CertifiedTube{Eigen::Vector3d(a,0.,0.),Eigen::Vector3d(b,0.,0.),tr});
        exercise(tubes,Eigen::Vector3d(1e300,0.,0.),.58,64);
    }
    std::cout<<"roots,nodes,queries,mismatches,pruned_candidates,unfilterable_descriptors\n"<<c.roots<<','<<c.nodes<<','<<c.queries<<','<<c.mismatches<<','<<c.pruned<<','<<c.fallback<<'\n';
    return c.mismatches?1:0;
}
