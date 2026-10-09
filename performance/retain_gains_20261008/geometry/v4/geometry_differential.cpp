#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <vector>
using namespace pc_gvf::depth_angular;
#include "geometry_functions.inc"
// Prevent the comparison caller from merging the two predicate invocations.
__attribute__((noinline)) bool original(const CertifiedTube& t,const Eigen::Vector3d& p,double r){return tubeContains(t,p,r);}
__attribute__((noinline)) bool cached(const PreparedTubeSegment& t,const Eigen::Vector3d& p,double r){return preparedTubeContains(t,p,r);}
int main(){
    std::mt19937_64 rng(441207);std::uniform_real_distribution<double> unit(-1.,1.),positive(0.,1.);
    std::uint64_t queries=0,mismatch=0,ordering_queries=0,ordering_mismatch=0,fallback=0;
    auto check=[&](const CertifiedTube& tube,const Eigen::Vector3d& p,double radius){
        const auto prepared=prepareTubeSegment(tube);fallback+=!prepared.cacheable;
        const bool a=original(tube,p,radius),b=cached(prepared,p,radius);++queries;
        if(a!=b){++mismatch;if(mismatch<5)std::cerr<<"mismatch "<<p.transpose()<<" radius "<<radius<<'\n';}
    };
    const double inf=std::numeric_limits<double>::infinity(),nan=std::numeric_limits<double>::quiet_NaN();
    for(double magnitude:{0.,1e-12,1.,1e3,1e6,1e12,1e100,1e300})for(int sample=0;sample<120000;++sample){
        const Eigen::Vector3d a=Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*magnitude;
        const Eigen::Vector3d b=a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(sample%7?.5:1e-10);
        const CertifiedTube tube{a,b,positive(rng)};
        const Eigen::Vector3d p=a+(b-a)*positive(rng)+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(sample%5?1.:1e-12);
        const double boundary=tube.radius-segmentDistance(p,a,b);
        for(double qr:{0.,tube.radius,positive(rng),boundary,std::nextafter(boundary,inf),std::nextafter(boundary,-inf)})check(tube,p,qr);
    }
    // Degenerate projection branch, dot-product cancellation, clamp endpoints,
    // overflow/NaN fallback, and radius/query values exactly at their limits.
    const double special[]={0.,-1.,1e-160,1e-10,std::nextafter(1e-10,0.),std::nextafter(1e-10,inf),1.,1e100,1e300,std::numeric_limits<double>::max(),nan,inf,-inf};
    for(double a:special)for(double b:special)for(double tr:special)for(double p0:{0.,1e-10,1.,1e300,inf,nan}){
        const CertifiedTube tube{Eigen::Vector3d(a,-a,0.),Eigen::Vector3d(b,-b,0.),tr};
        for(double qr:{0.,.58,tr})check(tube,Eigen::Vector3d(p0,0.,0.),qr);
    }
    for(int group=0;group<4000;++group){
        std::vector<CertifiedTube> tubes;std::vector<PreparedTubeSegment> prepared;
        for(int i=0;i<32;++i){const Eigen::Vector3d a(unit(rng),unit(rng),unit(rng));tubes.push_back({a,a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*.5,positive(rng)});}
        if(group%7==0)tubes[group%32].radius=nan;
        for(auto it=tubes.rbegin();it!=tubes.rend();++it)prepared.push_back(prepareTubeSegment(*it));
        for(int point=0;point<64;++point){
            const Eigen::Vector3d p(unit(rng),unit(rng),unit(rng));const double r=positive(rng);
            const CertifiedTube *before=nullptr,*after=nullptr;
            for(auto it=tubes.rbegin();it!=tubes.rend();++it)if(original(*it,p,r)){before=&*it;break;}
            for(const auto& item:prepared)if(cached(item,p,r)){after=item.tube;break;}
            ++ordering_queries;ordering_mismatch+=before!=after;
        }
    }
    std::cout<<"queries,mismatches,ordering_queries,ordering_mismatches,fallback_queries\n"<<queries<<','<<mismatch<<','<<ordering_queries<<','<<ordering_mismatch<<','<<fallback<<'\n';
    return mismatch||ordering_mismatch?1:0;
}
