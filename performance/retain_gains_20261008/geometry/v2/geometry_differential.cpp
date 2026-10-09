#include "pc_gvf/paper_guidance.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <random>
#include <cstdint>
using namespace pc_gvf::depth_angular;
#include "geometry_functions.inc"
int main(){
    std::mt19937_64 rng(441207);std::uniform_real_distribution<double> unit(-1.,1.),positive(0.,1.);
    std::uint64_t queries=0,mismatch=0,pruned=0;
    for(double magnitude:{0.,1e-12,1.,1e3,1e6,1e12,1e100,1e300})for(int sample=0;sample<40000;++sample){
        const Eigen::Vector3d center=Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*magnitude;
        const double radius=positive(rng),tube_radius=positive(rng);
        const Eigen::Vector3d a=center+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(sample%4?3.:1e6);
        const Eigen::Vector3d b=a+Eigen::Vector3d(unit(rng),unit(rng),unit(rng))*(sample%7?.5:1e-12);
        const CertifiedTube tube{a,b,tube_radius};bool kept=retain(tube,center,radius);pruned+=!kept;
        Eigen::Vector3d p=center;double half=radius;
        for(int level=0;level<=7;++level){
            for(double qr:{0.,std::sqrt(3.)*half}){bool original=frozen::tubeContains(tube,p,qr),filtered=kept&&original;++queries;mismatch+=original!=filtered;}
            half*=.5;for(int axis=0;axis<3;++axis)p[axis]+=(rng()%2?1.:-1.)*half;
        }
        // The root-cube extreme is more demanding than a finite-depth center.
        for(int axis=0;axis<3;++axis)for(double side:{-1.,1.}){
            p=center;p[axis]+=side*radius;const bool original=frozen::tubeContains(tube,p,0.);++queries;mismatch+=original!=(kept&&original);
        }
    }
    const double special[]={0.,-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::max()};
    for(double a:special)for(double b:special)for(double tr:special)for(double c:{0.,1e12,1e300}){
        const CertifiedTube t{Eigen::Vector3d(a,0.,0.),Eigen::Vector3d(b,0.,0.),tr};const Eigen::Vector3d p(c,0.,0.);const bool kept=retain(t,p,.58),original=frozen::tubeContains(t,p,.58);++queries;mismatch+=original!=(kept&&original);
    }
    std::cout<<"queries,mismatches,roots_pruned\n"<<queries<<','<<mismatch<<','<<pruned<<'\n';return mismatch?1:0;
}
