#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <random>
#include <limits>
using namespace pc_gvf::depth_angular;
int main(){
    Camera c(64,48,90,68,10.);double elapsed[2]={};SphericalMemory slow,fast;
    for(int k=0;k<80;++k) {
        const double yaw=(k%4)*std::acos(-1.)/2.;
        DepthObservation o(c,std::vector<double>(3072,4.),Eigen::Vector3d(.02*(k/4),0,0),
            Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation(),k*.025,k+1,k%4);
        auto hits=std::make_shared<std::vector<Eigen::Vector3d>>();
        for(int y=0;y<240;++y)for(int x=0;x<320;++x) {
            const Eigen::Vector3d ray=c.rayFromPixel(Eigen::Vector2d(x/5.,y/5.));
            hits->push_back(ray*((k%13==0&&x>150&&x<154)?3.8:4.)/ray.z());
        }o.obstacle_points=hits;
        auto start=std::chrono::steady_clock::now();slow.ingest(o,false);
        elapsed[0]+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        start=std::chrono::steady_clock::now();fast.ingest(o,true);
        elapsed[1]+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        if(slow.slots().size()!=fast.slots().size())return 1;
        for(const auto& slot:slow.slots())if(!fast.slots().count(slot.first)||fast.slots().at(slot.first)->stamp!=slot.second->stamp)return 2;
    }
    // A world-space midpoint used to lose half an ULP near x=1e9 and
    // suppress a true contradiction. Check the same near-seam geometry at
    // five translations, including 1e12; exact and accelerated provenance
    // revocation must agree even when the two native hits are one ULP apart.
    int translated_revocations=0;
    for(double origin:{0.,1e6,1e9,1e12,-1e9}) {
        Camera camera(32,24,90.,68.,10.);
        DepthObservation old(camera,std::vector<double>(768,10.),Eigen::Vector3d(origin,0,0),Eigen::Matrix3d::Identity(),1.,1);
        const double lower=origin-2.,upper=std::nextafter(lower,std::numeric_limits<double>::infinity());
        const double z=.03*std::sqrt(2.)-(upper-origin)+1e-9;
        auto next=old;next.stamp=1.1;next.version=2;
        auto points=std::make_shared<std::vector<Eigen::Vector3d>>();
        points->push_back(Eigen::Vector3d(lower-origin,0,z));points->push_back(Eigen::Vector3d(upper-origin,0,z));next.obstacle_points=points;
        SphericalMemory exact,fast;exact.ingest(old,false);fast.ingest(old,true);
        auto a=exact.evidence(1.).front(),b=fast.evidence(1.).front();
        if(!observedEnvelope(*a,old.origin+points->back(),.03))return 7;
        exact.ingest(next,false);fast.ingest(next,true);
        if(!a->revoked||!b->revoked)return 8;
        ++translated_revocations;
    }
    // Adversarial, seeded parity with the exact pointwise oracle. Keep borrowed
    // pointers too: erasing a slot is insufficient if old evidence stays live.
    std::mt19937_64 rng(20261007);
    std::uniform_real_distribution<double> unit(-1.,1.);
    int revoked=0;
    for(int trial=0;trial<160;++trial) {
        SphericalMemory exact,blocked;
        std::vector<std::shared_ptr<const DepthObservation>> borrowed_exact,borrowed_blocked;
        for(int view=0;view<12;++view) {
            const double yaw=view*std::acos(-1.)/6.+.02*unit(rng);
            std::vector<double> depth(3072,4.+.5*unit(rng));
            for(int k=0;k<12;++k)depth[rng()%depth.size()]=k%3?0.:std::numeric_limits<double>::quiet_NaN();
            DepthObservation old(c,depth,Eigen::Vector3d(.1*unit(rng),.1*unit(rng),.1*unit(rng)),
                Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation(),0.,view+1,view);
            auto a=std::make_shared<DepthObservation>(old),b=std::make_shared<DepthObservation>(old);
            exact.restore(view,a);blocked.restore(view,b);borrowed_exact.push_back(a);borrowed_blocked.push_back(b);
        }
        DepthObservation current(c,std::vector<double>(3072,4.),Eigen::Vector3d(.4*unit(rng),.4*unit(rng),.4*unit(rng)),
            Eigen::AngleAxisd(unit(rng)*std::acos(-1.),Eigen::Vector3d::UnitZ()).toRotationMatrix()*fixedCameraRotation(),.1,20,20);
        auto points=std::make_shared<std::vector<Eigen::Vector3d>>();
        for(int k=0;k<1025;++k) {
            // Include partial final blocks and points straddling camera seams.
            const double z=trial%3==0?4.:2.+3.*std::abs(unit(rng));
            points->push_back(Eigen::Vector3d(z*unit(rng),z*unit(rng),z));
        }
        current.obstacle_points=points;exact.ingest(current,false);blocked.ingest(current,true);
        if(exact.slots().size()!=blocked.slots().size())return 3;
        for(const auto& slot:exact.slots())if(!blocked.slots().count(slot.first)||blocked.slots().at(slot.first)->stamp!=slot.second->stamp)return 4;
        for(std::size_t k=0;k<borrowed_exact.size();++k) {
            if(borrowed_exact[k]->revoked!=borrowed_blocked[k]->revoked)return 5;
            revoked+=borrowed_exact[k]->revoked;
        }
    }
    if(revoked==0)return 6;
    std::cout<<"frames,native_points_per_frame,slow_ms,broadphase_ms,same_retained_evidence,seeded_parity_cases,borrowed_revocations,large_world_revocations\n80,76800,"<<elapsed[0]<<','<<elapsed[1]<<",1,160,"<<revoked<<','<<translated_revocations<<"\n";
    return 0;
}
