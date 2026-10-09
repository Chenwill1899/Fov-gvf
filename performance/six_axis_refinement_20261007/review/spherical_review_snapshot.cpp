#include "pc_gvf/spherical_memory.hpp"
#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <cmath>
#include <set>
namespace pc_gvf { namespace depth_angular {
namespace {
int bin(const Eigen::Vector3d& ray) {
    const double pi=std::acos(-1.);
    const int az=std::min(23,std::max(0,int((std::atan2(ray.y(),ray.x())+pi)*24/(2*pi))));
    const int el=std::min(11,std::max(0,int((std::asin(std::max(-1.,std::min(1.,ray.z())))+pi/2)*12/pi)));
    return el*24+az;
}
struct HitBlock {
    std::size_t begin,end;
    Eigen::Vector3d lower,upper;
};
bool cannotContradict(const HitBlock& block,const DepthObservation& old,double maximum_depth,double minimum_z) {
    // Interval evaluation of the exact visibility half-spaces. A block is
    // discarded only when every point fails a necessary containment test.
    // Padding keeps roundoff at a camera seam in the exact pointwise path.
    const Eigen::Matrix3d inverse=old.rotation.transpose();
    const Eigen::Vector3d center=inverse*((block.lower+block.upper)*.5-old.origin);
    const Eigen::Vector3d half=inverse.cwiseAbs()*((block.upper-block.lower)*.5);
    const double pad=1e-10*(1.+center.norm()+half.norm());
    const double ax=(-.5-old.camera.cx())/old.camera.fx();
    const double bx=(old.camera.width()-.5-old.camera.cx())/old.camera.fx();
    const double ay=(-.5-old.camera.cy())/old.camera.fy();
    const double by=(old.camera.height()-.5-old.camera.cy())/old.camera.fy();
    if(center.z()+half.z()+pad<=.03)return true;
    // Every old pixel tail starts no farther than maximum_depth.
    if(maximum_depth<=center.z()-half.z()+.03*minimum_z-pad)return true;
    for(const Eigen::Vector3d& n:{Eigen::Vector3d(1,0,-ax),Eigen::Vector3d(-1,0,bx),
            Eigen::Vector3d(0,1,-ay),Eigen::Vector3d(0,-1,by)})
        if(n.dot(center)+n.cwiseAbs().dot(half)+pad<=.03*n.norm())return true;
    return false;
}
}
void SphericalMemory::ingest(const DepthObservation& o,bool broadphase) {
    if(!std::isfinite(o.stamp)||!o.origin.allFinite()||!o.rotation.allFinite()||o.depth.size()!=o.camera.rays().size())return;
    if(checked_.count(o.view)&&o.stamp<checked_[o.view].first-.1){slots_.clear();checked_.clear();}
    if(checked_.count(o.view)&&o.stamp<=checked_[o.view].first)return;
    checked_[o.view]={o.stamp,o.version};
    std::vector<Eigen::Vector3d> hits;
    hits.reserve(o.obstacle_points?o.obstacle_points->size():o.depth.size());
    if(o.obstacle_points) {
        for(const auto& p:*o.obstacle_points)if(p.allFinite()&&p.z()>0&&p.z()<o.camera.maxDepth()-.01)hits.push_back(o.origin+o.rotation*p);
    }
    if(!o.obstacle_points)for(std::size_t i=0;i<o.depth.size();++i)if(o.depth[i]>0&&std::isfinite(o.depth[i])&&o.depth[i]<o.camera.maxDepth()-.01)
        hits.push_back(o.origin+o.rotation*o.camera.rays()[i]*o.depth[i]/o.camera.rays()[i].z());
    std::vector<HitBlock> blocks;
    if(broadphase)for(std::size_t begin=0;begin<hits.size();begin+=128) {
        HitBlock block{begin,std::min(begin+128,hits.size()),hits[begin],hits[begin]};
        for(std::size_t i=begin+1;i<block.end;++i) {
            block.lower=block.lower.cwiseMin(hits[i]);block.upper=block.upper.cwiseMax(hits[i]);
        }
        blocks.push_back(block);
    }
    for(auto it=slots_.begin();it!=slots_.end();) {
        auto& old=it->second;bool remove=old->revoked||o.stamp-old->stamp>lifetime;
        // A contradiction invalidates the original provenance, including any
        // references borrowed by a previously constructed direction field.
        if(!remove) {
            double minimum_z=1.;
            for(double x:{-.5,old->camera.width()-.5})for(double y:{-.5,old->camera.height()-.5})
                minimum_z=std::min(minimum_z,old->camera.rayFromPixel(Eigen::Vector2d(x,y)).z());
            double maximum_depth=0.;
            for(double d:old->depth)if(std::isfinite(d))maximum_depth=std::max(maximum_depth,d);
            const Eigen::Matrix3d inverse=old->rotation.transpose();
            std::size_t block_index=0;
            for(std::size_t i=0;i<hits.size();++i) {
                if(broadphase&&block_index<blocks.size()&&i==blocks[block_index].begin) {
                    const auto& block=blocks[block_index++];
                    if(cannotContradict(block,*old,maximum_depth,minimum_z)){i=block.end-1;continue;}
                }
                const auto& hit=hits[i];
                if(broadphase) {
                    const Eigen::Vector3d p=inverse*(hit-old->origin);
                    if(p.z()<=.03)continue;
                    const int x=int(std::floor(old->camera.fx()*p.x()/p.z()+old->camera.cx()+.5));
                    const int y=int(std::floor(old->camera.fy()*p.y()/p.z()+old->camera.cy()+.5));
                    if(x<0||y<0||x>=old->camera.width()||y>=old->camera.height())continue;
                    const double depth=old->depth[y*old->camera.width()+x];
                    // Necessary condition only: the unknown tail on the exact
                    // center ray is within 3cm whenever axial gap <= 3cm*cos.
                    // Global minimum cos is conservative for every pixel ray.
                    if(!std::isfinite(depth)||depth<=p.z()+.03*minimum_z-1e-12)continue;
                }
                if(observedEnvelope(*old,hit,.03)){remove=true;old->revoked=true;break;}
            }
        }
        if(remove)it=slots_.erase(it);else ++it;
    }
    const int key=bin(o.rotation.col(2));
    auto retained=std::make_shared<DepthObservation>(o);retained->obstacle_points.reset();retained->supporting_views.clear();
    retained->memory_uncertainty_rate=.05;
    slots_[key]=std::move(retained);
    while(slots_.size()>48) {
        auto oldest=std::min_element(slots_.begin(),slots_.end(),[](const auto& a,const auto& b){return a.second->stamp<b.second->stamp;});slots_.erase(oldest);
    }
}
std::vector<std::shared_ptr<const DepthObservation>> SphericalMemory::evidence(double now)const {
    std::vector<std::shared_ptr<const DepthObservation>> views;
    for(const auto& slot:slots_)if(!slot.second->revoked&&now>=slot.second->stamp-.02&&now-slot.second->stamp<=lifetime)views.push_back(slot.second);
    return views;
}
std::vector<Eigen::Vector3d> SphericalMemory::directions(const Eigen::Vector3d& position,double now)const {
    std::map<int,Eigen::Vector3d> sphere;
    for(const auto& view:evidence(now)) {
        const auto& c=view->camera;
        for(int y=c.height()/6;y<c.height();y+=std::max(1,c.height()/3))
            for(int x=c.width()/6;x<c.width();x+=std::max(1,c.width()/3)) {
                const auto ray=c.ray(x,y);const double d=view->depth[y*c.width()+x];
                if(!std::isfinite(d)||d<=.2)continue;
                // Reproject an actual observed world point, including translation.
                Eigen::Vector3d direction=view->origin+view->rotation*ray*(.7*d/ray.z())-position;
                if(direction.norm()<.1)continue;
                direction.normalize();sphere[bin(direction)]=direction;
            }
    }
    std::vector<Eigen::Vector3d> result;for(const auto& cell:sphere)result.push_back(cell.second);return result;
}
} }
