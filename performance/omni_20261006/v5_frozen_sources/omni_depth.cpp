#include "pc_gvf/omni_depth.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <queue>

namespace pc_gvf { namespace depth_angular {
namespace {
constexpr double pi = 3.14159265358979323846;
double clamp(double x) { return std::max(-1., std::min(1., x)); }
Eigen::Vector3d ray(double yaw, double elevation) {
    return {std::cos(elevation)*std::cos(yaw),
            std::cos(elevation)*std::sin(yaw), std::sin(elevation)};
}
struct Sphere { Eigen::Vector3d center; double radius; };
double entry(const Eigen::Vector3d& direction, const Sphere& sphere, double limit) {
    const double norm2 = sphere.center.squaredNorm();
    const double radius2 = sphere.radius*sphere.radius;
    if (norm2 <= radius2) return 0.;
    const double along = direction.dot(sphere.center);
    if (along <= 0.) return limit;
    const double perpendicular2 = std::max(0., norm2-along*along);
    if (perpendicular2 > radius2) return limit;
    return std::min(limit, std::max(0., along-std::sqrt(radius2-perpendicular2)));
}
bool visible(const DepthObservation& view, const Eigen::Vector3d& point) {
    const Eigen::Vector3d p = view.rotation.transpose()*(point-view.origin);
    if (p.z() <= 0.) return false;
    // Full pixel cells, including half-pixel FOV edges; never clamp outside rays.
    const double u = view.camera.fx()*p.x()/p.z()+view.camera.cx();
    const double v = view.camera.fy()*p.y()/p.z()+view.camera.cy();
    if (u < -.5 || v < -.5 || u >= view.camera.width()-.5 || v >= view.camera.height()-.5)
        return false;
    const double d = view.depth[static_cast<int>(std::floor(v+.5))*view.camera.width()+
                               static_cast<int>(std::floor(u+.5))];
    return std::isfinite(d) && d > p.z();
}
}

OmniDepthResult omniDepthProposal(
    const std::vector<std::shared_ptr<const DepthObservation>>& views,
    const Eigen::Vector3d& position, const Eigen::Vector3d& intent,
    const Eigen::Vector3d& previous_direction, double now, double corridor,
    const OmniDepthConfig& cfg,
    const std::function<bool(const Eigen::Vector3d&)>& certify)
{
    if (cfg.azimuth_bins < 8 || cfg.azimuth_bins > 720 ||
        cfg.elevation_bins < 3 || cfg.elevation_bins > 181 ||
        !std::isfinite(cfg.horizon) || cfg.horizon <= 0. ||
        !std::isfinite(cfg.radius) || cfg.radius <= 0. ||
        !std::isfinite(cfg.max_age) || cfg.max_age <= 0. ||
        !std::isfinite(cfg.continuity_weight) || cfg.continuity_weight < 0. ||
        cfg.max_certificates < 1)
        throw std::invalid_argument("invalid omni depth proposal configuration");
    OmniDepthResult result;
    if (!position.allFinite() || !intent.allFinite() || intent.norm() < 1e-8 ||
        !std::isfinite(now) || !std::isfinite(corridor) || corridor < 0. || !certify)
        return result;
    std::vector<const DepthObservation*> fresh;
    std::vector<Sphere> spheres;
    for (const auto& ptr : views) {
        if (!ptr || ptr->revoked || !std::isfinite(ptr->stamp) ||
            now-ptr->stamp > cfg.max_age || ptr->stamp-now > .02 ||
            !ptr->origin.allFinite() || !ptr->rotation.allFinite() ||
            ptr->depth.size() != ptr->camera.rays().size()) continue;
        const auto& view = *ptr;
        fresh.push_back(&view);
        for (std::size_t i=0; i<view.depth.size(); ++i) {
            const double z = view.depth[i];
            if (!std::isfinite(z) || z <= 0. || z >= view.camera.maxDepth()) continue;
            const auto& camera_ray = view.camera.rays()[i];
            const Eigen::Vector3d point = view.origin + view.rotation*(z/camera_ray.z()*camera_ray)-position;
            // Cover the coarse pixel's transverse footprint when proposing.
            // Original conservative frusta remain the sole safety proof.
            const double radius = cfg.radius + .5*z*std::hypot(1./view.camera.fx(),1./view.camera.fy());
            if (point.norm()-radius < cfg.horizon) spheres.push_back({point,radius});
        }
    }
    result.fresh_views = static_cast<int>(fresh.size());
    if (fresh.empty()) return result;
    const Eigen::Vector3d desired=intent.normalized();
    auto exact_clearance=[&](const Eigen::Vector3d& d) {
        double clearance=cfg.horizon;
        for (const auto& sphere : spheres) clearance=entry(d,sphere,clearance);
        return clearance;
    };
    const double direct_clearance=exact_clearance(desired);
    bool direct_checked=false;
    if (direct_clearance >= cfg.horizon-1e-9) {
        ++result.certificates;direct_checked=true;
        if (certify(desired)) {
            result.direction=desired;result.valid=true;
            // No need to rasterize thousands of unused alternatives in open
            // space. Visual subscribers can explicitly request the whole atlas.
            if (!cfg.include_atlas) return result;
        }
    }
    const int width=cfg.azimuth_bins, height=cfg.elevation_bins;
    const double dyaw=2*pi/width, delevation=pi/(height-1);
    result.rays.reserve(width*height);
    result.clearance.assign(width*height,cfg.horizon);
    result.observed.assign(width*height,0);
    for (int y=0;y<height;++y) for (int x=0;x<width;++x) {
        const Eigen::Vector3d d=ray(-pi+x*dyaw,-pi/2+y*delevation);
        result.rays.push_back(d);
        for (const auto* view : fresh)
            if (visible(*view,position+cfg.horizon*d)) {result.observed[y*width+x]=1;break;}
    }
    // Rasterize angular caps on a periodic longitude domain. Bound each row
    // analytically; no artificial obstacle or disconnected edge at +/-pi.
    for (const auto& sphere : spheres) {
        const double distance=sphere.center.norm();
        if (distance <= sphere.radius) {
            std::fill(result.clearance.begin(),result.clearance.end(),0.);
            break;
        }
        const double yaw=std::atan2(sphere.center.y(),sphere.center.x());
        const double elevation=std::asin(clamp(sphere.center.z()/distance));
        const double alpha=std::asin(std::min(1.,sphere.radius/distance));
        const int ymin=std::max(0,static_cast<int>(std::ceil((elevation-alpha+pi/2)/delevation)));
        const int ymax=std::min(height-1,static_cast<int>(std::floor((elevation+alpha+pi/2)/delevation)));
        for (int y=ymin;y<=ymax;++y) {
            const double e=-pi/2+y*delevation;
            const double denominator=std::cos(e)*std::cos(elevation);
            const double numerator=std::cos(alpha)-std::sin(e)*std::sin(elevation);
            if (denominator > 1e-12 && numerator/denominator > 1.+1e-12) continue;
            const double half=denominator <= 1e-12 ? pi : std::acos(clamp(numerator/denominator));
            const int first=static_cast<int>(std::ceil((yaw-half+pi)/dyaw));
            const int last=std::min(first+width-1,static_cast<int>(std::floor((yaw+half+pi)/dyaw)));
            for (int a=first;a<=last;++a) {
                const int x=(a%width+width)%width, i=y*width+x;
                result.clearance[i]=entry(result.rays[i],sphere,result.clearance[i]);
            }
        }
    }
    if (result.valid) return result;
    // Distance to the blocked angular domain, with periodic longitude and
    // physical angular edge lengths. Prefer the middle of a broad corridor,
    // not a direction barely grazing an inflated obstacle.
    constexpr double room_cap=.4;
    std::vector<double> angular_clearance(width*height,room_cap);
    using QueueItem=std::pair<double,int>;
    std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> queue;
    for(int i=0;i<width*height;++i)
        if(!result.observed[i]||result.clearance[i]<cfg.horizon-1e-9) angular_clearance[i]=0.;
    // Only boundary cells can reduce a free cell's capped distance. Seeding
    // blocked interiors needlessly dominates the heap on a spherical atlas.
    for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
        const int i=y*width+x;if(angular_clearance[i]!=0.)continue;
        bool boundary=false;
        for(int dy=-1;dy<=1&&!boundary;++dy)for(int dx=-1;dx<=1;++dx) {
            const int yy=y+dy;if(yy<0||yy>=height)continue;
            if(angular_clearance[yy*width+(x+dx+width)%width]>0.){boundary=true;break;}
        }
        if(boundary)queue.push({0.,i});
    }
    while(!queue.empty()) {
        const auto item=queue.top();queue.pop();
        const int i=item.second,y=i/width,x=i%width;
        if(item.first>angular_clearance[i])continue;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
            const int yy=y+dy;if((!dx&&!dy)||yy<0||yy>=height)continue;
            const int xx=(x+dx+width)%width,j=yy*width+xx;
            const double e=-pi/2+(y+.5*dy)*delevation;
            const double distance=item.first+std::hypot(dx*dyaw*std::cos(e),dy*delevation);
            if(distance+1e-12<angular_clearance[j]){angular_clearance[j]=distance;queue.push({distance,j});}
        }
    }
    auto angular_room=[&](const Eigen::Vector3d& d) {
        int x=static_cast<int>(std::lround((std::atan2(d.y(),d.x())+pi)/dyaw));x=(x%width+width)%width;
        const int y=std::max(0,std::min(height-1,static_cast<int>(std::lround((std::asin(clamp(d.z()))+pi/2)/delevation))));
        return angular_clearance[y*width+x];
    };
    const bool has_previous=previous_direction.allFinite() && previous_direction.norm()>1e-8 &&
        previous_direction.normalized().dot(desired)>0.;
    const Eigen::Vector3d previous=has_previous ? previous_direction.normalized().eval() : desired;
    struct Candidate { Eigen::Vector3d d; double cost; };
    std::vector<Candidate> candidates;
    auto score=[&](const Eigen::Vector3d& d,double clearance) {
        const double angle=std::acos(clamp(d.dot(desired)));
        const double continuity=std::acos(clamp(d.dot(previous)));
        (void)clearance;
        const double elevation_error=std::asin(clamp(d.z()))-std::asin(clamp(desired.z()));
        return .25*(angle*angle+elevation_error*elevation_error)+
            .25*cfg.continuity_weight*continuity*continuity-.55*std::min(.4,angular_room(d));
    };
    auto append=[&](const Eigen::Vector3d& d,double clearance,bool seen) {
        if (!seen || clearance < cfg.horizon-1e-9 || d.dot(desired) <= 0.) return;
        candidates.push_back({d,score(d,clearance)});
    };
    auto seen=[&](const Eigen::Vector3d& d) {
        return std::any_of(fresh.begin(),fresh.end(),[&](const DepthObservation* view){
            return visible(*view,position+cfg.horizon*d);
        });
    };
    append(desired,direct_clearance,seen(desired));
    if (has_previous) append(previous,exact_clearance(previous),seen(previous));
    for (std::size_t i=0;i<result.rays.size();++i) append(result.rays[i],result.clearance[i],result.observed[i]);
    std::stable_sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.cost<b.cost;});
    std::vector<Eigen::Vector3d> attempted;
    if (direct_checked) attempted.push_back(desired);
    for (const auto& candidate : candidates) {
        if (result.certificates >= cfg.max_certificates) break;
        const double spacing=(result.certificates<12?8.:20.)*pi/180.;
        if (std::any_of(attempted.begin(),attempted.end(),[&](const Eigen::Vector3d& d){return d.dot(candidate.d)>std::cos(spacing);})) continue;
        attempted.push_back(candidate.d);
        ++result.certificates;
        if (!certify(candidate.d)) continue;
        result.direction=candidate.d;result.valid=true;
        double best_cost=candidate.cost;
        // Sub-cell refinement toward intent is certified after interpolation.
        // Interpolate vectors, never angles across a wrapped seam.
        for (double fraction : {.5,.25,.125}) {
            if (result.certificates >= cfg.max_certificates) break;
            const Eigen::Vector3d refined=((1.-fraction)*result.direction+fraction*desired).normalized();
            const double clearance=exact_clearance(refined);
            if (clearance < cfg.horizon-1e-9 || !seen(refined)) continue;
            const double cost=score(refined,clearance);
            if (cost > best_cost) continue;
            ++result.certificates;
            if (certify(refined)) {result.direction=refined;best_cost=cost;}
        }
        break;
    }
    return result;
}
} }
