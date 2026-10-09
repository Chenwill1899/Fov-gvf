#include "pc_gvf/local_depth_path.hpp"
#include <algorithm>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
namespace pc_gvf { namespace depth_angular {
LocalDepthPath localDepthPath(const std::vector<std::shared_ptr<const DepthObservation>>& views,
    const Eigen::Vector3d& position,const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& goal,double body_envelope,double horizon,double response_time,const Eigen::Vector3d* direction_hint) {
    LocalDepthPath out;
    if(views.empty()||!position.allFinite()||!goal.allFinite()||!velocity.allFinite()||
        !std::isfinite(body_envelope)||body_envelope<=0||!std::isfinite(horizon)||horizon<2.||
        !std::isfinite(response_time)||response_time<0.||response_time>2.)return out;
    const double distance=(goal-position).norm();
    if(!std::isfinite(distance)||distance<2.)return out;
    const Eigen::Vector3d forward=(goal-position)/distance;
    Eigen::Vector3d side(-forward.y(),forward.x(),0.);
    if(side.norm()<.01)side=forward.unitOrthogonal();else side.normalize();
    const Eigen::Vector3d up=forward.cross(side).normalized();
    Eigen::Matrix3d basis;basis.col(0)=side;basis.col(1)=up;basis.col(2)=forward;
    constexpr double cell=.25;
    constexpr int half_side=12,half_up=10,ny=2*half_side+1,nz=2*half_up+1;
    const int last=2*static_cast<int>(std::floor(std::min({horizon,distance,8.})/(2*cell)));
    const int count=(last+1)*ny*nz;
    auto index=[](int l,int y,int z){return (l*nz+z)*ny+y;};
    auto local=[](int l,int y,int z){return Eigen::Vector3d((y-half_side)*cell,(z-half_up)*cell,l*cell);};
    std::vector<unsigned char> blocked(count,0),visible(count,0);
    // Candidate centers inside the current body use the same initialization
    // premise as the certifier. This does not certify any translated body.
    for(int l=1;l<=last;++l)for(int z=0;z<nz;++z)for(int y=0;y<ny;++y)
        if(local(l,y,z).norm()<=body_envelope)visible[index(l,y,z)]=1;
    // The certifier already includes body, depth and execution margins.
    // This small proposal-only padding reduces grid-edge chatter; it is not
    // an additional execution envelope or a proof of entire grid cells.
    const double radius=body_envelope+.10;
    // Anticipate velocity-reference and vehicle lag by extending obstacle
    // proposals against measured motion. Lateral clearance remains geometric;
    // this capsule is only a candidate heuristic, never motion authorization.
    Eigen::Vector3d lag=basis.transpose()*velocity*response_time;
    if(!lag.allFinite()||lag.norm()>4.)return out;
    const double lag_squared=lag.squaredNorm();
    for(const auto& view:views) {
        if(!view||view->revoked||!view->origin.allFinite()||!view->rotation.allFinite()||view->depth.size()!=view->camera.rays().size())continue;
        const auto native=view->obstacle_points?view->obstacle_points:view->proposal_points;
        const auto raster=native?std::vector<Eigen::Vector3d>():backprojectObstaclePoints(view->depth,view->camera,1);
        const auto& candidates=native?*native:raster;
        const std::size_t stride=std::max(std::size_t(1),(candidates.size()+4095)/4096);
        for(std::size_t point=0;point<candidates.size();point+=stride) {
            const auto& hit=candidates[point];
            const Eigen::Vector3d p=basis.transpose()*(view->origin+view->rotation*hit-position);
            const Eigen::Vector3d lo=p.cwiseMin(p-lag)-Eigen::Vector3d::Constant(radius);
            const Eigen::Vector3d hi=p.cwiseMax(p-lag)+Eigen::Vector3d::Constant(radius);
            if(!p.allFinite()||lo.x()>half_side*cell||hi.x()< -half_side*cell||
                lo.y()>half_up*cell||hi.y()< -half_up*cell||hi.z()<0.||lo.z()>last*cell)continue;
            const int ya=std::max(0,int(std::ceil(lo.x()/cell))+half_side);
            const int yb=std::min(ny-1,int(std::floor(hi.x()/cell))+half_side);
            const int za=std::max(0,int(std::ceil(lo.y()/cell))+half_up);
            const int zb=std::min(nz-1,int(std::floor(hi.y()/cell))+half_up);
            const int la=std::max(0,int(std::ceil(lo.z()/cell)));
            const int lb=std::min(last,int(std::floor(hi.z()/cell)));
            for(int l=la;l<=lb;++l)for(int z=za;z<=zb;++z)for(int y=ya;y<=yb;++y) {
                const Eigen::Vector3d delta=local(l,y,z)-p;
                const double fraction=lag_squared>1e-12?std::max(0.,std::min(1.,-delta.dot(lag)/lag_squared)):0.;
                if((delta+fraction*lag).squaredNorm()<=radius*radius)blocked[index(l,y,z)]=1;
            }
        }
        const Eigen::Matrix3d rotation=view->rotation.transpose()*basis;
        const Eigen::Vector3d origin=view->rotation.transpose()*(position-view->origin);
        for(int l=1;l<=last;++l)for(int z=0;z<nz;++z)for(int y=0;y<ny;++y) {
            const int i=index(l,y,z);if(visible[i]||blocked[i])continue;
            const Eigen::Vector3d p=origin+rotation*local(l,y,z);
            if(p.z()<=.03||p.z()+.03>=view->camera.maxDepth())continue;
            const double u=view->camera.fx()*p.x()/p.z()+view->camera.cx();
            const double v=view->camera.fy()*p.y()/p.z()+view->camera.cy();
            if(!std::isfinite(u)||!std::isfinite(v)||u<-.5||v<-.5||
                u>=view->camera.width()-.5||v>=view->camera.height()-.5)continue;
            const int x=int(std::floor(u+.5)),row=int(std::floor(v+.5));
            if(x<0||row<0||x>=view->camera.width()||row>=view->camera.height())continue;
            const double depth=view->depth[row*view->camera.width()+x];
            if(std::isfinite(depth)&&depth>p.z()+.03)visible[i]=1;
        }
    }
    // The body at layer zero is checked by the final certifier. No new grid
    // volume around it is marked free; all subsequent vertices need visibility.
    bool straight_visible=true;
    for(int l=1;l<=last;++l)if(blocked[index(l,half_side,half_up)]||!visible[index(l,half_side,half_up)])straight_visible=false;
    const int start=index(0,half_side,half_up);
    std::vector<double> costs(count,std::numeric_limits<double>::infinity());
    std::vector<int> parent(count,-1);
    costs[start]=0.;
    auto coordinates=[](int i){int y=i%ny;i/=ny;return Eigen::Vector3i(i/nz,y,i%nz);};
    const Eigen::Vector3d measured=velocity.norm()>.03?velocity.normalized():forward;
    for(int l=2;l<=last;l+=2)for(int z=0;z<nz;++z)for(int y=0;y<ny;++y) {
        const int i=index(l,y,z);if(blocked[i]||!visible[i])continue;
        for(int dz=-1;dz<=1;++dz)for(int dy=-2;dy<=2;++dy) {
            const int py=y-dy,pz=z-dz;
            if(py<0||py>=ny||pz<0||pz>=nz)continue;
            const int previous=index(l-2,py,pz);if(!std::isfinite(costs[previous]))continue;
            bool edge=true;
            for(int yy:{(y+py)/2,(y+py+1)/2})for(int zz:{(z+pz)/2,(z+pz+1)/2}) {
                const int mid=index(l-1,yy,zz);if(blocked[mid]||!visible[mid])edge=false;
            }
            if(!edge)continue;
            const Eigen::Vector3d delta=local(l,y,z)-local(l-2,py,pz);
            const Eigen::Vector3d direction=basis*delta.normalized();
            Eigen::Vector3d before=measured;
            if(parent[previous]>=0) {
                const auto a=coordinates(parent[previous]);
                before=basis*(local(l-2,py,pz)-local(a.x(),a.y(),a.z())).normalized();
            }
            double cost=costs[previous]+delta.norm()+.04*(direction-before).squaredNorm();
            // A small soft preference at the lookahead section stabilizes
            // nearly equal detours. Every graph edge is rebuilt from current
            // evidence; no old path is committed through new occlusion.
            if(!straight_visible&&l==std::min(last,12)&&direction_hint&&direction_hint->allFinite()&&direction_hint->norm()>.5) {
                const Eigen::Vector3d target=basis*local(l,y,z).normalized();
                cost+=.4*(target-direction_hint->normalized()).squaredNorm();
            }
            if(cost<costs[i]){costs[i]=cost;parent[i]=previous;}
        }
    }
    int best=-1;double best_cost=std::numeric_limits<double>::infinity();
    for(int z=0;z<nz;++z)for(int y=0;y<ny;++y) {
        const int i=index(last,y,z);
        const double cost=costs[i]+(goal-position-basis*local(last,y,z)).norm();
        if(cost<best_cost){best_cost=cost;best=i;}
    }
    if(best<0)return out;
    std::vector<Eigen::Vector3d> path;
    for(int i=best;i>=0;i=parent[i]) {
        const auto c=coordinates(i);path.push_back(position+basis*local(c.x(),c.y(),c.z()));
    }
    std::reverse(path.begin(),path.end());
    if(path.size()<2)return out;
    const std::size_t target=std::min(std::size_t(6),path.size()-1);
    out.direction=(path[target]-position).normalized();out.points=path;out.valid=true;
    for(std::size_t i=1;i<path.size();++i)out.length+=(path[i]-path[i-1]).norm();
    return out;
}
Eigen::Vector3d localPathDirection(const std::vector<Eigen::Vector3d>& path,const Eigen::Vector3d& position,double lookahead) {
    if(path.size()<2||!position.allFinite()||!std::isfinite(lookahead)||lookahead<=0)return Eigen::Vector3d::Zero();
    std::size_t segment=0;double best=std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest=path.front();
    for(std::size_t i=1;i<path.size();++i) {
        const Eigen::Vector3d delta=path[i]-path[i-1];
        const double fraction=delta.squaredNorm()>1e-12?std::max(0.,std::min(1.,(position-path[i-1]).dot(delta)/delta.squaredNorm())):0.;
        const Eigen::Vector3d p=path[i-1]+fraction*delta;
        const double distance=(p-position).squaredNorm();
        if(distance<best){best=distance;segment=i;nearest=p;}
    }
    Eigen::Vector3d target=nearest;
    for(std::size_t i=segment;i<path.size();++i) {
        const Eigen::Vector3d delta=path[i]-target;const double length=delta.norm();
        if(length>=lookahead&&length>1e-9){target+=lookahead/length*delta;break;}
        lookahead-=length;target=path[i];
    }
    const Eigen::Vector3d delta=target-position;
    return delta.norm()>1e-6?delta.normalized().eval():Eigen::Vector3d::Zero();
}
}}
