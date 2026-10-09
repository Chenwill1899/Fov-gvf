#include "pc_gvf/depth_uncertainty.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace pc_gvf { namespace depth_angular {
std::vector<double> conservativeDepth(const std::vector<double>& input,int w,int h,
    double fx,double fy,double cx,double cy,const DepthUncertaintyConfig& c) {
    if(!c.enabled)return input;
    if(w<1||h<1||!std::isfinite(fx)||!std::isfinite(fy)||fx<=0||fy<=0||
       !std::isfinite(cx)||!std::isfinite(cy))throw std::invalid_argument("invalid depth intrinsics");
    for(double v:{c.sigma0,c.sigma_range2,c.sigma_multiplier,c.pose_bound,c.edge_threshold,c.reserved_radial_bound})
        if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid depth uncertainty bound");
    if(c.edge_radius<0||c.edge_radius>4)throw std::invalid_argument("invalid edge radius");
    if(input.size()!=static_cast<std::size_t>(w)*static_cast<std::size_t>(h))
        throw std::invalid_argument("invalid depth size");
    auto out=input;
    // The old square footprint selects its minimum exactly when it is unknown
    // or differs from the center by more than edge_threshold. A square minimum
    // is separable: min_y(min_x(z)) = min_{x,y}(z). Normalize unknown once and
    // reuse horizontal minima instead of checking every neighbor in every
    // square. This changes neither the footprint nor the uncertainty bound.
    std::vector<double> horizontal;
    if(c.edge_radius>0) {
        for(double& z:out)if(!std::isfinite(z)||z<=0)z=0.;
        horizontal.resize(input.size());
        for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
            const auto row=static_cast<std::size_t>(y)*w;
            double near=out[row+x];
            for(int u=std::max(0,x-c.edge_radius);u<=std::min(w-1,x+c.edge_radius);++u)
                near=std::min(near,out[row+u]);
            horizontal[row+x]=near;
        }
    }
    // Intrinsics do not vary across a row/column. Preserve the exact original
    // expression order, while avoiding repeated divisions at native resolution.
    std::vector<double> horizontal_ray2(w),vertical_ray2(h);
    if(c.reserved_radial_bound>0.) {
        for(int x=0;x<w;++x){const double u=(std::abs(x-cx)+.5)/fx;horizontal_ray2[x]=u*u;}
        for(int y=0;y<h;++y){const double v=(std::abs(y-cy)+.5)/fy;vertical_ray2[y]=v*v;}
    }
    for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
        const auto i=static_cast<std::size_t>(y)*w+x;
        const double d=input[i];
        if(!std::isfinite(d)||d<=0){out[i]=0;continue;}
        double near=d;
        // Occlusion/missing-return footprints expand; no spatial averaging
        // across the foreground/background discontinuity can create freedom.
        if(c.edge_radius>0) {
            double minimum=d;
            for(int v=std::max(0,y-c.edge_radius);v<=std::min(h-1,y+c.edge_radius);++v)
                minimum=std::min(minimum,horizontal[static_cast<std::size_t>(v)*w+x]);
            if(minimum==0.||d-minimum>c.edge_threshold)near=minimum;
        }
        if(near==0.){out[i]=0.;continue;}
        const double minimum_z=c.reserved_radial_bound>0.?
            1./std::sqrt(1.+horizontal_ray2[x]+vertical_ray2[y]):0.;
        const double bound=std::max(0.,c.sigma_multiplier*(c.sigma0+c.sigma_range2*near*near)+c.pose_bound-
            c.reserved_radial_bound*minimum_z);
        out[i]=std::max(0.,std::min(d,near-bound));
    }
    return out;
}
DepthObservation conservativeObservation(const DepthObservation& input,const DepthUncertaintyConfig& c) {
    auto out=input;
    out.depth=conservativeDepth(input.depth,input.camera.width(),input.camera.height(),
        input.camera.fx(),input.camera.fy(),input.camera.cx(),input.camera.cy(),c);
    return out;
}
} }
