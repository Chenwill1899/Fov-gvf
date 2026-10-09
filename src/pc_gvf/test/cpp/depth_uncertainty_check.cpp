#include "pc_gvf/depth_uncertainty.hpp"
#include <iostream>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <limits>
#include <string>
#include <random>
#include <stdexcept>
using namespace pc_gvf::depth_angular;

namespace {
// Frozen P5 implementation before this refinement. Test-only reference: there
// is no production switch that can relax or replace the uncertainty bound.
std::vector<double> referenceDepthBeforeRefinement(const std::vector<double>& input,int w,int h,
    double fx,double fy,double cx,double cy,const DepthUncertaintyConfig& c) {
    if(!c.enabled)return input;
    if(w<1||h<1||!std::isfinite(fx)||!std::isfinite(fy)||fx<=0||fy<=0||
       !std::isfinite(cx)||!std::isfinite(cy))throw std::invalid_argument("invalid depth intrinsics");
    for(double v:{c.sigma0,c.sigma_range2,c.sigma_multiplier,c.pose_bound,c.edge_threshold,c.reserved_radial_bound})
        if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid depth uncertainty bound");
    if(c.edge_radius<0||c.edge_radius>4)throw std::invalid_argument("invalid edge radius");
    auto out=input;
    if(input.size()!=static_cast<std::size_t>(w*h))throw std::invalid_argument("invalid depth size");
    for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
        const double d=input[y*w+x];
        if(!std::isfinite(d)||d<=0){out[y*w+x]=0;continue;}
        double near=d;
        // Occlusion/missing-return footprints expand; no spatial averaging
        // across the foreground/background discontinuity can create freedom.
        for(int v=std::max(0,y-c.edge_radius);v<=std::min(h-1,y+c.edge_radius);++v)
            for(int u=std::max(0,x-c.edge_radius);u<=std::min(w-1,x+c.edge_radius);++u) {
                const double z=input[v*w+u];
                if(!std::isfinite(z)||z<=0)near=0;
                else if(d-z>c.edge_threshold)near=std::min(near,z);
            }
        const double u=(std::abs(x-cx)+.5)/fx,v=(std::abs(y-cy)+.5)/fy;
        const double minimum_z=1./std::sqrt(1.+u*u+v*v);
        const double bound=std::max(0.,c.sigma_multiplier*(c.sigma0+c.sigma_range2*near*near)+c.pose_bound-
            c.reserved_radial_bound*minimum_z);
        out[y*w+x]=std::max(0.,std::min(d,near-bound));
    }
    return out;
}


bool exactlyEqual(const std::vector<double>& a,const std::vector<double>& b) {
    return a.size()==b.size()&&std::memcmp(a.data(),b.data(),a.size()*sizeof(double))==0;
}
std::size_t compareImplementations() {
    std::mt19937 rng(7102026);std::uniform_real_distribution<double> depth(.01,12.);
    std::size_t compared=0;
    for(int frame=0;frame<420;++frame) {
        const int w=frame%7==0?1:1+(frame*17)%103;
        const int h=frame%11==0?1:1+(frame*13)%71;
        std::vector<double> input(w*h);
        for(std::size_t i=0;i<input.size();++i) {
            input[i]=frame%3==0?8.:depth(rng);
            const auto kind=rng()%83;
            if(kind==0)input[i]=0.;
            if(kind==1)input[i]=-1.;
            if(kind==2)input[i]=std::numeric_limits<double>::quiet_NaN();
            if(kind==3)input[i]=std::numeric_limits<double>::infinity();
            if(kind==4)input[i]=-std::numeric_limits<double>::infinity();
        }
        DepthUncertaintyConfig cfg;cfg.enabled=true;cfg.edge_radius=frame%5;
        cfg.edge_threshold=frame%4==0?0.:.30;
        cfg.reserved_radial_bound=frame%3==0?0.:.03;
        if(frame%13==0){cfg.sigma0=0.;cfg.sigma_range2=0.;cfg.pose_bound=0.;}
        const double fx=.8*w,fy=1.1*h,cx=.47*w,cy=.42*h;
        const auto before=referenceDepthBeforeRefinement(input,w,h,fx,fy,cx,cy,cfg);
        const auto after=conservativeDepth(input,w,h,fx,fy,cx,cy,cfg);
        if(!exactlyEqual(before,after))throw std::runtime_error("uncertainty footprint changed");
        for(std::size_t i=0;i<input.size();++i) {
            if(!std::isfinite(after[i])||after[i]<0.||
               ((!std::isfinite(input[i])||input[i]<=0.)&&after[i]!=0.)||
               (std::isfinite(input[i])&&input[i]>0.&&after[i]>input[i]))
                throw std::runtime_error("unknown or increased depth accepted");
        }
        compared+=input.size();
        cfg.enabled=false;
        if(!exactlyEqual(input,conservativeDepth(input,w,h,fx,fy,cx,cy,cfg)))
            throw std::runtime_error("disabled uncertainty changed input");
    }
    // Equality and adjacent floating-point values at the strict edge threshold.
    for(double z:{std::nextafter(2.25,0.),2.25,std::nextafter(2.25,3.)}) {
        const std::vector<double> input{2.,z,z,z,z,z};
        DepthUncertaintyConfig cfg;cfg.enabled=true;cfg.edge_threshold=.25;
        for(int r=0;r<=4;++r) {
            cfg.edge_radius=r;
            if(!exactlyEqual(referenceDepthBeforeRefinement(input,3,2,2.,2.,1.,.5,cfg),
                conservativeDepth(input,3,2,2.,2.,1.,.5,cfg)))
                throw std::runtime_error("strict threshold semantics changed");
            compared+=input.size();
        }
    }
    // Malformed geometry/configuration must reject before reading any pixels.
    auto mustReject=[](const std::vector<double>& input,int w,int h,double fx,DepthUncertaintyConfig cfg) {
        try{conservativeDepth(input,w,h,fx,2.,1.,1.,cfg);}
        catch(const std::invalid_argument&){return;}
        throw std::runtime_error("malformed depth was accepted");
    };
    DepthUncertaintyConfig cfg;cfg.enabled=true;
    mustReject({},3,2,2.,cfg);mustReject({},0,2,2.,cfg);mustReject({},3,2,0.,cfg);
    mustReject({},std::numeric_limits<int>::max(),2,2.,cfg);
    cfg.edge_radius=5;mustReject(std::vector<double>(6,2.),3,2,2.,cfg);
    cfg.edge_radius=1;cfg.sigma0=-.1;mustReject(std::vector<double>(6,2.),3,2,2.,cfg);
    cfg.sigma0=std::numeric_limits<double>::quiet_NaN();
    mustReject(std::vector<double>(6,2.),3,2,2.,cfg);
    return compared;
}

int benchmark() {
    using Clock=std::chrono::steady_clock;
    constexpr int w=320,h=240,batches=8,views=4,rounds=9;
    volatile double sink=0.;
    std::cout<<"scene,radius,credit_m,round,order,before_ms_per_four_views,after_ms_per_four_views,exact_equal\n";
    for(const std::string scene:{"smooth","occlusion","missing"})for(int radius:{0,1,4}) {
        DepthUncertaintyConfig cfg;cfg.enabled=true;cfg.edge_radius=radius;cfg.reserved_radial_bound=.03;
        std::vector<std::vector<double>> images;
        for(int view=0;view<views;++view) {
            std::vector<double> d(w*h);
            for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
                d[y*w+x]=5.+.004*x+.002*y+.1*view;
                if(scene!="smooth"&&x%73<6&&y%71>11)d[y*w+x]=1.2+.1*view;
                if(scene=="missing"&&(x+3*y+view)%113==0)d[y*w+x]=
                    (x%2)?0.:std::numeric_limits<double>::quiet_NaN();
            }
            const auto before=referenceDepthBeforeRefinement(d,w,h,160.,170.,159.5,119.5,cfg);
            const auto after=conservativeDepth(d,w,h,160.,170.,159.5,119.5,cfg);
            if(!exactlyEqual(before,after))throw std::runtime_error("benchmark results differ");
            images.push_back(std::move(d));
        }
        auto measure=[&](bool refined) {
            const auto start=Clock::now();
            for(int repeat=0;repeat<batches;++repeat)for(const auto& d:images) {
                auto result=refined?conservativeDepth(d,w,h,160.,170.,159.5,119.5,cfg):
                    referenceDepthBeforeRefinement(d,w,h,160.,170.,159.5,119.5,cfg);
                sink+=result[(repeat*997)%result.size()];
            }
            return std::chrono::duration<double,std::milli>(Clock::now()-start).count()/batches;
        };
        measure(false);measure(true);
        for(int round=0;round<rounds;++round) {
            double before,after;
            if(round%2==0){before=measure(false);after=measure(true);}
            else {after=measure(true);before=measure(false);}
            std::cout<<scene<<','<<radius<<','<<cfg.reserved_radial_bound<<','<<round<<','
                <<(round%2==0?"before_first":"after_first")<<','<<std::setprecision(10)<<before<<','<<after<<",1\n";
        }
    }
    std::cerr<<"benchmark checksum="<<sink<<'\n';
    return 0;
}
}

int main(int argc,char** argv) {
    if(argc==2&&std::string(argv[1])=="--benchmark")return benchmark();
    std::cerr<<"bit-identical depth comparisons="<<compareImplementations()<<'\n';
    Camera camera(32,24,90,68,10.);DepthUncertaintyConfig cfg;cfg.enabled=true;
    std::mt19937 rng(62026);std::normal_distribution<double> noise(0,1);
    int baseline_unsafe=0,robust_unsafe=0,baseline_false_free=0,robust_false_free=0,credited_false_free=0;double lost=0;
    for(int k=0;k<1000;++k) {
        const double truth=2.+6.*(k%100)/99.;
        const double sigma=cfg.sigma0+cfg.sigma_range2*truth*truth;
        const double measured=truth+sigma*noise(rng);
        DepthObservation o(camera,std::vector<double>(768,measured),Eigen::Vector3d::Zero(),fixedCameraRotation(),1.);
        auto r=conservativeObservation(o,cfg);
        baseline_unsafe+=o.depth[400]>truth;robust_unsafe+=r.depth[400]>truth;lost+=truth-r.depth[400];
        const Eigen::Vector3d physically_colliding_center(truth-.48+.005,0,0);
        baseline_false_free+=observedEnvelope(o,physically_colliding_center,.58);
        robust_false_free+=observedEnvelope(r,physically_colliding_center,.58);
        auto credited_cfg=cfg;credited_cfg.reserved_radial_bound=.03;
        credited_false_free+=observedEnvelope(conservativeObservation(o,credited_cfg),physically_colliding_center,.58);
        if(r.depth[400]>o.depth[400])throw std::runtime_error("invented depth");
    }
    DepthObservation edge(camera,std::vector<double>(768,8.),Eigen::Vector3d::Zero(),fixedCameraRotation(),1.);
    edge.depth[12*32+16]=2.;edge.depth[8*32+16]=0.;
    const auto robust=conservativeObservation(edge,cfg);
    if(robust.depth[12*32+17]>=2.||robust.depth[8*32+17]!=0.)return 2;
    // A one-native-pixel silhouette must not grow by a whole coarse pixel.
    Camera native(320,240,90,68,10.);std::vector<double> raw(320*240,8.);
    for(int y=60;y<180;++y)raw[y*320+160]=2.;
    const auto down=conservativeDepthResize(raw,320,240,32,24,10.);
    DepthObservation coarse(camera,down,Eigen::Vector3d::Zero(),fixedCameraRotation(),1.);
    const auto coarse_uncertain=conservativeObservation(coarse,cfg);
    const auto native_uncertain=conservativeDepth(raw,320,240,native.fx(),native.fy(),native.cx(),native.cy(),cfg);
    const auto resized=conservativeDepthResize(native_uncertain,320,240,32,24,10.);
    int coarse_foreground=0,native_foreground=0;
    for(std::size_t i=0;i<resized.size();++i){coarse_foreground+=coarse_uncertain.depth[i]<3.;native_foreground+=resized[i]<3.;}
    if(native_foreground>=coarse_foreground||native_foreground==0)return 4;
    std::cerr<<"foreground coarse expansion="<<coarse_foreground<<" native expansion="<<native_foreground<<'\n';
    cfg.enabled=false;if(conservativeObservation(edge,cfg).depth!=edge.depth)return 3;
    std::cout<<"samples,baseline_overestimate,uncertainty_overestimate,mean_depth_reserve_m,baseline_false_free_envelope,robust_false_free_envelope,credited_false_free_envelope\n"
             <<1000<<','<<baseline_unsafe<<','<<robust_unsafe<<','<<lost/1000<<','<<baseline_false_free<<','<<robust_false_free<<','<<credited_false_free<<'\n';
    return robust_unsafe<baseline_unsafe/10&&baseline_false_free>0&&robust_false_free==0&&credited_false_free==0?0:1;
}
