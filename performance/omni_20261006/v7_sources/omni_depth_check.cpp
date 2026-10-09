#include "pc_gvf/omni_depth.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace pc_gvf::depth_angular;
namespace {
constexpr double pi=3.14159265358979323846;
void require(bool b,const char* message) {if(!b){std::cerr<<message<<'\n';std::exit(1);}}
Eigen::Vector3d direction(double degrees,double elevation=0.) {
    const double a=degrees*pi/180.,e=elevation*pi/180.;
    return {std::cos(e)*std::cos(a),std::cos(e)*std::sin(a),std::sin(e)};
}
std::vector<std::shared_ptr<const DepthObservation>> views(double yaw=0.,double depth=10.) {
    std::vector<std::shared_ptr<const DepthObservation>> out;
    Camera c(32,24,90.,68.,10.);
    for(int i=0;i<4;++i) {
        auto rotation=Eigen::AngleAxisd(yaw+i*pi/2,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        out.push_back(std::make_shared<DepthObservation>(c,std::vector<double>(32*24,depth),
            rotation*Eigen::Vector3d(.3,0,0),rotation*fixedCameraRotation(),1.,1,i));
    }
    return out;
}
}
int main() {
    OmniDepthConfig cfg;cfg.horizon=4.;cfg.include_atlas=true;
    const auto zero=Eigen::Vector3d::Zero().eval();
    const auto yes=[](const Eigen::Vector3d&){return true;};
    auto input=views(.173);
    // Continuous input through all camera boundaries and +/-pi; no quantization.
    for(double a=-180.;a<=180.;a+=.5) {
        const auto q=direction(a,7.13);
        const auto out=omniDepthProposal(input,zero,q,direction(a-.5),1.05,.8,cfg,yes);
        require(out.valid && (out.direction-q).norm()<1e-12,"open-space direction quantized or discontinuous");
        require(out.fresh_views==4,"did not fuse four views");
    }
    // Independent ray/sphere intersection oracle, using non-coincident camera
    // centers and a moved/translated vehicle; catches z-depth vs range errors.
    std::vector<std::pair<Eigen::Vector3d,double>> spheres;
    const Eigen::Vector3d position(.11,-.07,.03);
    for(int view=0;view<4;++view) {
        auto v=std::make_shared<DepthObservation>(*input[view]);
        for(int j=0;j<8;++j) {
            int u=(j*7+3)%32,y=(j*5+2)%24;
            double z=1.2+.13*j;v->depth[y*32+u]=z;
            Eigen::Vector3d local(z*(u-v->camera.cx())/v->camera.fx(),z*(y-v->camera.cy())/v->camera.fy(),z);
            spheres.push_back({v->origin+v->rotation*local-position,
                cfg.radius+.5*z*std::hypot(1./v->camera.fx(),1./v->camera.fy())});
        }
        input[view]=v;
    }
    auto out=omniDepthProposal(input,position,direction(179.),zero,1.05,.5,cfg,yes);
    for(std::size_t i=0;i<out.rays.size();++i) {
        double expected=cfg.horizon;
        for(const auto& sphere:spheres) {
            double b=out.rays[i].dot(sphere.first),c=sphere.first.squaredNorm()-sphere.second*sphere.second;
            if(c<=0.)expected=0.;
            else if(b>0. && b*b>=c)expected=std::min(expected,b-std::sqrt(b*b-c));
        }
        require(std::abs(out.clearance[i]-expected)<1e-7,"atlas differs from independent ray-sphere oracle");
    }
    auto reversed=input;std::reverse(reversed.begin(),reversed.end());
    auto out2=omniDepthProposal(reversed,position,direction(179.),zero,1.05,.5,cfg,yes);
    require((out.direction-out2.direction).norm()<1e-10 && out.clearance==out2.clearance,"view ordering changes fusion");
    // A certified corridor on the other side of the old 45-degree boundary
    // must be reachable without selecting the adjacent camera first.
    int checks=0;
    out=omniDepthProposal(views(),zero,direction(44.),zero,1.05,.5,cfg,[&](const Eigen::Vector3d& d){
        ++checks;return d.y()>d.x() && d.z()>=-.001 && d.z()<=.001;
    });
    require(out.valid && out.direction.y()>out.direction.x(),"cannot select across camera seam");
    require(checks==out.certificates && checks<=cfg.max_certificates,"certificate count/bound wrong");
    out=omniDepthProposal(views(),zero,direction(179.9),zero,1.05,.5,cfg,[](const Eigen::Vector3d& d){return d.y()<0. && std::abs(d.z())<.01;});
    require(out.valid && out.direction.y()<0. && out.direction.x()<-.99,"cannot wrap across longitude seam");
    out=omniDepthProposal(views(),zero,direction(0),zero,1.05,.5,cfg,[](const Eigen::Vector3d& d){
        return d.y()>.85 && d.x()>.1 && d.z()>-.1 && d.z()<.1;
    });
    require(out.valid,"certificate budget missed a wide lateral corridor");
    // A rejected center ray does not prove its adjacent four-degree corridor
    // blocked. Global budget diversification must preserve fine local search.
    out=omniDepthProposal(views(),zero,direction(0),zero,1.05,.5,cfg,[](const Eigen::Vector3d& d){
        return d.dot(direction(4.))>std::cos(.1*pi/180.);
    });
    require(out.valid,"nearby narrow certified corridor was skipped");
    out=omniDepthProposal(views(),zero,direction(0),zero,1.31,.5,cfg,yes);
    require(!out.valid && out.fresh_views==0,"stale views accepted");
    out=omniDepthProposal(views(),zero,direction(0),zero,.9,.5,cfg,yes);
    require(!out.valid,"future views accepted");
    auto missing=views();missing[0].reset();
    out=omniDepthProposal(missing,zero,direction(90),zero,1.05,.5,cfg,yes);
    require(out.valid && out.fresh_views==3,"missing front camera vetoed side view");
    auto revoked=views();for(auto& v:revoked)v->revoked=true;
    require(!omniDepthProposal(revoked,zero,direction(0),zero,1.05,.5,cfg,yes).valid,"revoked view used");
    // Actual certification, not a permissive callback: unknown space remains
    // unknown even though no obstacle endpoints were added to the histogram.
    PaperConfig pc;pc.motion.body_radius=.48;pc.motion.safety_margin=.02;
    pc.motion.rollout_margin=.02;pc.depth_uncertainty=.03;pc.uncertainty_rate=.1;
    pc.observation_timeout=.3;pc.continuous_certificates=true;
    PaperGuidance paper(pc);
    auto unknown=views(0.,0.);
    DepthObservation evidence=*unknown[0];evidence.supporting_views=unknown;
    out=omniDepthProposal(unknown,zero,direction(45.),zero,1.05,.8,cfg,[&](const Eigen::Vector3d& d){
        return paper.proposalCorridor(evidence,zero,d,.8,1.05);
    });
    require(!out.valid,"unknown depth authorized a corridor");
    require(std::none_of(out.observed.begin(),out.observed.end(),[](int v){return v!=0;}),"unknown marked observed");
    require(!paper.motionSafe(evidence,zero,zero,direction(45),1.05),"unknown motion certified");
    auto clear=views();evidence=*clear[0];evidence.supporting_views=clear;
    require(paper.setVerifiedSeed(zero,2.5),"seed failed");
    for(double a:{44.9,45.,45.1,134.9,135.1,-135.1,-134.9,-45.1,-44.9,179.9,-179.9}) {
        out=omniDepthProposal(clear,zero,direction(a),zero,1.05,1.,cfg,[&](const Eigen::Vector3d& d){
            return paper.proposalCorridor(evidence,zero,d,1.,1.05);
        });
        require(out.valid,"real corridor failed at seam");
        require(paper.motionSafe(evidence,zero,zero,.1*out.direction,1.05),"proposed seam motion not certifiable");
    }
    bool threw=false;try{cfg.azimuth_bins=0;omniDepthProposal(clear,zero,direction(0),zero,1.,1.,cfg,yes);}catch(const std::invalid_argument&){threw=true;}
    require(threw,"invalid dimensions accepted");
    std::cout<<"omni_depth_check: continuous sweep, periodic fusion oracle, cross-view proposals, poses, missing/stale/revoked/unknown and real seam certificates passed\n";
}
