#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace pc_gvf::depth_angular;
namespace {
void require(bool value,const char* message) {
    if(!value){std::cerr<<message<<'\n';std::exit(1);}
}
PaperConfig config() {
    PaperConfig c;c.motion.body_radius=.48;c.motion.safety_margin=.02;
    c.motion.rollout_margin=.02;c.depth_uncertainty=.03;
    c.uncertainty_rate=.1;c.observation_timeout=.3;
    return c;
}
}
int main() {
    const Camera camera(8,6,90.,68.,10.);
    const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
    for(const Eigen::Vector3d center : {Eigen::Vector3d(.37,-.28,1.5),Eigen::Vector3d(-2.2,1.3,.9)}) {
        DepthObservation unknown(camera,std::vector<double>(48,0.),center,Eigen::Matrix3d::Identity(),1.);
        PaperGuidance empty(config());const double radius=empty.envelopeRadius(),offset=.4;
        require(std::abs(radius-.58)<1e-12,"test must use the unchanged complete vehicle envelope");
        require(!empty.envelopeKnown(unknown,center,radius,1.),"unknown body authorized");
        require(!empty.motionSafe(unknown,center,zero,zero,1.),"unknown stationary footprint authorized");
        std::vector<Eigen::Vector3d> centers;
        for(int axis=0;axis<3;++axis)for(int sign:{-1,1}) {
            Eigen::Vector3d p=center;p[axis]+=sign*offset;centers.push_back(p);
        }
        // For every point of norm s <= R, its nearest axial center has
        // squared distance <= s^2+d^2-2*d*s/sqrt(3). This is convex in s;
        // its maximum is attained at s=0 or s=R. The following union thus
        // covers the WHOLE ball analytically, not merely sampled surfaces.
        const double critical=std::max(offset,std::sqrt(radius*radius+offset*offset-
            2.*radius*offset/std::sqrt(3.)));
        PaperGuidance covered(config());
        require(covered.setVerifiedNeighborhood(centers,critical+.008),"seed premise failed");
        require(covered.envelopeKnown(unknown,center,radius,1.),
            "analytically complete six-ball union rejected by shallow enclosing-box proof");
        // A two-millimetre gap at a diagonal is genuinely unobserved. Extra
        // subdivision must never convert it into free space.
        PaperGuidance hole(config());
        require(hole.setVerifiedNeighborhood(centers,critical-.002),"hole seed premise failed");
        const Eigen::Vector3d witness=center+radius/std::sqrt(3.)*Eigen::Vector3d::Ones();
        for(const auto& c:centers)require((witness-c).norm()>critical-.002,"invalid analytical hole witness");
        require(!hole.envelopeKnown(unknown,witness,0.,1.),"uncovered witness accepted");
        require(!hole.envelopeKnown(unknown,center,radius,1.),"thin unknown gap bridged");
        require(!hole.motionSafe(unknown,center,zero,.1*Eigen::Vector3d::UnitX(),1.),"motion through an unknown body authorized");
        PaperGuidance missing(config());
        centers.erase(centers.begin()+1); // Remove the +X cover.
        require(missing.setVerifiedNeighborhood(centers,critical+.008),"missing-sector premise failed");
        require(!missing.envelopeKnown(unknown,center,radius,1.),"missing sector filled with invented evidence");
    }
    std::cout<<"union_refinement_check: analytical whole-ball cover accepted; thin hole, missing sector and unknown motion rejected\n";
}
