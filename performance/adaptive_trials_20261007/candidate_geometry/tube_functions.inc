namespace frozen {
double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double segmentDistance(const Eigen::Vector3d& p,const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    const Eigen::Vector3d d=b-a;
    return (p-a-(d.squaredNorm()>1e-20?clip((p-a).dot(d)/d.squaredNorm(),0.,1.):0.)*d).norm();
}
bool tubeContains(const CertifiedTube& tube,const Eigen::Vector3d& p,double radius) {
    return radius<=tube.radius && segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
}
}
namespace candidate {
double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double segmentDistance(const Eigen::Vector3d& p,const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    const Eigen::Vector3d d=b-a;
    return (p-a-(d.squaredNorm()>1e-20?clip((p-a).dot(d)/d.squaredNorm(),0.,1.):0.)*d).norm();
}
bool tubeContains(const CertifiedTube& tube,const Eigen::Vector3d& p,double radius) {
    if(!(radius<=tube.radius))return false;
    // Containment implies that the query center lies inside the center-line
    // AABB expanded by the remaining radius. This only rejects impossible
    // tubes; all overlapping and numerically ambiguous cases keep the exact
    // historical projection/norm test, with identical evidence ordering.
    const double margin=tube.radius-radius;
    for(int axis=0;axis<3;++axis) {
        const double low=std::min(tube.a[axis],tube.b[axis]);
        const double high=std::max(tube.a[axis],tube.b[axis]);
        // Charge coordinate cancellation and bound construction outwards.
        // Overflow or NaN makes these comparisons false and uses the old test.
        const double guard=32.*std::numeric_limits<double>::epsilon()*
            (1.+std::abs(low)+std::abs(high)+std::abs(p[axis])+std::abs(tube.radius)+std::abs(radius));
        if(p[axis]<low-margin-guard||p[axis]>high+margin+guard)return false;
    }
    return segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
}
}
