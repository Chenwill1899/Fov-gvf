double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double segmentDistance(const Eigen::Vector3d& p,const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    const Eigen::Vector3d d=b-a;
    return (p-a-(d.squaredNorm()>1e-20?clip((p-a).dot(d)/d.squaredNorm(),0.,1.):0.)*d).norm();
}
bool tubeContains(const CertifiedTube& tube,const Eigen::Vector3d& p,double radius) {
    return radius<=tube.radius && segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
}
// Per-recursive-proof metadata only. It never grants coverage, changes the
// stored tube, or survives beyond this immutable envelope query.
struct RecursiveTubeBounds {
    const CertifiedTube* tube=nullptr;
    Eigen::Vector3d lower=Eigen::Vector3d::Zero(),upper=Eigen::Vector3d::Zero();
    bool filterable=false;
};
RecursiveTubeBounds recursiveTubeBounds(const CertifiedTube& tube,
    const Eigen::Vector3d& root_center,double root_radius) {
    RecursiveTubeBounds out;out.tube=&tube;
    if(!tube.a.allFinite()||!tube.b.allFinite()||!std::isfinite(tube.radius)||tube.radius<0.)return out;
    for(int axis=0;axis<3;++axis) {
        // All descendants lie inside root_center +/- root_radius. Charge
        // capsule-bound cancellation plus seven child-center additions once;
        // each subsequent cube test uses these outward-expanded bounds.
        const double guard=64.*std::numeric_limits<double>::epsilon()*
            (1.+std::abs(tube.a[axis])+std::abs(tube.b[axis])+std::abs(root_center[axis])+tube.radius+root_radius);
        out.lower[axis]=std::min(tube.a[axis],tube.b[axis])-tube.radius-guard;
        out.upper[axis]=std::max(tube.a[axis],tube.b[axis])+tube.radius+guard;
    }
    // Unbounded, malformed or overflowed metadata must retain the tube and
    // use its original exact test, rather than pruning uncertain evidence.
    out.filterable=out.lower.allFinite()&&out.upper.allFinite();return out;
}
bool tubeBoundsDisjointCube(const RecursiveTubeBounds& tube,
    const Eigen::Vector3d& lower,const Eigen::Vector3d& upper) {
    return tube.filterable&&((upper.array()<tube.lower.array()).any()||
                            (lower.array()>tube.upper.array()).any());
}
