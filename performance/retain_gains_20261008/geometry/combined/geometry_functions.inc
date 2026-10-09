double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double segmentDistance(const Eigen::Vector3d& p,const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    const Eigen::Vector3d d=b-a;
    return (p-a-(d.squaredNorm()>1e-20?clip((p-a).dot(d)/d.squaredNorm(),0.,1.):0.)*d).norm();
}
bool tubeContains(const CertifiedTube& tube,const Eigen::Vector3d& p,double radius) {
    return radius<=tube.radius && segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
}
// Immutable segment geometry reused only within one recursive envelope proof.
// Query-dependent projection, clamp, distance and radius comparisons preserve
// the exact original expressions. Malformed fixed geometry uses the old path.
struct PreparedTubeSegment {
    const CertifiedTube* tube=nullptr;
    Eigen::Vector3d delta=Eigen::Vector3d::Zero();
    double squared_length=0.;
    bool cacheable=false;
};
PreparedTubeSegment prepareTubeSegment(const CertifiedTube& tube) {
    PreparedTubeSegment out;out.tube=&tube;
    if(!tube.a.allFinite()||!tube.b.allFinite()||!std::isfinite(tube.radius)||tube.radius<0.)return out;
    out.delta=tube.b-tube.a;out.squared_length=out.delta.squaredNorm();
    out.cacheable=out.delta.allFinite()&&std::isfinite(out.squared_length);return out;
}
bool preparedTubeContains(const PreparedTubeSegment& prepared,const Eigen::Vector3d& p,double radius) {
    if(!prepared.cacheable)return tubeContains(*prepared.tube,p,radius);
    const auto& tube=*prepared.tube;const auto& d=prepared.delta;
    return radius<=tube.radius &&
        (p-tube.a-(prepared.squared_length>1e-20?
            clip((p-tube.a).dot(d)/prepared.squared_length,0.,1.):0.)*d).norm()+radius<=tube.radius;
}

bool retain(const CertifiedTube& tube,const Eigen::Vector3d& center,double radius){
 const auto* it=&tube;
            bool separated=false;
            for(int axis=0;axis<3;++axis) {
                const double low=std::min(it->a[axis],it->b[axis])-it->radius;
                const double high=std::max(it->a[axis],it->b[axis])+it->radius;
                // Only discard a tube whose entire occupied capsule cannot
                // reach any recursive center. Outward error also covers the
                // seven levels of child-center additions at large coordinates.
                const double guard=64.*std::numeric_limits<double>::epsilon()*
                    (1.+std::abs(it->a[axis])+std::abs(it->b[axis])+std::abs(center[axis])+std::abs(it->radius)+radius);
                if(center[axis]+radius<low-guard||center[axis]-radius>high+guard) {separated=true;break;}
            }
return !separated;}
