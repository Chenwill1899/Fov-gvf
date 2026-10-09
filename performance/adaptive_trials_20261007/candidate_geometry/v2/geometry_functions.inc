namespace frozen{double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
double segmentDistance(const Eigen::Vector3d& p,const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
    const Eigen::Vector3d d=b-a;
    return (p-a-(d.squaredNorm()>1e-20?clip((p-a).dot(d)/d.squaredNorm(),0.,1.):0.)*d).norm();
}
bool tubeContains(const CertifiedTube& tube,const Eigen::Vector3d& p,double radius) {
    return radius<=tube.radius && segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
}
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
