#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/IterativeLinearSolvers>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#ifdef _OPENMP
#include <omp.h>
#endif
namespace pc_gvf { namespace depth_angular {
namespace {
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
int idx(int x, int y, int width) { return y * width + x; }
double clip(double x, double a, double b) { return std::max(a, std::min(b, x)); }
Eigen::Vector2d local(const AngularField& f, const Eigen::Vector2d& y) { return (y-f.offset)/f.spacing; }
// Certify a query ball against two already certified balls. Their radical
// plane partitions space by the smaller squared-distance-minus-radius^2.
// On each half of the query ball, maximize that ball's squared distance exactly.
// This avoids an octree box protruding outside a thin, but genuinely covered,
// overlap between consecutive measured footprints. It adds no new evidence.
bool twoBallUnionContains(const Eigen::Vector3d& center,double radius,
    const std::pair<Eigen::Vector3d,double>& a,
    const std::pair<Eigen::Vector3d,double>& b) {
    // A line perpendicular to the center axis has concentric intersection
    // intervals, so two smaller radii cannot cover a larger query ball.
    if(radius>std::max(a.second,b.second))return false;
    const Eigen::Vector3d da=center-a.first,db=center-b.first;
    const double a2=da.squaredNorm(),b2=db.squaredNorm();
    if(a2>a.second*a.second && b2>b.second*b.second)return false;
    const Eigen::Vector3d axis=b.first-a.first;
    const double distance=axis.norm();
    if(distance<1e-12)return false;
    const Eigen::Vector3d normal=axis/distance;
    const double split=(b2-a2+a.second*a.second-b.second*b.second)/(2*distance);
    // Whole-ball containment was checked by the caller. Only a plane cutting
    // through the query can improve on that test.
    if(split<=-radius || split>=radius)return false;
    auto cap_maximum=[&](const Eigen::Vector3d& offset,const Eigen::Vector3d& n,double bound) {
        const double length=offset.norm(),parallel=offset.dot(n);
        if(length<1e-12)return radius*radius;
        if(radius*parallel/length<=bound)return (length+radius)*(length+radius);
        const double perpendicular=(offset-parallel*n).norm();
        const double linear=parallel*bound+perpendicular*std::sqrt(std::max(0.0,radius*radius-bound*bound));
        return offset.squaredNorm()+radius*radius+2*linear;
    };
    // Roundoff is charged against the proof, never used to enlarge it.
    return cap_maximum(da,normal,split)<=a.second*a.second-1e-12 &&
        cap_maximum(db,-normal,-split)<=b.second*b.second-1e-12;
}
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
// Intersect a parameterized center line with an eroded convex certificate.
// Union of the resulting closed intervals is a sufficient proof for the
// complete swept body, including joints between consecutive capsules.
using Interval=std::pair<double,double>;
bool quadraticInterval(double a,double b,double c,Interval* range) {
    if(a<1e-20) {
        if(std::abs(b)<1e-20)return c<=0;
        const double root=-c/b;
        if(b>0)range->second=std::min(range->second,root);
        else range->first=std::max(range->first,root);
    } else {
        const double discriminant=b*b-4*a*c;
        if(discriminant<0)return false;
        const double root=std::sqrt(discriminant);
        range->first=std::max(range->first,(-b-root)/(2*a));
        range->second=std::min(range->second,(-b+root)/(2*a));
    }
    return range->first<=range->second;
}
void sphereInterval(const Eigen::Vector3d& a,const Eigen::Vector3d& delta,
    const Eigen::Vector3d& center,double radius,std::vector<Interval>* intervals) {
    if(radius<=1e-9)return;
    radius-=1e-9;
    const auto offset=(a-center).eval();Interval range{0,1};
    if(quadraticInterval(delta.squaredNorm(),2*offset.dot(delta),offset.squaredNorm()-radius*radius,&range))
        intervals->push_back(range);
}
void capsuleIntervals(const Eigen::Vector3d& a,const Eigen::Vector3d& delta,
    const CertifiedTube& tube,double query_radius,std::vector<Interval>* intervals) {
    const double radius=tube.radius-query_radius-1e-9;
    if(radius<=0)return;
    const Eigen::Vector3d lo=tube.a.cwiseMin(tube.b)-Eigen::Vector3d::Constant(radius);
    const Eigen::Vector3d hi=tube.a.cwiseMax(tube.b)+Eigen::Vector3d::Constant(radius);
    if((a.cwiseMin(a+delta).array()>hi.array()).any()||
       (a.cwiseMax(a+delta).array()<lo.array()).any())return;
    sphereInterval(a,delta,tube.a,radius,intervals);
    sphereInterval(a,delta,tube.b,radius,intervals);
    const Eigen::Vector3d axis=tube.b-tube.a;const double length=axis.norm();
    if(length<1e-12)return;
    const Eigen::Vector3d n=axis/length,offset=a-tube.a;
    const double start=offset.dot(n),rate=delta.dot(n);
    const Eigen::Vector3d perpendicular=offset-start*n,change=delta-rate*n;
    Interval range{0,1};
    if(std::abs(rate)<1e-14) {if(start<0||start>length)return;}
    else {
        const double t0=-start/rate,t1=(length-start)/rate;
        range.first=std::max(range.first,std::min(t0,t1));
        range.second=std::min(range.second,std::max(t0,t1));
    }
    if(range.first<=range.second&&quadraticInterval(change.squaredNorm(),2*perpendicular.dot(change),
        perpendicular.squaredNorm()-radius*radius,&range))intervals->push_back(range);
}
// Only retain the source-connected component. Every other component is unknown.
void component(BinaryMask& mask, int w, int h, int seed) {
    if (seed < 0 || seed >= w*h || mask[seed]) { std::fill(mask.begin(),mask.end(),1); return; }
    BinaryMask seen(mask.size(),0); std::queue<int> q; q.push(seed); seen[seed]=1;
    while(!q.empty()) { int i=q.front(); q.pop(); int x=i%w,y=i/w;
        for(auto d: {Eigen::Vector2i(1,0),Eigen::Vector2i(-1,0),Eigen::Vector2i(0,1),Eigen::Vector2i(0,-1)}) {
            int u=x+d.x(),v=y+d.y(); if(u<0||v<0||u>=w||v>=h) continue;
            int j=idx(u,v,w); if(!mask[j]&&!seen[j]) {seen[j]=1;q.push(j);}
        }
    }
    for(std::size_t i=0;i<mask.size();++i) mask[i]=!seen[i];
}
// Five-point chart Laplacian; missing/blocked neighbors implement zero flux.
AngularField solve(AngularField f, const std::vector<double>& boundary, const std::vector<double>& guess = {}) {
    f.valid=false;f.iterations=0;f.incremental=false;f.residual=0.;
    int n=f.width*f.height; std::vector<int> map(n,-1); int count=0;
    bool has_fixed=false;
    for(int i=0;i<n;++i) if(!f.blocked[i]) {
        if(std::isfinite(boundary[i])) has_fixed=true; else map[i]=count++;
    }
    if(!has_fixed) return f;
    f.potential.assign(n,nan);
    std::vector<Eigen::Triplet<double>> triplets;
    Eigen::VectorXd rhs=Eigen::VectorXd::Zero(count);
    for(int i=0;i<n;++i) {
        if(f.blocked[i]) continue;
        if(map[i]<0) {f.potential[i]=boundary[i];continue;}
        int x=i%f.width,y=i/f.width,degree=0;
        for(auto d: {Eigen::Vector2i(1,0),Eigen::Vector2i(-1,0),Eigen::Vector2i(0,1),Eigen::Vector2i(0,-1)}) {
            int u=x+d.x(),v=y+d.y(); if(u<0||v<0||u>=f.width||v>=f.height) continue;
            int j=idx(u,v,f.width); if(f.blocked[j]) continue; ++degree;
            if(map[j]>=0) triplets.emplace_back(map[i],map[j],-1.0); else rhs[map[i]]+=boundary[j];
        }
        if(degree==0) return f;
        triplets.emplace_back(map[i],map[i],degree);
    }
    if(count) {
        Eigen::SparseMatrix<double> matrix(count,count); matrix.setFromTriplets(triplets.begin(),triplets.end());
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,Eigen::Lower|Eigen::Upper> solver;
        solver.setTolerance(1e-9); solver.setMaxIterations(2500); solver.compute(matrix);
        Eigen::VectorXd x=Eigen::VectorXd::Zero(count);
        f.incremental=guess.size()==static_cast<std::size_t>(n);
        if(f.incremental) {
            for(int i=0;i<n;++i)if(map[i]>=0)x[map[i]]=std::isfinite(guess[i])?clip(guess[i],0.,1.):.5;
            const Eigen::VectorXd correction_rhs=rhs-matrix*x;
            if(correction_rhs.norm()>1e-9*std::max(1.,rhs.norm())) {
                // Match the cold solve's absolute residual target. Applying
                // 1e-9 relative to an already tiny correction over-solves the
                // incremental problem without improving the field contract.
                const double target=1e-9*std::max(1.,rhs.norm());
                solver.setTolerance(std::min(1e-2,target/correction_rhs.norm()));
                const Eigen::VectorXd correction=solver.solve(correction_rhs);
                f.iterations=solver.iterations()+1;x+=correction;
            }
            if(!x.allFinite()||(matrix*x-rhs).norm()>1e-7*std::max(1.,rhs.norm())) {
                solver.setTolerance(1e-9);
                x=solver.solve(rhs);f.iterations+=solver.iterations()+1;f.incremental=false;
            }
        } else {x=solver.solve(rhs);f.iterations=solver.iterations()+1;}
        if(solver.info()!=Eigen::Success || !x.allFinite()) return f;
        f.residual=(matrix*x-rhs).norm()/std::max(1.0,rhs.norm());
        if(f.residual>1e-7) return f;
        for(int i=0;i<n;++i) if(map[i]>=0) f.potential[i]=x[map[i]];
    }
    double lo=1,hi=0;
    for(double v:f.potential) if(std::isfinite(v)) {lo=std::min(lo,v);hi=std::max(hi,v);}
    f.valid=hi-lo>1e-7;
    // Neumann ghost values are only for derivative stencils. They do not alter
    // the admissible mask, the Laplace solve, or the trajectory feasibility test.
    f.boundary_extension=f.potential;
    std::queue<int> frontier;
    for(int i=0;i<n;++i)if(!f.blocked[i])frontier.push(i);
    while(!frontier.empty()) {
        const int i=frontier.front();frontier.pop();const int x=i%f.width,y=i/f.width;
        for(auto d:{Eigen::Vector2i(1,0),Eigen::Vector2i(-1,0),Eigen::Vector2i(0,1),Eigen::Vector2i(0,-1)}) {
            int u=x+d.x(),v=y+d.y();if(u<0||v<0||u>=f.width||v>=f.height)continue;
            int j=idx(u,v,f.width);
            if(!std::isfinite(f.boundary_extension[j])) {f.boundary_extension[j]=f.boundary_extension[i];frontier.push(j);}
        }
    }
    return f;
}
bool smoothCell(const AngularField& f, const Eigen::Vector2d& y) {
    if(!f.contains(y))return false;
    const auto& values=f.boundary_extension.empty()?f.potential:f.boundary_extension;
    const Eigen::Vector2d p=local(f,y);
    const int x=static_cast<int>(std::floor(p.x())),v=static_cast<int>(std::floor(p.y()));
    if(x<1||v<1||x+2>=f.width||v+2>=f.height)return false;
    for(int j=-1;j<=2;++j)for(int i=-1;i<=2;++i)
        if(!std::isfinite(values[idx(x+i,v+j,f.width)]))return false;
    return true;
}
// Quintic Hermite interpolation with shared centered first/second derivatives.
// Adjacent patches share value, gradient and Hessian: phi is C2 and u is C1
// on the interior region with complete valid stencils.
void smoothWeights(double t, double* w, double* d) {
    const double t2=t*t,t3=t2*t,t4=t3*t,t5=t4*t;
    const double h00=1-10*t3+15*t4-6*t5, h10=t-6*t3+8*t4-3*t5;
    const double h20=.5*(t2-3*t3+3*t4-t5), h01=10*t3-15*t4+6*t5;
    const double h11=-4*t3+7*t4-3*t5, h21=.5*(t3-2*t4+t5);
    const double a00=-30*t2+60*t3-30*t4, a10=1-18*t2+32*t3-15*t4;
    const double a20=t-4.5*t2+6*t3-2.5*t4, a01=30*t2-60*t3+30*t4;
    const double a11=-12*t2+28*t3-15*t4, a21=1.5*t2-4*t3+2.5*t4;
    w[0]=-.5*h10+h20; w[1]=h00-2*h20-.5*h11+h21;
    w[2]=.5*h10+h20+h01-2*h21; w[3]=.5*h11+h21;
    d[0]=-.5*a10+a20; d[1]=a00-2*a20-.5*a11+a21;
    d[2]=.5*a10+a20+a01-2*a21; d[3]=.5*a11+a21;
}
// One interpolant defines both phi and -grad(phi). Bilinear fallback is for
// source/target handling and cannot initialize or traverse a regular flow box.
bool sampleField(const AngularField& f,const Eigen::Vector2d& y,double* value,Eigen::Vector2d* gradient) {
    if(!f.contains(y)||f.potential.empty())return false;
    Eigen::Vector2d p=local(f,y);int x=std::min(f.width-2,static_cast<int>(p.x())),v=std::min(f.height-2,static_cast<int>(p.y()));
    double tx=p.x()-x,ty=p.y()-v;*value=0;gradient->setZero();
    if(smoothCell(f,y)) {
        double wx[4],wy[4],dx[4],dy[4];smoothWeights(tx,wx,dx);smoothWeights(ty,wy,dy);
        for(int j=0;j<4;++j)for(int i=0;i<4;++i){double a=(f.boundary_extension.empty()?f.potential:f.boundary_extension)[idx(x+i-1,v+j-1,f.width)];
            *value+=wx[i]*wy[j]*a;gradient->x()+=dx[i]*wy[j]*a;gradient->y()+=wx[i]*dy[j]*a;}
    } else {
        for(int j=0;j<2;++j)for(int i=0;i<2;++i){int k=idx(x+i,v+j,f.width);
            if(f.blocked[k]||!std::isfinite(f.potential[k]))return false;
            double a=f.potential[k];*value+=(i?tx:1-tx)*(j?ty:1-ty)*a;
            gradient->x()+=(i?1:-1)*(j?ty:1-ty)*a;gradient->y()+=(i?tx:1-tx)*(j?1:-1)*a;}
    }
    *gradient/=f.spacing;return std::isfinite(*value)&&gradient->allFinite();
}
bool traceSection(const AngularField& f, const Eigen::Vector2d& y, double target,
    const PaperConfig& cfg, Eigen::Vector2d* intersection) {
    if(!f.contains(y)) return false;
    Eigen::Vector2d p=y; double traveled=0;
    for(int k=0;k<400;++k) {
        double v=f.value(p),error=v-target;
        if(!std::isfinite(v)||!smoothCell(f,p)) return false;
        if(std::abs(error)<1e-7) {*intersection=p;return true;}
        Eigen::Vector2d u=f.flow(p); double g=u.norm();
        if(g<cfg.minimum_gradient || traveled>cfg.max_return_arc) return false;
        double sign=error>0?1.0:-1.0;
        double h=std::min(0.20,std::abs(error)/g);
        Eigen::Vector2d mid=p+sign*0.5*h*u/g;
        Eigen::Vector2d um=f.flow(mid);
        if(um.norm()<cfg.minimum_gradient) return false;
        Eigen::Vector2d next=p+sign*h*um.normalized();
        if(!smoothCell(f,mid)||!smoothCell(f,next)||!f.segmentFree(p,next)) return false;
        double nv=f.value(next);
        if(!std::isfinite(nv)) return false;
        if((nv-target)*error<=0) {
            *intersection=p+(next-p)*(error/(v-nv)); return true;
        }
        traveled+=(next-p).norm();p=next;
    }
    return false;
}
std::vector<Eigen::Vector3d> referenceCurve(const AngularField& field,
    const Eigen::Vector2d& anchor,const DepthObservation& observation,double minimum_gradient) {
    std::vector<Eigen::Vector3d> result;
    for(double sign:{-1.0,1.0}) {
        std::vector<Eigen::Vector3d> half;Eigen::Vector2d p=anchor;
        for(int k=0;k<320;++k) {
            half.push_back(observation.rotation*observation.camera.rayFromPixel(p));
            const Eigen::Vector2d u=field.flow(p);if(u.norm()<minimum_gradient)break;
            const Eigen::Vector2d middle=p+sign*.0625*u.normalized();
            const Eigen::Vector2d um=field.flow(middle);if(um.norm()<minimum_gradient)break;
            const Eigen::Vector2d next=p+sign*.125*um.normalized();
            if(!field.segmentFree(p,next))break;
            p=next;
        }
        if(sign<0){std::reverse(half.begin(),half.end());result=std::move(half);}
        else if(half.size()>1)result.insert(result.end(),half.begin()+1,half.end());
    }
    return result;
}

}
std::vector<double> conservativeDepthResize(const std::vector<double>& in,int w,int h,int ow,int oh,double maxd,bool positive_infinity_is_no_return) {
    if(w<=0||h<=0||ow<=0||oh<=0||in.size()!=static_cast<std::size_t>(w*h)) throw std::invalid_argument("depth resize shape");
    std::vector<double> out(ow*oh,0);
    for(int y=0;y<oh;++y) for(int x=0;x<ow;++x) {
        double nearest=maxd; bool valid=true;
        for(int v=y*h/oh;v<std::min(h,((y+1)*h+oh-1)/oh);++v)
            for(int u=x*w/ow;u<std::min(w,((x+1)*w+ow-1)/ow);++u) {
                double d=in[idx(u,v,w)];
                if(positive_infinity_is_no_return && std::isinf(d) && d>0)d=maxd;
                if(!std::isfinite(d)||d<=0) valid=false; else nearest=std::min(nearest,d);
            }
        out[idx(x,y,ow)]=valid?nearest:0.0;
    }
    return out;
}
bool AngularField::contains(const Eigen::Vector2d& y) const {
    Eigen::Vector2d p=local(*this,y); if(!p.allFinite()||p.x()<0||p.y()<0||p.x()>width-1||p.y()>height-1) return false;
    int x=static_cast<int>(std::round(p.x())),v=static_cast<int>(std::round(p.y()));
    return !blocked[idx(x,v,width)];
}
bool AngularField::segmentFree(const Eigen::Vector2d& a,const Eigen::Vector2d& b) const {
    if(!a.allFinite()||!b.allFinite()||!contains(a)||!contains(b))return false;
    const Eigen::Vector2d p=local(*this,a),q=local(*this,b),delta=q-p;
    std::vector<double> cuts{0.0,1.0};
    for(int axis=0;axis<2;++axis) {
        if(std::abs(delta[axis])<1e-12)continue;
        const int size=axis==0?width:height;
        for(int k=0;k<size-1;++k) {
            double t=(k+.5-p[axis])/delta[axis];
            if(t>0&&t<1)cuts.push_back(t);
        }
    }
    std::sort(cuts.begin(),cuts.end());
    auto free_at=[&](double t) {
        const Eigen::Vector2d z=p+t*delta;
        int x0=static_cast<int>(std::floor(z.x()+.5-1e-10)),x1=static_cast<int>(std::floor(z.x()+.5+1e-10));
        int y0=static_cast<int>(std::floor(z.y()+.5-1e-10)),y1=static_cast<int>(std::floor(z.y()+.5+1e-10));
        for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x)
            if(x<0||y<0||x>=width||y>=height||blocked[idx(x,y,width)])return false;
        return true;
    };
    for(std::size_t i=0;i<cuts.size();++i) {
        if(!free_at(cuts[i]))return false;
        if(i>0&&!free_at(.5*(cuts[i-1]+cuts[i])))return false;
    }
    return true;
}

double AngularField::value(const Eigen::Vector2d& y) const {
    double result;Eigen::Vector2d gradient;
    return sampleField(*this,y,&result,&gradient)?result:nan;
}
Eigen::Vector2d AngularField::flow(const Eigen::Vector2d& y) const {
    double result;Eigen::Vector2d gradient;
    return sampleField(*this,y,&result,&gradient)?Eigen::Vector2d(-gradient):Eigen::Vector2d::Zero();
}
bool FlowBox::initialize(const AngularField& f,const Eigen::Vector2d& a,const PaperConfig& cfg) {
    anchor=a;section_value=f.value(a);Eigen::Vector2d u=f.flow(a);
    valid=smoothCell(f,a)&&f.segmentFree(a,a)&&std::isfinite(section_value)&&u.norm()>=cfg.minimum_gradient;
    if(valid) tangent=Eigen::Vector2d(-u.y(),u.x()).normalized();
    return valid;
}
bool FlowBox::coordinate(const AngularField& f,const Eigen::Vector2d& y,const PaperConfig& cfg,double* chi) const {
    if(!valid) return false;
    Eigen::Vector2d intersection;
    if(!traceSection(f,y,section_value,cfg,&intersection)) return false;
    if((intersection-anchor).norm()>cfg.section_radius) return false;
    *chi=tangent.dot(intersection-anchor);return std::isfinite(*chi);
}
bool FlowBox::evaluate(const AngularField& f,const Eigen::Vector2d& y,const PaperConfig& cfg,double* chi,Eigen::Vector2d* j) const {
    if(!smoothCell(f,y)||!coordinate(f,y,cfg,chi)) return false;
    // At the anchor, chi=0 and Dchi is exactly the unit section tangent:
    // Dchi annihilates the flow and is identity along the section. A fixed
    // +/-0.04 finite-difference stencil may leave a narrow but OPEN free
    // neighborhood and falsely reject this regular initialization.
    if((y-anchor).norm()<1e-10 && f.segmentFree(y,y)) {
        *chi=0;*j=tangent;
        const Eigen::Vector2d u=f.flow(y);
        return j->allFinite()&&u.norm()>=cfg.minimum_gradient&&
            std::abs(j->dot(u.normalized()))<1e-8;
    }
    const double eps=0.04;
    for(int axis=0;axis<2;++axis) {Eigen::Vector2d d=Eigen::Vector2d::Zero();d[axis]=eps;double a,b;
        if(!coordinate(f,y+d,cfg,&a)||!coordinate(f,y-d,cfg,&b)) return false;
        (*j)[axis]=(a-b)/(2*eps);
    }
    Eigen::Vector2d u=f.flow(y);
    return j->allFinite()&&j->norm()>0.05&&j->norm()<20.0&&u.norm()>cfg.minimum_gradient
        && std::abs(j->dot(u.normalized()))<0.08*j->norm();
}
namespace {
// Squared Euclidean distance to a pixel's unknown truncated cone:
// z >= depth, ax*z <= x <= bx*z, ay*z <= y <= by*z.
// At fixed z, x/y are clamped independently. The convex objective is piecewise
// quadratic; enumerating its nine active sets includes its exact minimum.
double unknownColumnDistance2(const Eigen::Vector3d& p,double ax,double bx,
    double ay,double by,double depth) {
    double best=std::numeric_limits<double>::infinity();
    for(int ix=-1;ix<=1;++ix)for(int iy=-1;iy<=1;++iy) {
        const double sx=ix<0?ax:(ix>0?bx:0.0);
        const double sy=iy<0?ay:(iy>0?by:0.0);
        const double z=std::max(depth,(p.z()+sx*p.x()+sy*p.y())/(1+sx*sx+sy*sy));
        const double x=clip(p.x(),ax*z,bx*z),y=clip(p.y(),ay*z,by*z);
        const double value=(p-Eigen::Vector3d(x,y,z)).squaredNorm();
        best=std::min(best,value);
    }
    return best;
}
bool sphereObserved(const DepthObservation& o,const Eigen::Vector3d& center,double radius) {
    if(!center.allFinite()||!std::isfinite(radius)||radius<0||
        o.depth.size()!=o.camera.rays().size()||!o.origin.allFinite()||!o.rotation.allFinite())return false;
    const Eigen::Vector3d p=o.rotation.transpose()*(center-o.origin);
    if(!p.allFinite()||p.z()<=radius)return false;
    const double ax=(-.5-o.camera.cx())/o.camera.fx();
    const double bx=(o.camera.width()-.5-o.camera.cx())/o.camera.fx();
    const double ay=(-.5-o.camera.cy())/o.camera.fy();
    const double by=(o.camera.height()-.5-o.camera.cy())/o.camera.fy();
    if(p.x()-ax*p.z()<=radius*std::hypot(1.0,ax)||
       bx*p.z()-p.x()<=radius*std::hypot(1.0,bx)||
       p.y()-ay*p.z()<=radius*std::hypot(1.0,ay)||
       by*p.z()-p.y()<=radius*std::hypot(1.0,by))return false;
    // Exact tangents of the projected sphere bound the relevant pixel cells.
    const double denominator=p.z()*p.z()-radius*radius;
    const double sx=radius*std::sqrt(p.x()*p.x()+denominator);
    const double sy=radius*std::sqrt(p.y()*p.y()+denominator);
    int x0=std::max(0,static_cast<int>(std::floor(o.camera.fx()*(p.x()*p.z()-sx)/denominator+o.camera.cx()+.5)));
    int x1=std::min(o.camera.width()-1,static_cast<int>(std::floor(o.camera.fx()*(p.x()*p.z()+sx)/denominator+o.camera.cx()+.5)));
    int y0=std::max(0,static_cast<int>(std::floor(o.camera.fy()*(p.y()*p.z()-sy)/denominator+o.camera.cy()+.5)));
    int y1=std::min(o.camera.height()-1,static_cast<int>(std::floor(o.camera.fy()*(p.y()*p.z()+sy)/denominator+o.camera.cy()+.5)));
    for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
        const double measured=o.depth[idx(x,y,o.camera.width())];
        const double depth=std::isfinite(measured)&&measured>0?measured:0.0;
        if(depth>p.z()+radius)continue;
        const double left=(x-.5-o.camera.cx())/o.camera.fx(),right=(x+.5-o.camera.cx())/o.camera.fx();
        const double top=(y-.5-o.camera.cy())/o.camera.fy(),bottom=(y+.5-o.camera.cy())/o.camera.fy();
        if(unknownColumnDistance2(p,left,right,top,bottom,depth)<=radius*radius+1e-12)return false;
    }
    return true;
}

}
AngularField solveHarmonicField(AngularField field,const std::vector<double>& boundary,const std::vector<double>& guess) {
    if(field.width<1||field.height<1||field.blocked.size()!=static_cast<std::size_t>(field.width*field.height)||boundary.size()!=field.blocked.size())
        throw std::invalid_argument("invalid harmonic domain");
    return solve(std::move(field),boundary,guess);
}
std::vector<double> reprojectPotential(const AngularField& previous,const DepthObservation& old,
    const AngularField& next,const DepthObservation& current,double range) {
    if(!previous.valid||!std::isfinite(range)||range<=0)return {};
    std::vector<double> guess(next.width*next.height,.5);
    // An identity SE(3) change on the same chart needs no interpolation.
    // This is only a numerical initial guess: solve() still rebuilds the
    // current mask, boundary conditions and checks the new residual.
    if(previous.width==next.width&&previous.height==next.height&&
       previous.spacing==next.spacing&&(previous.offset.array()==next.offset.array()).all()&&
       old.camera.width()==current.camera.width()&&old.camera.height()==current.camera.height()&&
       old.camera.fx()==current.camera.fx()&&old.camera.fy()==current.camera.fy()&&
       old.camera.cx()==current.camera.cx()&&old.camera.cy()==current.camera.cy()&&
       (old.origin.array()==current.origin.array()).all()&&
       (old.rotation.array()==current.rotation.array()).all()&&
       previous.potential.size()==guess.size()&&previous.blocked.size()==guess.size()) {
        for(std::size_t i=0;i<guess.size();++i)
            if(!previous.blocked[i]&&std::isfinite(previous.potential[i]))guess[i]=previous.potential[i];
        return guess;
    }
    for(int y=0;y<next.height;++y)for(int x=0;x<next.width;++x) {
        const Eigen::Vector2d uv=next.offset+next.spacing*Eigen::Vector2d(x,y);
        const Eigen::Vector3d world=current.origin+range*current.rotation*current.camera.rayFromPixel(uv);
        Eigen::Vector2d prior;
        if(old.camera.pixelFromDirection(old.rotation.transpose()*(world-old.origin),&prior)&&previous.contains(prior)) {
            const double phi=previous.value(prior);if(std::isfinite(phi))guess[y*next.width+x]=phi;
        }
    }
    return guess;
}
bool observedEnvelope(const DepthObservation& o,const Eigen::Vector3d& center,double radius) {
    return sphereObserved(o,center,radius);
}
Camera intentChartCamera(const Camera& physical) {
    // A virtual angular domain is not a physical camera. Retain front/lateral
    // candidates when q is blocked; every ray still needs real free-space proof.
    return Camera(physical.width(),physical.height(),140.,140.,physical.maxDepth());
}
Eigen::Matrix3d intentChartRotation(const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent) {
    const Eigen::Vector3d desired=intent.normalized();
    Eigen::Vector3d forward=desired;
    // The chart follows the measured state while moving. Biasing its center
    // toward q distorts the finite source/goal geometry on every refresh.
    // At rest q alone selects the chart; there is no blind rearward search.
    if(velocity.norm()>.03)forward=velocity.normalized();
    const Eigen::Vector3d up=std::abs(forward.z())<.95?Eigen::Vector3d::UnitZ():Eigen::Vector3d::UnitY();
    Eigen::Matrix3d chart;chart.col(2)=forward;chart.col(0)=forward.cross(up).normalized();
    chart.col(1)=forward.cross(chart.col(0));return chart;
}
PaperGuidance::PaperGuidance(const PaperConfig& config) : cfg_(config) {
    if(cfg_.dynamic_obstacles) {
        cfg_.retain_verified_travel=false;cfg_.retain_certified_volume=false;
        cfg_.history_duration=std::min(.5,cfg_.history_duration);
        // Four cameras need several recent poses inside the 0.5s horizon.
        // Static Cloud's 1s ingress cadence otherwise expires before refresh.
        cfg_.history_sample_interval=std::min(.1,cfg_.history_sample_interval);
        if(cfg_.history_max_observations>=4)cfg_.history_max_observations=std::max(32,cfg_.history_max_observations);
        cfg_.history_uncertainty_rate=std::max(.35,cfg_.history_uncertainty_rate);
    }
    const double positive[] = {cfg_.field_interval, cfg_.observation_timeout,
        cfg_.max_speed, cfg_.transverse_gain, cfg_.chart_speed, cfg_.minimum_lookahead,
        cfg_.minimum_gradient, cfg_.section_radius, cfg_.max_return_arc,
        cfg_.motion.brake_accel, cfg_.motion.max_accel, cfg_.motion.velocity_tau,cfg_.proposal_response_time,cfg_.certificate_resolution};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("paper parameter must be finite and positive");
    const double nonnegative[] = {cfg_.motion.body_radius,cfg_.motion.safety_margin,
        cfg_.history_sample_interval,cfg_.history_uncertainty_rate,cfg_.history_duration,cfg_.motion.rollout_margin,cfg_.motion.delay,cfg_.motion.rollout_horizon,
        cfg_.motion.planning_horizon,cfg_.depth_uncertainty,cfg_.uncertainty_rate,cfg_.restart_clearance,cfg_.proposal_command_accel};
    for(double value:nonnegative)
        if(!std::isfinite(value)||value<0)throw std::invalid_argument("paper margin must be finite and nonnegative");
    if (!std::isfinite(cfg_.command_change_angle) || cfg_.command_change_angle < 0 ||
        cfg_.command_change_angle >= std::acos(-1.0))
        throw std::invalid_argument("invalid paper command-change angle");
    if (cfg_.fine_width < 4 || cfg_.fine_height < 4 || cfg_.coarse_factor < 1 ||
        !std::isfinite(cfg_.max_vertical_speed) || cfg_.max_vertical_speed < 0)
        throw std::invalid_argument("invalid paper grid or vertical limit");
    if(cfg_.proposal_feedforward&&cfg_.model_velocity_shaping)
        throw std::invalid_argument("select one proposal response model");
    if(cfg_.history_max_observations<4||cfg_.history_max_observations>256)
        throw std::invalid_argument("history capacity must be between 4 and 256");
    if(!std::isfinite(cfg_.proposal_jerk_limit)||cfg_.proposal_jerk_limit<=0)
        throw std::invalid_argument("invalid proposal jerk limit");
}
void PaperGuidance::reset() {
    diagnostic_valid_=false; target_holding_=false; observation_.reset(); field_=AngularField(); box_=FlowBox(); reference_world_.clear();
    proposal_reference_valid_=false;proposal_reference_.setZero();
    previous_command_.setZero(); proposal_acceleration_.setZero(); previous_intent_.setZero(); reference_intent_.setZero(); last_update_=-1;
}
void PaperGuidance::confirmPublishedCommand(const Eigen::Vector3d& command) {
    if(!command.allFinite()||(command-previous_command_).norm()>1e-9) {
        proposal_acceleration_.setZero();proposal_reference_valid_=false;
    }
    previous_command_=command.allFinite()?command:Eigen::Vector3d::Zero();
    if(previous_command_.norm()<1e-8)diagnostic_valid_=false;
}
void PaperGuidance::ingestObservation(const DepthObservation& observation) {
    if(cfg_.dynamic_obstacles||cfg_.shared_obstacles)dynamic_.ingest(observation);
    if(cfg_.spherical_memory)spherical_.ingest(observation);
    ingestion_cost_ms_.fill(0.);
    auto phase_start=std::chrono::steady_clock::now();
    auto phase=[&](int i){auto end=std::chrono::steady_clock::now();
        ingestion_cost_ms_[i]=std::chrono::duration<double,std::milli>(end-phase_start).count();phase_start=end;};
    if(cfg_.history_duration<=0||!std::isfinite(observation.stamp)||
        observation.depth.size()!=observation.camera.rays().size()||
        !observation.origin.allFinite()||!observation.rotation.allFinite())return;
    const auto previous_view=checked_observations_.find(observation.view);
    if(previous_view!=checked_observations_.end()&&observation.stamp<previous_view->second.first-.1) {
        history_.clear();checked_observations_.clear();last_retained_stamp_.clear();seeds_.clear();verified_travel_.clear();pending_motion_.clear();verified_tubes_.clear();reached_proofs_.clear();last_reached_time_=-1;reset();
    }
    const auto identity=std::make_pair(observation.stamp,observation.version);
    const auto checked=checked_observations_.find(observation.view);
    if(checked!=checked_observations_.end() && checked->second==identity)return;
    checked_observations_[observation.view]=identity;
    const auto history_before_expiration=history_.size();
    history_.erase(std::remove_if(history_.begin(),history_.end(),[&](const auto& old) {
        // Static certified-space mode already retains reached volumes without
        // a TTL. Preserve their depth provenance under the same assumption;
        // bounded coverage eviction and fresh-hit contradiction still apply.
        return !(cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.) &&
            observation.stamp-old->stamp>cfg_.history_duration;
    }),history_.end());
    expired_views_+=history_before_expiration-history_.size();
    const auto last_retained=last_retained_stamp_.find(observation.view);
    bool retain=last_retained==last_retained_stamp_.end()||
        observation.stamp-last_retained->second>=cfg_.history_sample_interval;
    for(auto it=history_.rbegin();it!=history_.rend();++it)
        if((*it)->view==observation.view) {
            if(observation.stamp-(*it)->stamp<cfg_.history_sample_interval)retain=false;
            if(cfg_.retain_verified_travel && (observation.origin-(*it)->origin).norm()<.25 &&
                (observation.rotation-(*it)->rotation).norm()<.15)retain=false;
            break;
        }
    std::vector<Eigen::Vector3d> hits;
    if(observation.obstacle_points) {
        hits.reserve(observation.obstacle_points->size());
        for(const auto& point:*observation.obstacle_points)
            hits.push_back(observation.origin+observation.rotation*point);
    } else for(int y=0;y<observation.camera.height();++y)for(int x=0;x<observation.camera.width();++x) {
        const double depth=observation.depth[idx(x,y,observation.camera.width())];
        if(!std::isfinite(depth)||depth<=0||depth>=observation.camera.maxDepth()-1e-6)continue;
        const auto& ray=observation.camera.ray(x,y);
        hits.push_back(observation.origin+observation.rotation*(depth/ray.z()*ray));
    }
    // Broad-phase culling only: every native hit inside the union's AABB is
    // still tested exactly. Distant pixels cannot intersect any of these balls.
    auto remove_conflicts=[&](std::vector<std::pair<Eigen::Vector3d,double>>& balls) {
        if(balls.empty())return std::size_t(0);
        Eigen::Vector3d lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector3d hi=-lo;
        for(const auto& ball:balls) {
            lo=lo.cwiseMin(ball.first-Eigen::Vector3d::Constant(ball.second));
            hi=hi.cwiseMax(ball.first+Eigen::Vector3d::Constant(ball.second));
        }
        std::vector<Eigen::Vector3d> nearby;
        for(const auto& point:hits)
            if((point.array()>=lo.array()).all()&&(point.array()<=hi.array()).all())nearby.push_back(point);
        const auto before=balls.size();
        balls.erase(std::remove_if(balls.begin(),balls.end(),[&](const auto& ball) {
            const double radius=ball.second-cfg_.depth_uncertainty;
            if(radius<=0)return false;
            for(const auto& point:nearby)if((point-ball.first).squaredNorm()<radius*radius)return true;
            return false;
        }),balls.end());
        return before-balls.size();
    };
    revoked_balls_+=remove_conflicts(verified_travel_);
    remove_conflicts(pending_motion_);
    remove_conflicts(seeds_);
    phase(0);
    // Index certificate AABBs, then visit each native hit once. Checking every
    // image point against every stored tube caused large pre-control delays.
    struct CellHash {
        std::size_t operator()(const std::array<int,3>& c) const {
            return std::hash<int>()(c[0]) ^ (std::hash<int>()(c[1])*73856093u) ^
                (std::hash<int>()(c[2])*19349663u);
        }
    };
    std::vector<CertifiedTube> primitives=verified_tubes_;
    for(const auto& proof:reached_proofs_)primitives.push_back({proof.first,proof.first,proof.second});
    std::unordered_map<std::array<int,3>,std::vector<std::size_t>,CellHash> tube_cells;
    auto cell=[](const Eigen::Vector3d& p) {
        return std::array<int,3>{{static_cast<int>(std::floor(p.x())),
            static_cast<int>(std::floor(p.y())),static_cast<int>(std::floor(p.z()))}};
    };
    Eigen::Vector3d primitive_lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d primitive_hi=-primitive_lo;
    for(std::size_t i=0;i<primitives.size();++i) {
        const auto& tube=primitives[i];const double r=std::max(0.,tube.radius-cfg_.depth_uncertainty);
        primitive_lo=primitive_lo.cwiseMin(tube.a.cwiseMin(tube.b)-Eigen::Vector3d::Constant(r));
        primitive_hi=primitive_hi.cwiseMax(tube.a.cwiseMax(tube.b)+Eigen::Vector3d::Constant(r));
        const auto lo=cell(tube.a.cwiseMin(tube.b)-Eigen::Vector3d::Constant(r));
        const auto hi=cell(tube.a.cwiseMax(tube.b)+Eigen::Vector3d::Constant(r));
        for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y)for(int z=lo[2];z<=hi[2];++z)
            tube_cells[{{x,y,z}}].push_back(i);
    }
    std::vector<unsigned char> revoked(primitives.size(),0);
    for(const auto& point:hits) {
        if((point.array()<primitive_lo.array()).any()||(point.array()>primitive_hi.array()).any())continue;
        const auto found=tube_cells.find(cell(point));if(found==tube_cells.end())continue;
        for(const auto i:found->second) {
            const auto& tube=primitives[i];
            if(!revoked[i]&&segmentDistance(point,tube.a,tube.b)<tube.radius-cfg_.depth_uncertainty)revoked[i]=1;
        }
    }
    std::size_t index=0;
    verified_tubes_.erase(std::remove_if(verified_tubes_.begin(),verified_tubes_.end(),[&](const CertifiedTube&) {
        return revoked[index++];
    }),verified_tubes_.end());
    const auto tube_end=revoked.begin()+index;
    revoked_tubes_+=std::count(revoked.begin(),tube_end,1);
    reached_proofs_.erase(std::remove_if(reached_proofs_.begin(),reached_proofs_.end(),[&](const auto&) {
        return revoked[index++];
    }),reached_proofs_.end());
    revoked_balls_+=std::count(tube_end,revoked.end(),1);
    phase(1);
    Eigen::Vector3d hits_lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hits_hi=-hits_lo;
    for(const auto& point:hits){hits_lo=hits_lo.cwiseMin(point);hits_hi=hits_hi.cwiseMax(point);}
    const Eigen::Vector3d hits_center=hits.empty()?Eigen::Vector3d::Zero().eval():((hits_lo+hits_hi)*.5).eval();
    const Eigen::Vector3d hits_half=hits.empty()?Eigen::Vector3d::Zero().eval():((hits_hi-hits_lo)*.5).eval();
    // Native pixels are ordered by scanline. Small contiguous groups have
    // tight world AABBs; discard a group only if all its points lie outside
    // an old viewing frustum. This avoids projecting every distant pixel
    // into every historical image without dropping any possible conflict.
    struct HitGroup { std::size_t begin,end; Eigen::Vector3d center,half; };
    std::vector<HitGroup> hit_groups;
    for(std::size_t begin=0;begin<hits.size();begin+=128) {
        const std::size_t end=std::min(hits.size(),begin+128);
        // Bound in the original optical frame. Rotating a world AABB back
        // into an old camera needlessly enlarges a thin scanline group twice.
        auto native=[&](std::size_t i)->Eigen::Vector3d {
            return observation.obstacle_points?(*observation.obstacle_points)[i]:hits[i];
        };
        Eigen::Vector3d lo=native(begin),hi=lo;
        for(std::size_t i=begin+1;i<end;++i){const auto point=native(i);lo=lo.cwiseMin(point);hi=hi.cwiseMax(point);}
        hit_groups.push_back({begin,end,(lo+hi)*.5,(hi-lo)*.5});
    }
    std::vector<unsigned char> contradicted(history_.size(),0);
    // Static keyframes are spatial evidence. A local contradiction must remove
    // its whole uncertain footprint, but need not erase unrelated free columns.
    // Non-static history retains the original whole-view revocation policy.
    const bool repair_static=cfg_.local_history_repair&&cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.;
    std::vector<std::vector<double>> repaired_depth(history_.size());
#ifdef _OPENMP
#pragma omp parallel for num_threads(2) schedule(static) if(history_.size()>=8)
#endif
    for(int index=0;index<static_cast<int>(history_.size());++index) {
        const auto& old=history_[index];
        // Exact broad-phase exclusion: if the complete incoming hit AABB is
        // outside any old frustum plane, none of its points can contradict
        // that observation. Retained data and per-hit exact checks are unchanged.
        if(hits.empty())continue;
        const Eigen::Vector3d center=old->rotation.transpose()*(hits_center-old->origin);
        const Eigen::Vector3d half=old->rotation.transpose().cwiseAbs()*hits_half;
        const double ax=(-.5-old->camera.cx())/old->camera.fx();
        const double bx=(old->camera.width()-.5-old->camera.cx())/old->camera.fx();
        const double ay=(-.5-old->camera.cy())/old->camera.fy();
        const double by=(old->camera.height()-.5-old->camera.cy())/old->camera.fy();
        bool outside=center.z()+half.z()<=0||center.z()-half.z()>=old->camera.maxDepth();
        for(const Eigen::Vector3d& plane:{Eigen::Vector3d(1,0,-ax),Eigen::Vector3d(-1,0,bx),
                Eigen::Vector3d(0,1,-ay),Eigen::Vector3d(0,-1,by)})
            outside=outside||plane.dot(center)+plane.cwiseAbs().dot(half)<-1e-9;
        if(outside)continue;
        const double uncertainty=cfg_.depth_uncertainty+cfg_.history_uncertainty_rate*std::max(0.0,observation.stamp-old->stamp);
        Eigen::Matrix3d old_from_new=old->rotation.transpose();
        if(observation.obstacle_points)old_from_new=old->rotation.transpose()*observation.rotation;
        const Eigen::Vector3d old_origin=old->rotation.transpose()*
            ((observation.obstacle_points?observation.origin:Eigen::Vector3d::Zero().eval())-old->origin);
        for(const auto& group:hit_groups) {
            const Eigen::Vector3d gc=old_origin+old_from_new*group.center;
            const Eigen::Vector3d gh=old_from_new.cwiseAbs()*group.half;
            bool group_outside=gc.z()+gh.z()<=0||gc.z()-gh.z()>=old->camera.maxDepth();
            for(const Eigen::Vector3d& plane:{Eigen::Vector3d(1,0,-ax),Eigen::Vector3d(-1,0,bx),
                    Eigen::Vector3d(0,1,-ay),Eigen::Vector3d(0,-1,by)})
                group_outside=group_outside||plane.dot(gc)+plane.cwiseAbs().dot(gh)<-1e-9;
            if(group_outside)continue;
            // A small projected rectangle can prove the entire group is at
            // or behind every old depth limit it might address. This is only
            // a necessary-condition cull; unresolved groups still visit every
            // native hit. No cached addresses or sampled depth maxima are used.
            const double near_z=gc.z()-gh.z(),far_z=gc.z()+gh.z();
            if(near_z>uncertainty+1e-6) {
                auto projection=[&](double center,double half,double focal,double principal) {
                    const std::array<double,4> values{{(center-half)/near_z,(center-half)/far_z,
                        (center+half)/near_z,(center+half)/far_z}};
                    return std::pair<double,double>(focal*(*std::min_element(values.begin(),values.end()))+principal,
                        focal*(*std::max_element(values.begin(),values.end()))+principal);
                };
                const auto u=projection(gc.x(),gh.x(),old->camera.fx(),old->camera.cx());
                const auto v=projection(gc.y(),gh.y(),old->camera.fy(),old->camera.cy());
                // Clip in floating point before conversion, and include one
                // extra border cell so rounding cannot underbound the footprint.
                const int xa=std::max(0,int(std::floor(clip(u.first,-2.,old->camera.width()+1.)+.5))-1);
                const int xb=std::min(old->camera.width()-1,int(std::floor(clip(u.second,-2.,old->camera.width()+1.)+.5))+1);
                const int ya=std::max(0,int(std::floor(clip(v.first,-2.,old->camera.height()+1.)+.5))-1);
                const int yb=std::min(old->camera.height()-1,int(std::floor(clip(v.second,-2.,old->camera.height()+1.)+.5))+1);
                if(xa<=xb&&ya<=yb&&(xb-xa+1)*(yb-ya+1)<=128) {
                    bool behind=true;const double bound=near_z+uncertainty-1e-9;
                    for(int y=ya;y<=yb&&behind;++y)for(int x=xa;x<=xb;++x)
                        if(!(old->depth[idx(x,y,old->camera.width())]<=bound)){behind=false;break;}
                    if(behind)continue;
                }
            }
            for(std::size_t hit=group.begin;hit<group.end;++hit) {
            const auto& point=hits[hit];
            const Eigen::Vector3d p=old->rotation.transpose()*(point-old->origin);
            if(p.z()<=uncertainty||p.z()+uncertainty>=old->camera.maxDepth())continue;
            const int u=static_cast<int>(std::floor(old->camera.fx()*p.x()/p.z()+old->camera.cx()+.5));
            const int v=static_cast<int>(std::floor(old->camera.fy()*p.y()/p.z()+old->camera.cy()+.5));
            if(u<0||v<0||u>=old->camera.width()||v>=old->camera.height())continue;
            // Test against the original view: a previous hit may have clipped
            // this center while leaving part of the next uncertainty footprint.
            if(old->depth[idx(u,v,old->camera.width())]<=p.z()+uncertainty)continue;
            if(!observedEnvelope(*old,point,uncertainty))continue;
            contradicted[index]=1;
            if(!repair_static)break;
            if(repaired_depth[index].empty())repaired_depth[index]=old->depth;
            // Conservatively project the entire uncertain hit ball, not just
            // its central pixel. Every intersecting column ends at its nearest
            // possible z; depth is only reduced, never enlarged or fabricated.
            const double denominator=p.z()*p.z()-uncertainty*uncertainty;
            const double sx=uncertainty*std::sqrt(p.x()*p.x()+denominator);
            const double sy=uncertainty*std::sqrt(p.y()*p.y()+denominator);
            const auto bound=[](double pixel,int count) {
                return static_cast<int>(std::floor(clip(pixel,-1.,double(count))+.5));
            };
            const int x0=std::max(0,bound(old->camera.fx()*(p.x()*p.z()-sx)/denominator+old->camera.cx(),old->camera.width())-1);
            const int x1=std::min(old->camera.width()-1,bound(old->camera.fx()*(p.x()*p.z()+sx)/denominator+old->camera.cx(),old->camera.width())+1);
            const int y0=std::max(0,bound(old->camera.fy()*(p.y()*p.z()-sy)/denominator+old->camera.cy(),old->camera.height())-1);
            const int y1=std::min(old->camera.height()-1,bound(old->camera.fy()*(p.y()*p.z()+sy)/denominator+old->camera.cy(),old->camera.height())+1);
            const double limit=std::max(0.,p.z()-uncertainty-1e-9);
            for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
                auto& value=repaired_depth[index][idx(x,y,old->camera.width())];
                value=std::isfinite(value)&&value>0?std::min(value,limit):0.;
            }
            }
            if(contradicted[index]&&!repair_static)break;
        }
    }
    for(std::size_t i=0;i<history_.size();++i)if(contradicted[i]) {
        // Invalidate any existing field/cache borrowing the original view.
        // Publish a new immutable, strictly smaller observation to history.
        history_[i]->revoked=true;
        if(repair_static) {
            auto repaired=std::make_shared<DepthObservation>(*history_[i]);
            repaired->depth=std::move(repaired_depth[i]);repaired->revoked=false;
            repaired->supporting_views.clear();repaired->obstacle_points.reset();
            history_[i]=std::move(repaired);
        }
    }
    history_.erase(std::remove_if(history_.begin(),history_.end(),[](const auto& old) {
        return old->revoked;
    }),history_.end());
    phase(2);
    // Check every fresh frame for contradictions, even between retained samples.
    if(!retain)return;
    last_retained_stamp_[observation.view]=observation.stamp;
    auto retained=std::make_shared<DepthObservation>(observation);
    retained->supporting_views.clear();retained->obstacle_points.reset();
    history_.push_back(std::move(retained));
    if(history_.size()>static_cast<std::size_t>(cfg_.history_max_observations)) {
        // Preserve coverage around the current footprint and short recovery
        // corridor. Novel camera poses alone do not identify indispensable
        // near-field views. Witnesses guide eviction only; they NEVER certify
        // free space or replace the full geometric containment test.
        std::vector<Eigen::Vector3d> witnesses;
        if(last_reached_time_>=0) {
            std::vector<Eigen::Vector3d> centers{last_reached_};
            if(previous_command_.norm()>.01) {
                centers.push_back(last_reached_+.25*previous_command_.normalized());
                centers.push_back(last_reached_-.25*previous_command_.normalized());
            }
            for(const auto& center:centers)for(int x:{-1,0,1})for(int y:{-1,0,1})for(int z:{-1,0,1}) {
                Eigen::Vector3d direction(x,y,z);if(direction.norm()<.5)continue;
                witnesses.push_back(center+(envelopeRadius()+.05)*direction.normalized());
            }
        }
        std::vector<std::vector<unsigned char>> coverage(history_.size(),std::vector<unsigned char>(witnesses.size(),0));
        std::vector<int> count(witnesses.size(),0);
        for(std::size_t i=0;i<history_.size();++i)for(std::size_t k=0;k<witnesses.size();++k)
            if(observedEnvelope(*history_[i],witnesses[k],.02)) {coverage[i][k]=1;++count[k];}
        std::size_t victim=0;double best_loss=std::numeric_limits<double>::infinity(),best_redundancy=best_loss;
        for(std::size_t i=0;i+1<history_.size();++i) {
            double loss=0,redundancy=std::numeric_limits<double>::infinity();
            for(std::size_t k=0;k<witnesses.size();++k)if(coverage[i][k])loss+=count[k]==1?100.:1./count[k];
            for(std::size_t j=i+1;j<history_.size();++j)if(history_[i]->view==history_[j]->view)
                redundancy=std::min(redundancy,(history_[i]->origin-history_[j]->origin).norm()+
                    2.*(history_[i]->rotation-history_[j]->rotation).norm());
            if(loss<best_loss-1e-9||(std::abs(loss-best_loss)<1e-9&&redundancy<best_redundancy))
                {best_loss=loss;best_redundancy=redundancy;victim=i;}
        }
        history_.erase(history_.begin()+victim);++evicted_views_;
    }
    phase(3);
}
bool PaperGuidance::setVerifiedSeed(const Eigen::Vector3d& center,double radius) {
    return setVerifiedNeighborhood({center},radius);
}
bool PaperGuidance::setVerifiedNeighborhood(const std::vector<Eigen::Vector3d>& centers,double radius) {
    if(centers.empty()||!std::isfinite(radius)||radius<=0||seed_initialized_)return false;
    for(const auto& center:centers)if(!center.allFinite())return false;
    for(const auto& center:centers)seeds_.emplace_back(center,radius);
    seed_initialized_=true;return true;
}

double PaperGuidance::envelopeRadius() const {
    // Budget the maximum admitted observation age once. A saved static-space
    // certificate must not grow stale merely because the next frame is older.
    return cfg_.motion.body_radius+cfg_.motion.safety_margin+cfg_.depth_uncertainty+
        cfg_.motion.rollout_margin+cfg_.uncertainty_rate*cfg_.observation_timeout;
}
bool PaperGuidance::rememberVerifiedPosition(const DepthObservation& input,
    const Eigen::Vector3d& position,double now) {
    if(!cfg_.retain_verified_travel||!position.allFinite()||!std::isfinite(now))return false;
    // The ROS layer already records after ingesting all cameras. step() may
    // request the identical footprint at the same timestamp; avoid doing the
    // same union proof twice. Revoked balls are removed before this check.
    if(now==last_reached_time_&&!verified_travel_.empty()&&
        (position-verified_travel_.back().first).norm()<1e-9&&verified_travel_.back().second>=envelopeRadius())return true;
    DepthObservation evidence=input;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());}
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    const double radius=envelopeRadius();
    bool known=envelopeKnown(evidence,position,radius,now);
    if(!known)for(const auto& certificate:pending_motion_)
        if((position-certificate.first).norm()+radius<=certificate.second){known=true;break;}
    if(!known)return false;
    // Re-centering a proved ball at the measured pose can discard almost all
    // of its useful room. Once the full measured footprint has arrived inside
    // an independently proved motion ball, retain that ORIGINAL geometry.
    // Merely predicted/unreached certificates are still excluded from queries.
    if(cfg_.continuous_certificates) {
        const std::pair<Eigen::Vector3d,double>* best=nullptr;
        for(const auto& proof:pending_motion_)
            if((position-proof.first).norm()+radius<=proof.second && (!best||proof.second>best->second))best=&proof;
        if(best) {
            bool redundant=false;
            for(const auto& old:reached_proofs_)if((best->first-old.first).norm()+best->second<=old.second+1e-12) {redundant=true;break;}
            if(!redundant) {
                reached_proofs_.erase(std::remove_if(reached_proofs_.begin(),reached_proofs_.end(),[&](const auto& old) {
                    return (best->first-old.first).norm()+old.second<=best->second;
                }),reached_proofs_.end());
                reached_proofs_.push_back(*best);
                if(reached_proofs_.size()>128) {
                    auto farthest=std::max_element(reached_proofs_.begin(),reached_proofs_.end()-1,[&](const auto& a,const auto& b) {
                        return (a.first-position).squaredNorm()<(b.first-position).squaredNorm();
                    });reached_proofs_.erase(farthest);
                }
            }
        }
    }
    rememberTube(evidence,position,now);
    // Only measured, fully certified balls are saved. In particular, no
    // predicted endpoint and no unobserved expansion may authorize motion.
    // Keep the latest measured footprint even during release/braking. Nearby
    // updates replace the newest sample; older spatial samples remain bounded.
    // Extra already-observed room is useful but never a prerequisite to save
    // an otherwise valid footprint. It must have its own containment proof.
    double retained_radius=radius;
    // Preserve a partial margin as well: requiring all 10 cm would discard
    // previously proved room precisely where a tight corridor needs it most.
    for(const auto& certificate:pending_motion_)
        retained_radius=std::max(retained_radius,std::min(radius+.10,
            certificate.second-(position-certificate.first).norm()));
    for(const auto& previous:verified_travel_)
        retained_radius=std::max(retained_radius,std::min(radius+.10,
            previous.second-(position-previous.first).norm()));
    double high=radius+.10;
    if(envelopeKnown(evidence,position,high,now))retained_radius=high;
    else for(int k=0;k<4;++k) {
        const double middle=.5*(retained_radius+high);
        if(envelopeKnown(evidence,position,middle,now))retained_radius=middle;
        else high=middle;
    }
    const std::pair<Eigen::Vector3d,double> ball(position,retained_radius);
    if(!verified_travel_.empty()&&(position-verified_travel_.back().first).norm()<1e-9) {
        verified_travel_.back().second=std::max(verified_travel_.back().second,retained_radius);
        return true;
    }
    if(verified_travel_.size()>=2 &&
        (position-verified_travel_[verified_travel_.size()-2].first).norm()<.15)
        verified_travel_.back()=ball;
    else {
        verified_travel_.push_back(ball);
        if(verified_travel_.size()>32)verified_travel_.erase(verified_travel_.begin());
    }
    return true;
}

void PaperGuidance::rememberTube(const DepthObservation& evidence,const Eigen::Vector3d& position,double now) {
    if(!cfg_.continuous_certificates)return;
    const double distance=(position-last_reached_).norm();
    if(last_reached_time_>=0 && distance>1e-8 && distance<=.5 &&
        now>=last_reached_time_ && now-last_reached_time_<=.25) {
        // Endpoints alone do not prove a corridor. Check the complete capsule
        // using current evidence or an unrevoked accepted-motion certificate.
        double radius=envelopeRadius();
        if(segmentKnown(evidence,last_reached_,position,radius,now,true)) {
            for(double extra:{.02,.01,.005}) {
                if(segmentKnown(evidence,last_reached_,position,radius+extra,now,true)) {radius+=extra;break;}
            }
            CertifiedTube tube{last_reached_,position,radius};
            if(!verified_tubes_.empty()) {
                const auto& old=verified_tubes_.back();
                // The broken-line tube contains this chord tube after charging
                // its entire triangle height against the radius. Never bridge a gap.
                const double loss=segmentDistance(old.b,old.a,tube.b);
                const double merged_radius=std::min(old.radius,tube.radius)-loss-1e-10;
                if((old.b-tube.a).norm()<1e-9 && (tube.b-old.a).norm()<=.5 &&
                    merged_radius>=envelopeRadius()+cfg_.certificate_resolution) {
                    tube.a=old.a;tube.radius=merged_radius;verified_tubes_.pop_back();
                }
            }
            verified_tubes_.push_back(tube);
            if(verified_tubes_.size()>256) {
                // Keep local recovery coverage under a fixed memory bound.
                auto victim=std::max_element(verified_tubes_.begin(),verified_tubes_.end()-1,[&](const auto& a,const auto& b) {
                    return segmentDistance(position,a.a,a.b)<segmentDistance(position,b.a,b.b);
                });
                verified_tubes_.erase(victim);
            }
        }
    }
    // A gap is not retrospectively filled after failed certification.
    last_reached_=position;last_reached_time_=now;
}

bool PaperGuidance::envelopeKnown(const DepthObservation& o,const Eigen::Vector3d& center,
    double radius,double now) const {
    return envelopeKnownImpl(o,center,radius,now,false);
}
bool PaperGuidance::envelopeKnownImpl(const DepthObservation& o,const Eigen::Vector3d& center,
    double radius,double now,bool pending) const {
    if(!center.allFinite()||!std::isfinite(radius)||radius<0)return false;
    pending=pending||(cfg_.retain_certified_volume&&cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.);
    // A verified seed is fixed in world coordinates; it never follows the robot.
    // It is an explicit initialization premise, not an observation fabricated
    // at each control instant. Default ROS configuration does not enable it.
    auto covered=[&](const Eigen::Vector3d& p,double r) {
        if(pending)for(const auto& ball:pending_motion_)
            if((p-ball.first).norm()+r<=ball.second)return true;
        for(auto it=reached_proofs_.rbegin();it!=reached_proofs_.rend();++it)
            if((p-it->first).norm()+r<=it->second)return true;
        for(auto it=verified_tubes_.rbegin();it!=verified_tubes_.rend();++it)
            if(tubeContains(*it,p,r))return true;
        for(const auto& seed:seeds_)if((p-seed.first).norm()+r<=seed.second)return true;
        for(const auto& ball:verified_travel_)if((p-ball.first).norm()+r<=ball.second)return true;
        for(std::size_t i=1;i<verified_travel_.size();++i)
            if(twoBallUnionContains(p,r,verified_travel_[i-1],verified_travel_[i]))return true;
        for(std::size_t i=1;i<seeds_.size();++i)
            if(twoBallUnionContains(p,r,seeds_[i-1],seeds_[i]))return true;
        if(!o.revoked&&now-o.stamp<=cfg_.observation_timeout&&now>=o.stamp-.02&&observedEnvelope(o,p,r))return true;
        for(const auto& view:o.supporting_views) {
            const double age=now-view->stamp;
            const bool static_memory=cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.&&view->memory_uncertainty_rate==0.;
            const double ttl=view->memory_uncertainty_rate>0?(cfg_.dynamic_obstacles?.5:SphericalMemory::lifetime):cfg_.history_duration;
            if(view->revoked||age<-.02||(!static_memory&&age>ttl))continue;
            const double extra=cfg_.history_uncertainty_rate*std::max(0.0,o.stamp-view->stamp)+
                view->memory_uncertainty_rate*std::max(0.,age);
            if(observedEnvelope(*view,p,r+extra))return true;
        }
        return false;
    };
    if(cache_enabled_&&!pending) {
        int worker=0;
#ifdef _OPENMP
        worker=omp_get_thread_num();
#endif
        auto& envelope_cache_=envelope_caches_.at(worker);
        const auto key=std::make_tuple(static_cast<int>(std::floor(center.x()/.2)),
            static_cast<int>(std::floor(center.y()/.2)),static_cast<int>(std::floor(center.z()/.2)));
        const auto found=envelope_cache_.find(key);
        if(found!=envelope_cache_.end()&&(center-found->second.first).norm()+radius<=found->second.second)return true;
        const double expanded=radius+std::sqrt(3.0)*.2;
        if(covered(center,expanded)) {envelope_cache_[key]={center,expanded};return true;}
    }
    if(covered(center,radius))return true;
    if(!covered(center,0))return false;
    if(o.supporting_views.empty()&&seeds_.empty()&&verified_travel_.empty()&&verified_tubes_.empty()&&reached_proofs_.empty()&&!pending)return false;
    // Certify a union without assuming that one camera must see the whole body.
    // Each intersecting octree box is enclosed by a sphere and must be wholly
    // contained in actual evidence. Unresolved leaves remain unknown.
    int remaining=1024;
    std::function<bool(const Eigen::Vector3d&,double,int)> certify;
    certify=[&](const Eigen::Vector3d& p,double half,int level) {
        if(((p-center).cwiseAbs()-Eigen::Vector3d::Constant(half)).cwiseMax(0).squaredNorm()>radius*radius)return true;
        if(covered(p,std::sqrt(3.0)*half))return true;
        if((p-center).squaredNorm()<radius*radius && !covered(p,0))return false;
        // Preserve P3 v22: complete containment, bounded refinement, unknown rejects.
        if(level>=7 || --remaining<=0)return false;
        const double h=.5*half;
        for(int z:{-1,1})for(int y:{-1,1})for(int x:{-1,1})
            if(!certify(p+h*Eigen::Vector3d(x,y,z),h,level+1))return false;
        return true;
    };
    return certify(center,radius,0);
}
bool PaperGuidance::segmentKnown(const DepthObservation& o,const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,double radius,double now,bool pending) const {
    pending=pending||(cfg_.retain_certified_volume&&cfg_.retain_verified_travel&&cfg_.history_uncertainty_rate==0.);
    // Exact containment in a convex stored capsule/ball needs both end balls.
    for(auto it=verified_tubes_.rbegin();it!=verified_tubes_.rend();++it)
        if(tubeContains(*it,a,radius)&&tubeContains(*it,b,radius))return true;
    auto in_ball=[&](const auto& ball) {
        return std::max((a-ball.first).norm(),(b-ball.first).norm())+radius<=ball.second;
    };
    for(const auto& ball:seeds_)if(in_ball(ball))return true;
    for(const auto& ball:verified_travel_)if(in_ball(ball))return true;
    for(const auto& ball:reached_proofs_)if(in_ball(ball))return true;
    if(pending)for(const auto& ball:pending_motion_)if(in_ball(ball))return true;
    if(!verified_tubes_.empty()||!reached_proofs_.empty()) {
        std::vector<Interval> intervals;intervals.reserve(3*verified_tubes_.size()+verified_travel_.size());
        const Eigen::Vector3d delta=b-a;
        for(const auto& tube:verified_tubes_)capsuleIntervals(a,delta,tube,radius,&intervals);
        for(const auto& ball:seeds_)sphereInterval(a,delta,ball.first,ball.second-radius,&intervals);
        for(const auto& ball:verified_travel_)sphereInterval(a,delta,ball.first,ball.second-radius,&intervals);
        for(const auto& ball:reached_proofs_)sphereInterval(a,delta,ball.first,ball.second-radius,&intervals);
        if(pending)for(const auto& ball:pending_motion_)sphereInterval(a,delta,ball.first,ball.second-radius,&intervals);
        std::sort(intervals.begin(),intervals.end());
        double covered=0.;
        for(const auto& interval:intervals) {
            if(interval.first>covered)break;
            covered=std::max(covered,interval.second);
            if(covered>=1)return true;
        }
    }
    // An endpoint is a necessary condition for the whole capsule. Reject
    // unknown distant ends before repeatedly refining an already doomed prefix.
    if(!envelopeKnownImpl(o,b,radius,now,pending))return false;
    // Refine only a failed enclosing-ball approximation. Bounded work and no
    // endpoint-only acceptance, even when both endpoints are individually free.
    std::function<bool(const Eigen::Vector3d&,const Eigen::Vector3d&,int)> certify;
    certify=[&](const Eigen::Vector3d& x,const Eigen::Vector3d& y,int level) {
        const double length=(y-x).norm();
        if(envelopeKnownImpl(o,.5*(x+y),radius+.5*length,now,pending))return true;
        if(length<=cfg_.certificate_resolution || level>=2)return false;
        if(!envelopeKnownImpl(o,.5*(x+y),radius,now,pending))return false;
        return certify(x,.5*(x+y),level+1)&&certify(.5*(x+y),y,level+1);
    };
    const int pieces=std::max(1,static_cast<int>(std::ceil((b-a).norm()/.10)));
    const Eigen::Vector3d step=(b-a)/pieces;
    for(int i=0;i<pieces;++i)if(!certify(a+i*step,a+(i+1)*step,0))return false;
    return true;
}
double PaperGuidance::directionalClearance(const DepthObservation& o,
    const Eigen::Vector3d& position,const Eigen::Vector3d& direction,double now,double requested_maximum) const {
    if(o.depth.size()!=o.camera.rays().size()||!position.allFinite()||!direction.allFinite()||direction.norm()<1e-8||
        now<o.stamp-0.02||now-o.stamp>cfg_.observation_timeout)return 0;
    const Eigen::Vector3d d=direction.normalized();
    const double radius=envelopeRadius();
    if(!envelopeKnown(o,position,radius,now))return 0;
    double lower=0.0;
    const double maximum=std::min(requested_maximum,2*o.camera.maxDepth());
    while(lower<maximum) {
        const double upper=std::min(maximum,lower+.25);
        if(segmentKnown(o,position+lower*d,position+upper*d,radius,now)) {lower=upper;continue;}
        const double start=lower;double high=upper;
        for(int k=0;k<10;++k) {
            const double middle=.5*(lower+high);
            if(segmentKnown(o,position+start*d,position+middle*d,radius,now))lower=middle;
            else high=middle;
        }
        break;
    }
    return lower;
}
bool PaperGuidance::proposalCorridor(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction,double length,double now) {
    if(!position.allFinite()||!direction.allFinite()||direction.norm()<1e-8||
       !std::isfinite(length)||length<0||!std::isfinite(now)||
       now-input.stamp>cfg_.observation_timeout||now<input.stamp-.02)return false;
    DepthObservation evidence=input;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());}
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    return segmentKnown(evidence,position,position+length*direction.normalized(),envelopeRadius(),now);
}
bool PaperGuidance::proposalPath(const DepthObservation& input,
    const std::vector<Eigen::Vector3d>& points,double now) const {
    if(points.size()<2||!std::isfinite(now)||!std::isfinite(input.stamp)||
       now-input.stamp>cfg_.observation_timeout||now<input.stamp-.02)return false;
    for(const auto& point:points)if(!point.allFinite())return false;
    DepthObservation evidence=input;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());}
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    for(std::size_t i=1;i<points.size();++i)
        if(!segmentKnown(evidence,points[i-1],points[i],envelopeRadius(),now))return false;
    return true;
}
bool PaperGuidance::motionSafe(const DepthObservation& o,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& command,double now) const {
    return certifyMotion(o,position,velocity,command,now,nullptr);
}
bool PaperGuidance::certifyMotion(const DepthObservation& o,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& command,double now,
    std::vector<std::pair<Eigen::Vector3d,double>>* prefix) const {
    if(prefix)prefix->clear();
    if(!position.allFinite()||!velocity.allFinite()||!command.allFinite()) return false;
    Eigen::Vector3d p=position,v=velocity;
    const double dt=0.025;
    const double age=std::max(0.0,now-o.stamp);
    if(age>cfg_.observation_timeout) return false;
    const double radius=envelopeRadius();
    if(!envelopeKnown(o,position,radius,now))return false;
    // Delay + transition followed by an explicit braking tail to rest.
    const double transition=cfg_.motion.delay+cfg_.motion.rollout_horizon;
    int steps=static_cast<int>(std::ceil((transition+
        std::max(velocity.norm(),command.norm())/cfg_.motion.brake_accel+0.5)/dt));
    for(int k=0;k<steps;++k) {
        double t=k*dt;
        Eigen::Vector3d acceleration=Eigen::Vector3d::Zero();
        if(t<cfg_.motion.delay) { /* unchanged velocity during transport delay */ }
        else if(t<transition) acceleration=clampNorm((command-v)/cfg_.motion.velocity_tau,cfg_.motion.max_accel);
        else if(v.norm()>1e-7) acceleration=-std::min(cfg_.motion.brake_accel,v.norm()/dt)*v.normalized();
        Eigen::Vector3d next_v=v+dt*acceleration, next=p+0.5*dt*(v+next_v);
        // Enclose the entire center segment plus the acceleration bulge.
        const double swept_radius=radius+
            std::max(cfg_.motion.max_accel,cfg_.motion.brake_accel)*dt*dt/8.0;
        if(cfg_.shared_obstacles&&!shared_.segmentSafe(p,next,swept_radius,now,t,t+dt))return false;
        if(cfg_.dynamic_obstacles&&!dynamic_.segmentSafe(p,next,swept_radius,now,t,t+dt))return false;
        if(!segmentKnown(o,p,next,swept_radius,now))return false;
        if(prefix) {
            const int pieces=std::max(1,static_cast<int>(std::ceil((next-p).norm()/.10)));
            const Eigen::Vector3d step=(next-p)/pieces;
            for(int i=0;i<pieces;++i) {
                const Eigen::Vector3d center=p+(i+.5)*step;
                double known_radius=swept_radius+.5*step.norm();
                if(envelopeKnown(o,center,known_radius,now)) {
                    if(envelopeKnown(o,center,known_radius+.02,now))known_radius+=.02;
                    prefix->emplace_back(center,known_radius);
                }
            }
        }
        p=next;v=next_v;
        if(t>=transition&&v.norm()<1e-6) {
            // Safety alone permits stopping on the boundary of F_safe, where
            // no open angular domain may remain. Preserve an observed restart
            // neighborhood; this is a feasibility constraint, not a new controller.
            if(cfg_.restart_clearance>0) {
                const double restart_radius=radius+cfg_.restart_clearance;
                if(!envelopeKnown(o,p,restart_radius,now))return false;
                if(prefix)prefix->emplace_back(p,restart_radius);
            }
            return true;
        }
    }
    return v.norm()<1e-5;
}
bool PaperGuidance::build(const DepthObservation& o,const Eigen::Vector3d& position,
    const Eigen::Vector3d& direction,const Eigen::Vector2d& state,double speed,double now,
    const Eigen::Vector2d* source_hint,const Eigen::Vector2d* goal_hint,double measured_speed) {
    int w=o.camera.width(),h=o.camera.height();
    BinaryMask full(w*h,1);
    const double lookahead=std::max(cfg_.minimum_lookahead,speed*cfg_.motion.planning_horizon+
        speed*cfg_.motion.delay+speed*speed/(2*cfg_.motion.brake_accel));
    const double stopping=measured_speed*cfg_.motion.delay+
        measured_speed*measured_speed/(2*cfg_.motion.brake_accel);
    // Geometric/execution uncertainty is already in B_eff. The remaining
    // distance reserve is explicit and tied to certification resolution.
    const double required=cfg_.adaptive_lookahead?
        std::max(2*cfg_.certificate_resolution,stopping+cfg_.certificate_resolution):
        std::max(cfg_.minimum_lookahead,stopping);
    required_prefix_=required;
    std::vector<double> horizons;
    double level_horizon=std::max(required,lookahead);
    for(int level=0;level<12;++level) {
        horizons.push_back(level_horizon);
        if(level_horizon<=required+1e-8)break;
        level_horizon=level==10?required:std::max(required,.5*level_horizon);
    }
    std::vector<double> clearances(w*h,0);
    // Eq. (5) needs threshold membership, not a millimetric estimate of rho
    // at every pixel. Prove nested horizons from short to long and stop at the
    // first uncertified extension. Only the final command needs fine clearance.
    // Each worker has a frame-local positive certificate cache. Geometry and
    // evidence are immutable until this parallel region has joined.
#ifdef _OPENMP
#pragma omp parallel for num_threads(2) schedule(static) if(o.supporting_views.size()>=4)
#endif
    for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x) {
        const Eigen::Vector3d ray=o.rotation*o.camera.ray(x,y);
        double prefix=0;
        for(auto threshold=horizons.rbegin();threshold!=horizons.rend();++threshold) {
            const double target=*threshold+cfg_.certificate_resolution;
            if(!segmentKnown(o,position+prefix*ray,position+target*ray,envelopeRadius(),now))break;
            prefix=target;
        }
        clearances[idx(x,y,w)]=prefix;
    }
    maximum_clearance_=*std::max_element(clearances.begin(),clearances.end());
    double active_horizon=0;
    auto attempt=[&](int factor)->bool {
    AngularField coarse;coarse.width=(w+factor-1)/factor;coarse.height=(h+factor-1)/factor;
    coarse.spacing=factor;coarse.offset=Eigen::Vector2d(0.5*(factor-1),0.5*(factor-1));
    coarse.blocked.assign(coarse.width*coarse.height,1);
    for(int y=0;y<coarse.height;++y) for(int x=0;x<coarse.width;++x) {
        bool blocked=false;
        for(int j=0;j<factor;++j) for(int i=0;i<factor;++i) {
            int u=x*factor+i,v=y*factor+j;
            if(u>=w||v>=h||full[idx(u,v,w)]) blocked=true;
        }
        coarse.blocked[idx(x,y,coarse.width)]=blocked;
    }
    auto nearest=[&](const Eigen::Vector2d& p)->int {
        int best=-1;double cost=1e100;
        for(int y=0;y<coarse.height;++y) for(int x=0;x<coarse.width;++x) {
            int i=idx(x,y,coarse.width);if(coarse.blocked[i]) continue;
            double c=(coarse.offset+factor*Eigen::Vector2d(x,y)-p).squaredNorm();
            if(c<cost){cost=c;best=i;}
        }return best;
    };
    int source=nearest(state);if(source<0){build_reason_="NO_SOURCE_COMPONENT";return false;}
    component(coarse.blocked,coarse.width,coarse.height,source);
    if(measured_speed>.03 && !coarse.contains(state)){build_reason_="STATE_OUTSIDE_COMPONENT";return false;}
    if(source_hint)source=nearest(*source_hint);
    int goal=-1;double best_angle=1e100,best_history=1e100;
    Eigen::Vector2d selected_goal=Eigen::Vector2d::Zero();
    double goal_horizon=active_horizon;
    if(cfg_.response_preview) {
        // Shorten the domain enough to include the measured stopping direction,
        // but keep a longer viable destination. Otherwise q remains the sink
        // until the robot is already at the obstacle's braking boundary.
        double best_prefix=0;
        for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x) {
            const Eigen::Vector2d p(x,y);
            if(coarse.contains(p)&&(o.rotation*o.camera.ray(x,y)).dot(direction)>1e-6)
                best_prefix=std::max(best_prefix,clearances[idx(x,y,w)]);
        }
        goal_horizon=std::max(active_horizon,best_prefix-cfg_.certificate_resolution-1e-8);
        for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x) {
            const Eigen::Vector2d p(x,y);
            if(!coarse.contains(p)||clearances[idx(x,y,w)]<=goal_horizon)continue;
            const double alignment=(o.rotation*o.camera.ray(x,y)).dot(direction);
            if(cfg_.intent_guard&&alignment<=1e-6)continue;
            const double angle=std::acos(clip(alignment,-1,1));
            const double history=goal_hint?(p-*goal_hint).squaredNorm():0;
            if(angle<best_angle-1e-9||(std::abs(angle-best_angle)<1e-9&&history<best_history)) {
                goal=idx(static_cast<int>(std::round((x-coarse.offset.x())/factor)),
                    static_cast<int>(std::round((y-coarse.offset.y())/factor)),coarse.width);
                selected_goal=p;best_angle=angle;best_history=history;
            }
        }
    } else for(int y=0;y<coarse.height;++y) for(int x=0;x<coarse.width;++x) {
        int i=idx(x,y,coarse.width);if(coarse.blocked[i]) continue;
        Eigen::Vector2d p=coarse.offset+factor*Eigen::Vector2d(x,y);
        const double alignment=(o.rotation*o.camera.rayFromPixel(p)).dot(direction);
        if(cfg_.intent_guard&&alignment<=1e-6)continue;
        double angle=std::acos(clip(alignment,-1,1));
        double history=goal_hint?(p-*goal_hint).squaredNorm():0;
        if(angle<best_angle-1e-9||(std::abs(angle-best_angle)<1e-9&&history<best_history))
            {goal=i;selected_goal=p;best_angle=angle;best_history=history;}
    }
    if(goal<0){build_reason_="NO_INTENT_PROGRESS";return false;}
    if(cfg_.response_preview) {
        Eigen::Vector2d requested;
        if(o.camera.pixelFromDirection(o.rotation.transpose()*direction,&requested)&&
            coarse.segmentFree(selected_goal,requested)) {
            const Eigen::Vector2d base=selected_goal,delta=requested-base;
            double low=0,high=1;
            for(int i=0;i<7;++i) {
                const double fraction=.5*(low+high);
                const Eigen::Vector2d probe=base+fraction*delta;
                const Eigen::Vector3d ray=o.rotation*o.camera.rayFromPixel(probe);
                if(segmentKnown(o,position,position+(goal_horizon+cfg_.certificate_resolution)*ray,envelopeRadius(),now)) {
                    low=fraction;selected_goal=probe;
                } else high=fraction;
            }
            goal=nearest(selected_goal);
        }
    }
    // Source and target must be disjoint. Put the source upstream when the
    // requested state already lies in the target neighborhood.
    const bool separate_source=source==goal;
    if(separate_source) {
        double best=1e100;int alternate=-1;
        Eigen::Vector2d desired=coarse.offset+factor*Eigen::Vector2d(goal%coarse.width,goal/coarse.width);
        Eigen::Vector2d away=desired-state;
        if(away.norm()<0.5) away=Eigen::Vector2d(1,0);
        Eigen::Vector2d upstream=state-4.0*away.normalized();
        for(int i=0;i<coarse.width*coarse.height;++i) if(!coarse.blocked[i]&&i!=goal) {
            Eigen::Vector2d p=coarse.offset+factor*Eigen::Vector2d(i%coarse.width,i/coarse.width);
            double d=(p-upstream).squaredNorm();if(d<best){best=d;alternate=i;}
        }
        if(alternate<0){build_reason_="SOURCE_TARGET_NOT_SEPARABLE";return false;}
        source=alternate;
    }
    coarse.source=coarse.offset+factor*Eigen::Vector2d(source%coarse.width,source/coarse.width);
    // Preserve the continuous world-direction source across chart refreshes.
    // Repeated nearest-cell snapping would drag it along with measured motion.
    if(!separate_source) {
        const Eigen::Vector2d exact_source=source_hint?*source_hint:state;
        if(coarse.segmentFree(exact_source,coarse.source))coarse.source=exact_source;
    }
    coarse.goal=selected_goal;
    Eigen::Vector2d exact_goal;
    if (o.camera.pixelFromDirection(o.rotation.transpose()*direction,&exact_goal) && coarse.contains(exact_goal) &&
        directionalClearance(o,position,direction,now,goal_horizon+.05)>goal_horizon)
        coarse.goal=exact_goal;
    std::vector<double> boundaries(coarse.blocked.size(),nan);boundaries[source]=1;boundaries[goal]=0;
    const auto coarse_guess=cfg_.incremental_field&&observation_?reprojectPotential(field_,*observation_,coarse,o,std::max(.1,active_horizon)):std::vector<double>();
    coarse=solve(coarse,boundaries,coarse_guess);if(!coarse.valid){build_reason_="COARSE_FIELD_DEGENERATE";return false;}
    if(!cfg_.coarse_fine){field_=std::move(coarse);coupling_error_=0;build_reason_="READY";return true;}
    AngularField fine;fine.width=std::min(w,cfg_.fine_width);fine.height=std::min(h,cfg_.fine_height);
    const Eigen::Vector2d fine_center=coarse.contains(state)?state:coarse.source;
    fine.offset=Eigen::Vector2d(clip(std::round(fine_center.x()-fine.width/2.0),0,w-fine.width),
        clip(std::round(fine_center.y()-fine.height/2.0),0,h-fine.height));
    fine.source=coarse.source;fine.goal=coarse.goal;
    fine.blocked.resize(fine.width*fine.height);
    std::vector<double> fixed(fine.blocked.size(),nan);
    for(int y=0;y<fine.height;++y) for(int x=0;x<fine.width;++x) {
        Eigen::Vector2d p=fine.offset+Eigen::Vector2d(x,y);int i=idx(x,y,fine.width);
        fine.blocked[i]=full[idx(static_cast<int>(p.x()),static_cast<int>(p.y()),w)] || !coarse.contains(p);
        if(fine.blocked[i]) continue;
        if((p-coarse.source).norm()<=1.0)fixed[i]=1;
        else if((p-coarse.goal).norm()<=1.0)fixed[i]=0;
        else if(x==0||y==0||x==fine.width-1||y==fine.height-1)fixed[i]=coarse.value(p);
    }
    int seed=-1;double dist=1e100;
    for(int y=0;y<fine.height;++y)for(int x=0;x<fine.width;++x){int i=idx(x,y,fine.width);
        double d=(fine.offset+Eigen::Vector2d(x,y)-state).squaredNorm();if(!fine.blocked[i]&&d<dist){dist=d;seed=i;}}
    component(fine.blocked,fine.width,fine.height,seed);
    const auto fine_guess=cfg_.incremental_field&&observation_?reprojectPotential(field_,*observation_,fine,o,std::max(.1,active_horizon)):std::vector<double>();
    field_=solve(fine,fixed,fine_guess);coupling_error_=0;
    if(field_.valid)for(int i=0;i<static_cast<int>(fixed.size());++i)
        if(!field_.blocked[i]&&std::isfinite(fixed[i]))coupling_error_=std::max(coupling_error_,std::abs(field_.potential[i]-fixed[i]));
    build_reason_=!field_.valid?"FINE_FIELD_DEGENERATE":
        (measured_speed>.03&&!field_.contains(state)?"STATE_OUTSIDE_FINE_COMPONENT":"READY");
    return build_reason_=="READY";
    };
    const int factor=cfg_.coarse_fine?std::max(1,cfg_.coarse_factor):1;
    // Eq. (5) permits a positive look-ahead; it is not a mandatory three-second
    // full-command trajectory. Keep the desired horizon when feasible, otherwise
    // shorten it no further than the measured-speed stopping requirement.
    double horizon=std::max(required,lookahead);
    for(int level=0;level<12;++level) {
        active_horizon=horizon;
        for(int i=0;i<w*h;++i)full[i]=clearances[i]<=horizon;
        free_directions_=std::count(full.begin(),full.end(),0);
        build_reason_=free_directions_?"BUILD_PENDING":"NO_CERTIFIED_DIRECTION";
        if(!free_directions_) {
            if(horizon<=required+1e-8)break;
            horizon=level==10?required:std::max(required,.5*horizon);continue;
        }
        const bool state_certified=measured_speed<=.03 ||
            directionalClearance(o,position,o.rotation*o.camera.rayFromPixel(state),now,horizon+.05)>horizon;
        if(state_certified && (attempt(factor)||(factor>1&&attempt(1)))) {field_lookahead_=horizon;return true;}
        if(!state_certified)build_reason_="STATE_DIRECTION_UNCERTIFIED";
        if(horizon<=required+1e-8)break;
        // Keep bounded work, but always evaluate the actual lower bound on
        // final attempt. Preserve intermediate .5/.25/.125m horizons instead
        // of jumping straight from about 1m to the 1cm stopping lower bound.
        horizon=level==10?required:std::max(required,.5*horizon);
    }
    return false;
}

bool PaperGuidance::continueAnchor(const DepthObservation& next,const Eigen::Vector2d& state,Eigen::Vector2d* anchor) const {
    if(reference_world_.empty())return false;
    double best=1e100;bool found=false,previous_valid=false;
    Eigen::Vector2d previous;
    for(const auto& ray:reference_world_) {
        Eigen::Vector2d p;
        if(!next.camera.pixelFromDirection(next.rotation.transpose()*ray,&p)) {previous_valid=false;continue;}
        Eigen::Vector2d candidate=p;
        if(previous_valid) {
            const Eigen::Vector2d segment=p-previous;
            if(segment.squaredNorm()>1e-12)
                candidate=previous+clip((state-previous).dot(segment)/segment.squaredNorm(),0,1)*segment;
        }
        const double d=(candidate-state).squaredNorm();
        if(d<best){best=d;*anchor=candidate;found=true;}
        previous=p;previous_valid=true;
    }
    return found;
}
Eigen::Vector3d PaperGuidance::memoryProposal(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& intent,double now) const {
    if(!cfg_.spherical_memory||!intent.allFinite()||intent.norm()<1e-5)return Eigen::Vector3d::Zero();
    DepthObservation evidence=input;auto views=spherical_.evidence(now);
    evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    auto candidates=spherical_.directions(position,now);candidates.push_back(intent.normalized());
    std::stable_sort(candidates.begin(),candidates.end(),[&](const auto& a,const auto& b){return a.dot(intent)>b.dot(intent);});
    int tested=0;
    for(const auto& direction:candidates) {
        if(direction.dot(intent)<=.1||tested++>=8)break;
        const double length=std::max(.25,std::min(1.,intent.norm()));
        if(segmentKnown(evidence,position,position+length*direction,envelopeRadius(),now))return std::min(intent.norm(),cfg_.max_speed)*direction;
    }
    return Eigen::Vector3d::Zero();
}
Eigen::Vector3d PaperGuidance::incrementalGoalProposal(const Eigen::Vector3d& position,
    const Eigen::Vector3d& intent,double now) const {
    if(cfg_.incremental_field&&
       position.allFinite()&&intent.allFinite()&&intent.norm()>1e-5&&
       observation_&&field_.valid&&std::isfinite(now)&&
       now>=observation_->stamp-.02&&now-observation_->stamp<=.5&&
       reference_intent_.norm()>.5&&
       reference_intent_.dot(intent.normalized())>=std::cos(cfg_.command_change_angle)) {
        const double range=std::max(cfg_.minimum_lookahead,maximum_clearance_);
        const Eigen::Vector3d target=field_position_+
            range*observation_->rotation*observation_->camera.rayFromPixel(field_.goal);
        const Eigen::Vector3d delta=target-position;
        if(delta.allFinite()&&delta.norm()>1e-5&&delta.dot(intent)>1e-6) {
            return std::min(intent.norm(),cfg_.max_speed)*delta.normalized();
        }
    }
    return Eigen::Vector3d::Zero();
}
PaperResult PaperGuidance::stepWithProposal(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,const Eigen::Vector3d& supplied_proposal,double now,double dt) {
    Eigen::Vector3d proposal=supplied_proposal;
    bool reused_goal=false;
    if(proposal.allFinite()&&proposal.norm()<1e-5) {
        const auto cached=incrementalGoalProposal(position,intent,now);
        if(cached.norm()>1e-5){proposal=cached;reused_goal=true;}
    }
    if(!position.allFinite()||!velocity.allFinite()||!intent.allFinite()||!proposal.allFinite()||
       !std::isfinite(now)||!std::isfinite(dt)||dt<=0||intent.norm()<1e-5||proposal.norm()<1e-5||
       now-input.stamp>cfg_.observation_timeout||now<input.stamp-.02||
       (cfg_.intent_guard&&(proposal.dot(intent)<=0||(velocity.norm()>.03&&velocity.dot(intent)<-1e-6))))
        return step(input,position,velocity,intent,now,dt);
    DepthObservation evidence=input;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());}
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    for(auto& cache:envelope_caches_)cache.clear();
    cache_enabled_=true;
    rememberVerifiedPosition(evidence,position,now);
    PaperResult out;
    if(envelopeKnown(evidence,position,envelopeRadius(),now)) {
        const double requested=std::min({intent.norm(),proposal.norm(),cfg_.max_speed});
        Eigen::Vector3d desired=requested*proposal.normalized();
        if(std::abs(desired.z())>cfg_.max_vertical_speed)desired*=cfg_.max_vertical_speed/std::abs(desired.z());
        const double interval=std::min(dt,.1);
        const bool feedforward=cfg_.proposal_feedforward&&cfg_.smooth_speed;
        Eigen::Vector3d anchor=cfg_.model_velocity_shaping?velocity:previous_command_;
        if(feedforward) {
            anchor=proposal_reference_valid_?proposal_reference_:velocity;
            double scale=anchor.norm()>cfg_.max_speed?cfg_.max_speed/anchor.norm():1.;
            if(std::abs(anchor.z())>cfg_.max_vertical_speed)scale=std::min(scale,cfg_.max_vertical_speed/std::abs(anchor.z()));
            anchor*=scale;
        }
        // The downstream plant already enforces its physical acceleration.
        // A separate bounded command ramp can remove duplicated slow response;
        // never increase the acceleration assumed by the motion certificate.
        const double command_accel=(!feedforward&&!cfg_.model_velocity_shaping&&cfg_.proposal_command_accel>0.)?
            cfg_.proposal_command_accel:cfg_.motion.max_accel;
        const Eigen::Vector3d target_acceleration=clampNorm((desired-anchor)/cfg_.proposal_response_time,command_accel);
        Eigen::Vector3d acceleration=clampNorm(proposal_acceleration_+
            clampNorm(target_acceleration-proposal_acceleration_,cfg_.proposal_jerk_limit*interval),command_accel);
        // Shape physical acceleration, then invert the known first-order
        // response. Avoid filtering the velocity twice before it reaches the
        // plant; the resulting whole command is still magnitude-limited and
        // independently certified through its complete braking trajectory.
        Eigen::Vector3d shaped=desired;
        if(cfg_.smooth_speed)shaped=cfg_.model_velocity_shaping?
            (velocity+cfg_.motion.velocity_tau*acceleration).eval():
            (previous_command_+interval*acceleration).eval();
        Eigen::Vector3d next_reference=anchor;
        if(feedforward) {
            // Keep an independent smooth velocity reference and compensate
            // the known plant lag. Do not integrate the already compensated
            // command or differentiate delayed measured velocity every frame.
            const double preview=std::max(interval,cfg_.motion.velocity_tau);
            const Eigen::Vector3d delta=preview*acceleration;
            double fraction=1.;
            if((anchor+delta).squaredNorm()>cfg_.max_speed*cfg_.max_speed&&delta.squaredNorm()>1e-18) {
                const double a=delta.squaredNorm(),b=2.*anchor.dot(delta);
                const double c=std::min(0.,anchor.squaredNorm()-cfg_.max_speed*cfg_.max_speed);
                fraction=clip((-b+std::sqrt(std::max(0.,b*b-4.*a*c)))/(2.*a),0.,1.);
            }
            if(std::abs(anchor.z()+fraction*delta.z())>cfg_.max_vertical_speed&&std::abs(delta.z())>1e-12)
                fraction=std::min(fraction,clip((std::copysign(cfg_.max_vertical_speed,delta.z())-anchor.z())/delta.z(),0.,1.));
            acceleration*=fraction;
            shaped=anchor+cfg_.motion.velocity_tau*acceleration;
            next_reference=anchor+interval*acceleration;
        }
        double scale=shaped.norm()>cfg_.max_speed?cfg_.max_speed/shaped.norm():1.;
        if(std::abs(shaped.z())>cfg_.max_vertical_speed)scale=std::min(scale,cfg_.max_vertical_speed/std::abs(shaped.z()));
        shaped*=scale;
        const double speed=shaped.norm();
        const Eigen::Vector3d ray=speed>1e-9?shaped/speed:proposal.normalized();
        const Eigen::Vector3d old_command=previous_command_;
        auto accept=[&](double magnitude) {
            if(magnitude<1e-4||ray.dot(intent)<=1e-6)return false;
            std::vector<std::pair<Eigen::Vector3d,double>> prefix;
            if(!certifyMotion(evidence,position,velocity,magnitude*ray,now,cfg_.retain_verified_travel?&prefix:nullptr))return false;
            if(cfg_.retain_verified_travel)pending_motion_=std::move(prefix);
            out.command=magnitude*ray;out.accepted=true;out.status="DEPTH_PROPOSAL_CERTIFIED";
            out.build_reason="FULL_MOTION_CERTIFICATE";
            // Safety reductions may exceed the ordinary jerk bound. Do not
            // carry their emergency acceleration into the next smooth update.
            const bool unchanged=std::abs(magnitude-speed)<1e-9;
            confirmPublishedCommand(out.command);
            proposal_acceleration_=unchanged?clampNorm(feedforward?acceleration:
                (cfg_.model_velocity_shaping?((out.command-velocity)/cfg_.motion.velocity_tau).eval():
                    ((out.command-old_command)/interval).eval()),command_accel):Eigen::Vector3d::Zero();
            if(feedforward&&unchanged) {proposal_reference_=next_reference;proposal_reference_valid_=true;}
            return true;
        };
        if(!accept(speed)) {
            double high=speed,low=std::min(speed,old_command.norm());
            bool found=low<high&&accept(low);
            if(!found)for(int i=0;i<10;++i){low=.5*high;if(accept(low)){found=true;break;}high=low;}
            if(found)for(int i=0;i<5&&high-low>.005;++i) {
                const double middle=.5*(low+high);if(accept(middle))low=middle;else high=middle;
            }
        }
    }
    cache_enabled_=false;
    if(out.accepted) {
        if(reused_goal)out.status="INCREMENTAL_GOAL_CERTIFIED";
        else {observation_.reset();field_=AngularField();box_=FlowBox();reference_world_.clear();
            reference_intent_=intent.normalized();}
        diagnostic_valid_=false;
        confirmPublishedCommand(out.command);return out;
    }
    return step(input,position,velocity,intent,now,dt);
}
PaperResult PaperGuidance::step(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,double now,double dt) {
    const Eigen::Vector3d published=previous_command_;
    PaperResult result=stepImpl(input,position,velocity,intent,now,dt);
    // Numerical angular cells/flow-box stencils can reject a physically safe
    // transition. Recover only with a fresh complete-space trajectory proof;
    // this is a bounded execution fallback, not fabricated free angular cells.
    if(cfg_.certified_direct&&position.allFinite()&&velocity.allFinite()&&intent.allFinite()&&
       intent.norm()>1e-5&&std::isfinite(dt)&&dt>0&&std::isfinite(now)&&
       now-input.stamp<=cfg_.observation_timeout&&now>=input.stamp-.02&&
       !(cfg_.intent_guard&&velocity.norm()>.03&&velocity.dot(intent)<-1e-6)&&
       (!result.accepted||result.command.norm()+1e-5<std::min(std::min(intent.norm(),cfg_.max_speed),published.norm()+cfg_.motion.max_accel*std::min(dt,.1)))) {
        DepthObservation evidence=input;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);evidence.supporting_views.insert(evidence.supporting_views.end(),views.begin(),views.end());}
        evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
        cache_enabled_=true;
        if(envelopeKnown(evidence,position,envelopeRadius(),now)) {
            const Eigen::Vector3d q=intent.normalized();
            const Eigen::Vector3d side=q.unitOrthogonal(),up=q.cross(side).normalized();
            const double requested=std::min(intent.norm(),cfg_.max_speed);
            const double increment=cfg_.motion.max_accel*std::min(dt,.1);
            const double nominal=cfg_.smooth_speed?clip(requested,std::max(0.,published.norm()-increment),published.norm()+increment):requested;
            const double stopping=velocity.norm()*cfg_.motion.delay+velocity.squaredNorm()/(2*cfg_.motion.brake_accel);
            const double lookahead=std::max(.25,requested*cfg_.motion.planning_horizon+stopping);
            auto towards=[](const Eigen::Vector3d& from,const Eigen::Vector3d& to,double bound)->Eigen::Vector3d {
                const double angle=std::atan2(from.cross(to).norm(),from.dot(to));
                if(angle<=bound)return to;
                Eigen::Vector3d axis=from.cross(to);
                if(axis.norm()<1e-10)axis=from.unitOrthogonal();else axis.normalize();
                return Eigen::AngleAxisd(bound,axis)*from;
            };
            bool recovered=false,exhausted=false;
            // Preserve smooth execution toward the freshly selected safe goal
            // when only the numerical angular stencil/flow box is unusable.
            // This mirrors the old controller's continuous target tracking,
            // but validates the resulting whole-vector motion before publishing.
            Eigen::Vector3d tracking_target=published.norm()>1e-5?published.normalized():q;
            if(observation_&&field_.valid)tracking_target=observation_->rotation*observation_->camera.rayFromPixel(field_.goal);
            Eigen::Vector3d tracking_ray=tracking_target;
            if(velocity.norm()>.03)tracking_ray=towards(velocity.normalized(),tracking_ray,cfg_.motion.max_direction_rate*(cfg_.motion.velocity_tau+std::min(dt,.1)));
            if(published.norm()>.03)tracking_ray=towards(published.normalized(),tracking_ray,cfg_.motion.max_direction_rate*std::min(dt,.1));
            double tracking_speed=nominal;
            if(std::abs(tracking_ray.z())>1e-9)tracking_speed=std::min(tracking_speed,cfg_.max_vertical_speed/std::abs(tracking_ray.z()));
            if(tracking_ray.dot(q)>1e-6&&(!result.accepted||tracking_speed>result.command.norm()+1e-5)) {
                std::vector<std::pair<Eigen::Vector3d,double>> prefix;
                if(certifyMotion(evidence,position,velocity,tracking_speed*tracking_ray,now,cfg_.retain_verified_travel?&prefix:nullptr)) {
                    if(cfg_.retain_verified_travel)pending_motion_=std::move(prefix);
                    result.command=tracking_speed*tracking_ray;result.accepted=true;
                    result.status="CERTIFIED_TRACKING";result.tracking=false;result.continued=false;
                    result.reset_reason="CERTIFIED_TRACKING";recovered=true;
                }
            }
            int ordinal=0,evaluated=0;
            const int begin=recovery_cursor_;
            for(double horizon:{lookahead,std::max(.25,.5*lookahead),std::max(.25,stopping+.25)}) {
                if(recovered||exhausted)break;
                for(double degrees:{0.,10.,20.,30.,45.,60.,80.}) {
                    if(recovered||exhausted)break;
                    const double angle=degrees*std::acos(-1.)/180.;
                    struct Proposal { Eigen::Vector3d target;double history; };
                    std::vector<Proposal> proposals;
                    for(int k=0;k<(degrees==0?1:8);++k) {
                        const double azimuth=k*std::acos(-1.)/4.;
                        const Eigen::Vector3d target=std::cos(angle)*q+std::sin(angle)*(std::cos(azimuth)*side+std::sin(azimuth)*up);
                        proposals.push_back({target,published.norm()>.03?target.dot(published.normalized()):target.dot(q)});
                    }
                    std::stable_sort(proposals.begin(),proposals.end(),[](const Proposal& a,const Proposal& b){return a.history>b.history;});
                    for(const auto& proposal:proposals) {
                        const int probe=ordinal++;
                        if(probe<begin)continue;
                        if(evaluated++>=12){recovery_cursor_=probe;exhausted=true;break;}
                        if(!segmentKnown(evidence,position,position+horizon*proposal.target,envelopeRadius(),now))continue;
                        Eigen::Vector3d ray=proposal.target;
                        if(velocity.norm()>.03)ray=towards(velocity.normalized(),ray,cfg_.motion.max_direction_rate*(cfg_.motion.velocity_tau+std::min(dt,.1)));
                        if(published.norm()>.03)ray=towards(published.normalized(),ray,cfg_.motion.max_direction_rate*std::min(dt,.1));
                        if(ray.dot(q)<=1e-6)continue;
                        double speed=nominal;
                        if(std::abs(ray.z())>1e-9)speed=std::min(speed,cfg_.max_vertical_speed/std::abs(ray.z()));
                        if(result.accepted&&speed<=result.command.norm()+1e-5)continue;
                        std::vector<std::pair<Eigen::Vector3d,double>> prefix;
                        if(!certifyMotion(evidence,position,velocity,speed*ray,now,cfg_.retain_verified_travel?&prefix:nullptr))continue;
                        reset();reference_intent_=q;
                        if(cfg_.retain_verified_travel)pending_motion_=std::move(prefix);
                        result.command=speed*ray;result.accepted=true;result.status="CERTIFIED_REENTRY";
                        result.tracking=false;result.continued=false;result.reset_reason="CERTIFIED_REENTRY";
                        result.clearance=horizon;recovered=true;break;
                    }
                }
            }
            if(recovered||!exhausted)recovery_cursor_=0;
        }
        cache_enabled_=false;
    }
    confirmPublishedCommand(result.command);
    return result;
}
PaperResult PaperGuidance::stepImpl(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,double now,double dt) {
    for(auto& cache:envelope_caches_)cache.clear();
    cache_enabled_=true;
    struct CacheScope {bool& enabled;~CacheScope(){enabled=false;}} cache_scope{cache_enabled_};
    DepthObservation next=input;
    next.supporting_views=history_;
    if(cfg_.spherical_memory){auto views=spherical_.evidence(now);next.supporting_views.insert(next.supporting_views.end(),views.begin(),views.end());}
    next.supporting_views.insert(next.supporting_views.end(),input.supporting_views.begin(),input.supporting_views.end());
    PaperResult out;
    const bool diagnostic_previous=diagnostic_valid_;
    diagnostic_valid_=false;
    if(!position.allFinite()||!velocity.allFinite()||!intent.allFinite()||!std::isfinite(now)||!std::isfinite(dt)||dt<=0) {out.status="INVALID_STATE";return out;}
    if(intent.norm()<1e-5){reset();out.status="ZERO_INTENT";return out;}
    if(now-next.stamp>cfg_.observation_timeout||now<next.stamp-0.02){out.status="STALE_DEPTH";return out;}
    Eigen::Vector3d direction=intent.normalized();
    if(cfg_.intent_guard&&velocity.norm()>.03&&velocity.dot(direction)<-1e-6) {
        // Intent reversal or drift behind the operator: brake before reseeding
        // from q. Do not keep commanding a long arc in the old hemisphere.
        reset();out.status="INTENT_REVERSAL_BRAKE";return out;
    }
    Eigen::Vector3d measured=velocity.norm()>0.03?velocity.normalized():
        (previous_command_.norm()>1e-5?previous_command_.normalized():direction);
    Eigen::Vector2d requested,new_state;
    if(!next.camera.pixelFromDirection(next.rotation.transpose()*direction,&requested)) {
        // Eq. (6) compares every admissible direction to q; q itself need not
        // belong to the active chart. This is only a low-speed initial hint.
        double best=-2;
        for(int y=1;y<next.camera.height()-1;++y)for(int x=1;x<next.camera.width()-1;++x) {
            const double cosine=(next.rotation*next.camera.ray(x,y)).dot(direction);
            if(cosine>best){best=cosine;requested=Eigen::Vector2d(x,y);}
        }
    }
    if(!next.camera.pixelFromDirection(next.rotation.transpose()*measured,&new_state)) {
        if (velocity.norm() <= 0.03) { measured=next.rotation*next.camera.rayFromPixel(requested); new_state=requested; }
        else { out.status="STATE_OUTSIDE_DEPTH_FOV"; return out; }
    }
    rememberVerifiedPosition(next,position,now);
    if(!envelopeKnown(next,position,envelopeRadius(),now)) {
        reset();out.status="UNOBSERVED_BODY_ENVELOPE";return out;
    }
    if(cfg_.certified_direct) {
        const double requested_speed=std::min(intent.norm(),cfg_.max_speed);
        const double stopping=velocity.norm()*cfg_.motion.delay+velocity.squaredNorm()/(2*cfg_.motion.brake_accel);
        const double corridor=requested_speed*cfg_.motion.planning_horizon+stopping+.05;
        // A straight reference needs no numerical harmonic solve when its whole
        // long corridor is already certified. No map/oracle is used here.
        if(segmentKnown(next,position,position+corridor*direction,envelopeRadius(),now)) {
            auto towards=[](const Eigen::Vector3d& from,const Eigen::Vector3d& to,double limit)->Eigen::Vector3d {
                const double angle=std::atan2(from.cross(to).norm(),from.dot(to));
                if(angle<=limit)return to;
                Eigen::Vector3d axis=from.cross(to);
                if(axis.norm()<1e-10)axis=from.unitOrthogonal();else axis.normalize();
                return Eigen::AngleAxisd(limit,axis)*from;
            };
            const double horizon=cfg_.response_preview?cfg_.motion.velocity_tau+std::min(dt,.1):dt;
            Eigen::Vector3d ray=velocity.norm()>.03?towards(measured,direction,cfg_.motion.max_direction_rate*horizon):direction;
            if(previous_command_.norm()>.03)ray=towards(previous_command_.normalized(),ray,cfg_.motion.max_direction_rate*std::min(dt,.1));
            const double increment=cfg_.motion.max_accel*std::min(dt,.1);
            double speed=cfg_.smooth_speed?clip(requested_speed,std::max(0.,previous_command_.norm()-increment),previous_command_.norm()+increment):requested_speed;
            if(std::abs(ray.z())>1e-9)speed=std::min(speed,cfg_.max_vertical_speed/std::abs(ray.z()));
            std::vector<std::pair<Eigen::Vector3d,double>> prefix;
            if(ray.dot(direction)>1e-6&&certifyMotion(next,position,velocity,speed*ray,now,cfg_.retain_verified_travel?&prefix:nullptr)) {
                reset();reference_intent_=direction;
                if(cfg_.retain_verified_travel)pending_motion_=std::move(prefix);
                out.command=speed*ray;out.accepted=true;out.status="CERTIFIED_DIRECT";
                out.build_reason="CERTIFIED_INTENT_CORRIDOR";out.clearance=corridor;
                out.best_prefix=corridor;out.required_prefix=stopping;return out;
            }
        }
    }
    // Eq. (24): compare q with the intent that selected the active reference,
    // not the last depth/field refresh. Small successive changes accumulate.
    out.intent_change_angle=reference_intent_.norm()<0.5?0.0:
        std::atan2(reference_intent_.cross(direction).norm(),reference_intent_.dot(direction));
    const bool changed=reference_intent_.norm()<0.5||out.intent_change_angle>cfg_.command_change_angle;
    out.command_changed=changed;
    const bool target_changed=(previous_intent_-direction).norm()>1e-6;
    bool refresh=!observation_||changed||next.view!=observation_->view||
        ((next.version!=observation_->version||target_changed)&&now-last_update_>=cfg_.field_interval);
    if(!refresh && diagnostic_previous && velocity.norm()>.03 && now>diagnostic_time_) {
        Eigen::Vector2d measured_state;
        if(observation_->camera.pixelFromDirection(observation_->rotation.transpose()*measured,&measured_state)) {
            out.execution_mismatch=((measured_state-diagnostic_state_)/(now-diagnostic_time_)-diagnostic_rate_).norm();
            out.mismatch_valid=std::isfinite(out.execution_mismatch);
        }
    }
    if(refresh) {
        if (changed) target_holding_=false;
        out.refreshed=true;
        Eigen::Vector2d continued,source,goal;
        bool may_continue=cfg_.continuation&&!changed&&observation_&&continueAnchor(next,new_state,&continued);
        bool source_ok=false,goal_ok=false,old_chi_valid=false;double old_chi=0;
        if(observation_) {
            source_ok=next.camera.pixelFromDirection(next.rotation.transpose()*observation_->rotation*observation_->camera.rayFromPixel(field_.source),&source);
            goal_ok=next.camera.pixelFromDirection(next.rotation.transpose()*observation_->rotation*observation_->camera.rayFromPixel(field_.goal),&goal);
            Eigen::Vector2d old_state;if(observation_->camera.pixelFromDirection(observation_->rotation.transpose()*measured,&old_state))
                old_chi_valid=box_.coordinate(field_,old_state,cfg_,&old_chi);
        }
        grid_refined_=false;
        bool built=build(next,position,direction,new_state,std::min(intent.norm(),cfg_.max_speed),now,
            !changed&&source_ok?&source:nullptr,!changed&&goal_ok?&goal:nullptr,velocity.norm());
        // Refine only a geometrically feasible but unresolved angular domain.
        // Keep physical depth unchanged: the denser chart has no observations.
        if(!built && cfg_.adaptive_grid && maximum_clearance_>required_prefix_ &&
            next.camera.width()<=48 && next.camera.height()<=36) {
            const Camera original=next.camera;
            Camera dense(2*original.width(),2*original.height(),original.horizontalFovDeg(),
                original.verticalFovDeg(),original.maxDepth());
            dense.setIntrinsics(2*original.fx(),2*original.fy(),2*original.cx()+.5,2*original.cy()+.5);
            auto physical=std::make_shared<DepthObservation>(next);physical->supporting_views.clear();
            next.supporting_views.push_back(physical);
            next.camera=dense;next.depth.assign(dense.rays().size(),0);
            new_state=2*new_state+Eigen::Vector2d::Constant(.5);
            if(source_ok)source=2*source+Eigen::Vector2d::Constant(.5);
            if(goal_ok)goal=2*goal+Eigen::Vector2d::Constant(.5);
            may_continue=false;grid_refined_=true;
            built=build(next,position,direction,new_state,std::min(intent.norm(),cfg_.max_speed),now,
                !changed&&source_ok?&source:nullptr,!changed&&goal_ok?&goal:nullptr,velocity.norm());
        }
        out.build_reason=build_reason_;out.required_prefix=required_prefix_;
        out.best_prefix=maximum_clearance_;out.free_directions=free_directions_;out.grid_refined=grid_refined_;
        if(!built) {reset();out.status="NO_REGULAR_FIELD";return out;}
        if(cfg_.response_preview&&velocity.norm()<=.03) {
            // At rest there is no measured travel direction to preserve. Seed
            // at the selected certified goal instead of a singular source or
            // blocked bilinear stencil near the operator's raw direction.
            // The actual residual velocity still enters the motion proof below.
            new_state=field_.goal;
            measured=next.rotation*next.camera.rayFromPixel(new_state);
            target_holding_=true;
        }
        if(velocity.norm()<=0.03&&!field_.segmentFree(new_state,new_state)) {
            double best=std::numeric_limits<double>::infinity();
            Eigen::Vector2d initial=field_.source;
            for(int y=0;y<field_.height;++y)for(int x=0;x<field_.width;++x) {
                const Eigen::Vector2d p=field_.offset+field_.spacing*Eigen::Vector2d(x,y);
                if(!field_.segmentFree(p,p))continue;
                const double distance=(p-new_state).squaredNorm();
                if(distance<best){best=distance;initial=p;}
            }
            if(!std::isfinite(best)){reset();out.status="NO_REGULAR_FIELD";return out;}
            new_state=initial;
            measured=next.rotation*next.camera.rayFromPixel(new_state);
            target_holding_=false;
        }
        field_position_=position;
        observation_.reset(new DepthObservation(next));last_update_=now;previous_intent_=direction;
        out.continued=may_continue&&box_.initialize(field_,continued,cfg_);
        double new_chi=0;Eigen::Vector2d j;
        if(out.continued&&!box_.evaluate(field_,new_state,cfg_,&new_chi,&j))out.continued=false;
        if(!out.continued) {
            out.reset_reason=changed?"COMMAND_CHANGE":"CONTINUATION_INFEASIBLE";
            box_.initialize(field_,new_state,cfg_);
            reference_intent_=direction;
        }
        if(old_chi_valid && box_.coordinate(field_,new_state,cfg_,&new_chi)) {
            out.jump=std::abs(new_chi-old_chi);out.jump_valid=true;
            out.jump_bound=2*cfg_.section_radius; // |chi| <= section radius on both valid boxes.
        }
        reference_world_.clear();
        if(box_.valid)reference_world_=referenceCurve(field_,box_.anchor,next,cfg_.minimum_gradient);
    }
    out.build_reason=build_reason_;out.required_prefix=required_prefix_;
    out.best_prefix=maximum_clearance_;out.free_directions=free_directions_;out.grid_refined=grid_refined_;
    if(!observation_||now-observation_->stamp>cfg_.observation_timeout){out.status="STALE_FIELD";return out;}
    const auto& o=*observation_;
    Eigen::Vector2d state;
    if(!o.camera.pixelFromDirection(o.rotation.transpose()*measured,&state)||!field_.contains(state)) {
        if(velocity.norm()<=0.03 && field_.segmentFree(field_.source,field_.source)) state=field_.source;
        else {out.status="STATE_OUTSIDE_FREE_DOMAIN";return out;}
    }
    out.state=state;out.field_residual=field_.residual;out.coupling_error=coupling_error_;
    Eigen::Vector2d candidate=state;
    // The plant tracks velocity with time constant tau. A dt-only heading
    // lead is attenuated by dt/tau every cycle. Preview its known response
    // time, then constrain command slew and certify the actual transition.
    // This is an execution-model extension, not literal paper Eq. (20).
    const double command_horizon=cfg_.response_preview?
        std::min(.5,cfg_.motion.velocity_tau+std::min(dt,.1)):std::min(dt,.1);
    const bool direct_intent=cfg_.intent_guard &&
        (o.rotation*o.camera.rayFromPixel(field_.goal)).dot(direction)>1.-1e-10 &&
        field_.segmentFree(state,field_.goal);
    if(direct_intent) {
        // Teleoperation goal capture: when q itself and the entire angular
        // approach are certified, stop following a now-unnecessary detour.
        // This remains one planned direction, rate-limited below and subject
        // to the same complete motion/braking proof as harmonic candidates.
        target_holding_=true;candidate=field_.goal;out.status="INTENT_DIRECT";
    } else if((cfg_.response_preview || target_holding_ || (state-field_.goal).norm()<=2.0) && field_.segmentFree(state,field_.goal)) {
        target_holding_=true; candidate=field_.goal;out.status="TARGET_HOLD";
    }
    else {
        Eigen::Vector2d u=field_.flow(state),j;double chi=0;
        const bool leaving_source=(state-field_.source).norm()<=2.5 &&
            (field_.value(state)>=1.0-1e-6 || u.norm()<cfg_.minimum_gradient);
        if(leaving_source) {
            // A rounded source can put the measured state on its wrong side.
            // Interpolated gradients in this boundary neighborhood are not a
            // regular flow box. Initialize through a checked short segment
            // toward a regular lower-potential cell, even if the interpolated
            // gradient happens to be nonzero.
            const double source_value=field_.value(state);
            double best=std::numeric_limits<double>::infinity();
            Eigen::Vector2d escape=state;
            for(int dy=-3;dy<=3;++dy) for(int dx=-3;dx<=3;++dx) {
                Eigen::Vector2d p=state+Eigen::Vector2d(dx,dy);
                if(!field_.segmentFree(state,p)||!smoothCell(field_,p)||field_.flow(p).norm()<cfg_.minimum_gradient ||
                    field_.value(p)>=source_value-1e-8) continue;
                double score=field_.value(p)+0.002*(p-field_.goal).norm();
                if(std::isfinite(score)&&score<best){best=score;escape=p;}
            }
            u=escape-state;
        }
        if(u.norm()<cfg_.minimum_gradient){out.status="SINGULAR_FIELD";return out;}
        if(!leaving_source && !box_.valid && smoothCell(field_,state) &&
            field_.flow(state).norm()>=cfg_.minimum_gradient) {
            // Initialization at a Dirichlet source need not admit a flow box.
            // Establish its reference once, on entering the first regular cell.
            if(box_.initialize(field_,state,cfg_)) {
                out.reset_reason="SOURCE_EXIT";
                reference_world_=referenceCurve(field_,state,o,cfg_.minimum_gradient);
            }
        }
        if(leaving_source) {
            box_.valid=false;reference_world_.clear();
            candidate=state+std::min(cfg_.response_preview?4.0:0.5,cfg_.chart_speed*command_horizon)*u.normalized();
            out.status="REFERENCE_INITIALIZATION";
        } else {
            if(!box_.evaluate(field_,state,cfg_,&chi,&j)) {
                // Eq. (24): a lost local flow box need not invalidate a safe,
                // regular current state. Re-anchor before demanding a stop.
                if(!box_.initialize(field_,state,cfg_)||!box_.evaluate(field_,state,cfg_,&chi,&j)) {
                    out.status="OUTSIDE_FLOW_BOX";return out;
                }
                reference_world_=referenceCurve(field_,state,o,cfg_.minimum_gradient);
                reference_intent_=direction;out.continued=false;out.reset_reason="TRACKING_RECOVERY";
            }
            out.tracking=true;out.chi=chi;out.ju=j.dot(u.normalized());out.jacobian_norm=j.norm();
            Eigen::Vector2d rate=cfg_.chart_speed*u.normalized();
            if(cfg_.feedback)rate-=j*(cfg_.transverse_gain*chi/j.squaredNorm());
            candidate=state+command_horizon*rate;out.status="TRACKING";
            diagnostic_state_=state;diagnostic_rate_=rate;diagnostic_time_=now;
        }
    }
    // Eq. (20): choose h <= dt continuously at the angular-rate bound.
    // Repeated halving made the effective turning rate jump by a factor of two
    // whenever a chart refresh moved an otherwise safe step across that bound.
    const double angular_limit=cfg_.motion.max_direction_rate*(cfg_.response_preview?command_horizon:dt);
    if(angularDistance(o.camera,state,candidate)>angular_limit) {
        const Eigen::Vector2d delta=candidate-state;
        double low=0,high=1;
        for(int i=0;i<12;++i) {
            const double middle=.5*(low+high);
            if(angularDistance(o.camera,state,state+middle*delta)<=angular_limit)low=middle;
            else high=middle;
        }
        candidate=state+low*delta;
    }
    if(cfg_.response_preview&&previous_command_.norm()>.03) {
        Eigen::Vector2d previous;
        if(o.camera.pixelFromDirection(o.rotation.transpose()*previous_command_.normalized(),&previous)) {
            const double slew=cfg_.motion.max_direction_rate*std::min(dt,.1);
            if(angularDistance(o.camera,previous,candidate)>slew) {
                const Eigen::Vector2d delta=candidate-previous;
                double low=0,high=1;
                for(int i=0;i<14;++i) {
                    const double middle=.5*(low+high);
                    if(angularDistance(o.camera,previous,previous+middle*delta)<=slew)low=middle;
                    else high=middle;
                }
                candidate=previous+low*delta;
            }
        }
    }
    bool safe_angle=false;
    for(int attempt=0;attempt<12;++attempt) {
        if(field_.segmentFree(state,candidate)&&angularDistance(o.camera,state,candidate)<=angular_limit+1e-8)
            {safe_angle=true;break;}
        candidate=0.5*(candidate+state);
    }
    if(!safe_angle){out.status="ANGULAR_STEP_BLOCKED";return out;}
    out.command_pixel=candidate;
    Eigen::Vector3d ray=o.rotation*o.camera.rayFromPixel(candidate);
    if(cfg_.intent_guard&&ray.dot(direction)<=1e-6) {
        out.status="INTENT_PROGRESS_BLOCKED";return out;
    }
    const double requested_speed=std::min(intent.norm(),cfg_.max_speed);
    const double scalar_step=cfg_.motion.max_accel*std::min(dt,.1);
    const double desired_speed=cfg_.smooth_speed?
        clip(requested_speed,std::max(0.,previous_command_.norm()-scalar_step),previous_command_.norm()+scalar_step):requested_speed;
    const double speed_certificate=std::max(.5,desired_speed*cfg_.motion.delay+
        desired_speed*desired_speed/(2*cfg_.motion.brake_accel)+.05);
    out.clearance=directionalClearance(o,position,ray,now,speed_certificate);
    const double residual=cfg_.adaptive_lookahead?cfg_.certificate_resolution:.05;
    const double available=std::max(0.,out.clearance-residual);
    const double at=cfg_.motion.brake_accel*cfg_.motion.delay;
    double speed=std::min(desired_speed,-at+std::sqrt(at*at+2*cfg_.motion.brake_accel*available));
    if(std::abs(ray.z())>1e-9)speed=std::min(speed,cfg_.max_vertical_speed/std::abs(ray.z()));
    // Limit scalar speed before safety validation; never smooth XYZ after
    // certification. Safety can always demand a faster reduction or a stop.
    const double minimum=cfg_.smooth_speed?1e-4:std::min(.05,requested_speed);
    auto accept=[&](double candidate_speed)->bool {
        if(candidate_speed<minimum-1e-9)return false;
        std::vector<std::pair<Eigen::Vector3d,double>> prefix;
        const Eigen::Vector3d command=candidate_speed*ray;
        if(!certifyMotion(o,position,velocity,command,now,cfg_.retain_verified_travel?&prefix:nullptr))return false;
        if(cfg_.retain_verified_travel)pending_motion_=std::move(prefix);
        out.command=command;out.accepted=true;
        diagnostic_valid_=out.tracking&&velocity.norm()>.03;return true;
    };
    if(accept(speed))return out;
    if(cfg_.smooth_speed) {
        // Prefer maintaining the last scalar speed to a needless 50% jump.
        double high=speed,low=std::min(speed,previous_command_.norm());
        bool found=low<high&&accept(low);
        if(!found) {
            low=.5*high;
            for(int i=0;i<10&&low>=minimum;++i) {
                if(accept(low)){found=true;break;}
                high=low;low*=.5;
            }
        }
        if(found) {
            // Feasibility need not be monotone: only explicitly certified
            // samples can replace the current safe result.
            for(int i=0;i<5&&high-low>.005;++i) {
                const double middle=.5*(low+high);
                if(accept(middle))low=middle;else high=middle;
            }
            return out;
        }
    } else {
        for(int i=0;i<11;++i){speed*=.5;if(accept(speed))return out;}
    }
    out.status="BRAKING_FALLBACK";return out;
}
} }
