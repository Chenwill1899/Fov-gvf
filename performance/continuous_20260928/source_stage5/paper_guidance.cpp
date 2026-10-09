#include "pc_gvf/paper_guidance.hpp"
#include <Eigen/Sparse>
#include <Eigen/LU>
#include <Eigen/IterativeLinearSolvers>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <functional>
#include <stdexcept>
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
    return radius<=tube.radius && segmentDistance(p,tube.a,tube.b)+radius<=tube.radius;
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
AngularField solve(AngularField f, const std::vector<double>& boundary) {
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
        Eigen::VectorXd x=solver.solve(rhs);
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
    valid=smoothCell(f,a)&&std::isfinite(section_value)&&u.norm()>=cfg.minimum_gradient;
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
bool observedEnvelope(const DepthObservation& o,const Eigen::Vector3d& center,double radius) {
    return sphereObserved(o,center,radius);
}
PaperGuidance::PaperGuidance(const PaperConfig& config) : cfg_(config) {
    const double positive[] = {cfg_.field_interval, cfg_.observation_timeout,
        cfg_.max_speed, cfg_.transverse_gain, cfg_.chart_speed, cfg_.minimum_lookahead,
        cfg_.minimum_gradient, cfg_.section_radius, cfg_.max_return_arc,
        cfg_.motion.brake_accel, cfg_.motion.max_accel, cfg_.motion.velocity_tau,cfg_.certificate_resolution};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("paper parameter must be finite and positive");
    const double nonnegative[] = {cfg_.motion.body_radius,cfg_.motion.safety_margin,
        cfg_.history_sample_interval,cfg_.history_uncertainty_rate,cfg_.history_duration,cfg_.motion.rollout_margin,cfg_.motion.delay,cfg_.motion.rollout_horizon,
        cfg_.motion.planning_horizon,cfg_.depth_uncertainty,cfg_.uncertainty_rate};
    for(double value:nonnegative)
        if(!std::isfinite(value)||value<0)throw std::invalid_argument("paper margin must be finite and nonnegative");
    if (cfg_.fine_width < 4 || cfg_.fine_height < 4 || cfg_.coarse_factor < 1 ||
        !std::isfinite(cfg_.max_vertical_speed) || cfg_.max_vertical_speed < 0)
        throw std::invalid_argument("invalid paper grid or vertical limit");
}
void PaperGuidance::reset() {
    diagnostic_valid_=false; target_holding_=false; observation_.reset(); field_=AngularField(); box_=FlowBox(); reference_world_.clear();
    previous_command_.setZero(); previous_intent_.setZero(); last_update_=-1;
}
void PaperGuidance::ingestObservation(const DepthObservation& observation) {
    if(cfg_.history_duration<=0||!std::isfinite(observation.stamp)||
        observation.depth.size()!=observation.camera.rays().size()||
        !observation.origin.allFinite()||!observation.rotation.allFinite())return;
    const auto previous_view=checked_observations_.find(observation.view);
    if(previous_view!=checked_observations_.end()&&observation.stamp<previous_view->second.first-.1) {
        history_.clear();checked_observations_.clear();seeds_.clear();verified_travel_.clear();pending_motion_.clear();verified_tubes_.clear();last_reached_time_=-1;reset();
    }
    const auto identity=std::make_pair(observation.stamp,observation.version);
    const auto checked=checked_observations_.find(observation.view);
    if(checked!=checked_observations_.end() && checked->second==identity)return;
    checked_observations_[observation.view]=identity;
    const auto history_before_expiration=history_.size();
    history_.erase(std::remove_if(history_.begin(),history_.end(),[&](const auto& old) {
        return observation.stamp-old->stamp>cfg_.history_duration;
    }),history_.end());
    expired_views_+=history_before_expiration-history_.size();
    bool retain=true;
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
    const auto tubes_before=verified_tubes_.size();
    verified_tubes_.erase(std::remove_if(verified_tubes_.begin(),verified_tubes_.end(),[&](const CertifiedTube& tube) {
        const double r=std::max(0.,tube.radius-cfg_.depth_uncertainty);
        const Eigen::Vector3d lo=tube.a.cwiseMin(tube.b)-Eigen::Vector3d::Constant(r);
        const Eigen::Vector3d hi=tube.a.cwiseMax(tube.b)+Eigen::Vector3d::Constant(r);
        for(const auto& point:hits) if((point.array()>=lo.array()).all()&&(point.array()<=hi.array()).all()&&
            segmentDistance(point,tube.a,tube.b)<r)return true;
        return false;
    }),verified_tubes_.end());
    revoked_tubes_+=tubes_before-verified_tubes_.size();
    std::vector<unsigned char> contradicted(history_.size(),0);
#ifdef _OPENMP
#pragma omp parallel for num_threads(4) schedule(static) if(history_.size()>=8)
#endif
    for(int index=0;index<static_cast<int>(history_.size());++index) {
        const auto& old=history_[index];
        const double uncertainty=cfg_.depth_uncertainty+cfg_.history_uncertainty_rate*std::max(0.0,observation.stamp-old->stamp);
        for(const auto& point:hits) {
            const Eigen::Vector3d p=old->rotation.transpose()*(point-old->origin);
            if(p.z()<=uncertainty)continue;
            const int u=static_cast<int>(std::floor(old->camera.fx()*p.x()/p.z()+old->camera.cx()+.5));
            const int v=static_cast<int>(std::floor(old->camera.fy()*p.y()/p.z()+old->camera.cy()+.5));
            if(u<0||v<0||u>=old->camera.width()||v>=old->camera.height()||
                old->depth[idx(u,v,old->camera.width())]<=p.z()+uncertainty)continue;
            if(observedEnvelope(*old,point,uncertainty)){contradicted[index]=1;break;}
        }
    }
    for(std::size_t i=0;i<history_.size();++i)if(contradicted[i])history_[i]->revoked=true;
    history_.erase(std::remove_if(history_.begin(),history_.end(),[](const auto& old) {
        return old->revoked;
    }),history_.end());
    // Check every fresh frame for contradictions, even between retained samples.
    if(!retain)return;
    auto retained=std::make_shared<DepthObservation>(observation);
    retained->supporting_views.clear();retained->obstacle_points.reset();history_.push_back(std::move(retained));
    if(history_.size()>32) {
        // Preserve spatial/view diversity, not simply the oldest timestamp.
        // This is a bounded retention heuristic; it never creates free volume.
        std::size_t victim=0;double redundancy=std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i+1<history_.size();++i)for(std::size_t j=i+1;j<history_.size();++j) {
            if(history_[i]->view!=history_[j]->view)continue;
            double score=(history_[i]->origin-history_[j]->origin).norm()+
                2.*(history_[i]->rotation-history_[j]->rotation).norm();
            if(score<redundancy){redundancy=score;victim=i;}
        }
        history_.erase(history_.begin()+victim);++evicted_views_;
    }
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
    DepthObservation evidence=input;
    evidence.supporting_views.insert(evidence.supporting_views.end(),history_.begin(),history_.end());
    const double radius=envelopeRadius();
    bool known=envelopeKnown(evidence,position,radius,now);
    if(!known)for(const auto& certificate:pending_motion_)
        if((position-certificate.first).norm()+radius<=certificate.second){known=true;break;}
    if(!known)return false;
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
    // A verified seed is fixed in world coordinates; it never follows the robot.
    // It is an explicit initialization premise, not an observation fabricated
    // at each control instant. Default ROS configuration does not enable it.
    auto covered=[&](const Eigen::Vector3d& p,double r) {
        for(auto it=verified_tubes_.rbegin();it!=verified_tubes_.rend();++it)
            if(tubeContains(*it,p,r))return true;
        if(pending)for(const auto& ball:pending_motion_)
            if((p-ball.first).norm()+r<=ball.second)return true;
        for(const auto& seed:seeds_)if((p-seed.first).norm()+r<=seed.second)return true;
        for(const auto& ball:verified_travel_)if((p-ball.first).norm()+r<=ball.second)return true;
        for(std::size_t i=1;i<verified_travel_.size();++i)
            if(twoBallUnionContains(p,r,verified_travel_[i-1],verified_travel_[i]))return true;
        for(std::size_t i=1;i<seeds_.size();++i)
            if(twoBallUnionContains(p,r,seeds_[i-1],seeds_[i]))return true;
        if(!o.revoked&&now-o.stamp<=cfg_.observation_timeout&&now>=o.stamp-.02&&observedEnvelope(o,p,r))return true;
        for(const auto& view:o.supporting_views) {
            const double age=now-view->stamp;
            if(view->revoked||age<-.02||age>cfg_.history_duration)continue;
            const double extra=cfg_.history_uncertainty_rate*std::max(0.0,o.stamp-view->stamp);
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
    if(o.supporting_views.empty()&&seeds_.empty()&&verified_travel_.empty()&&verified_tubes_.empty()&&!pending)return false;
    // Certify a union without assuming that one camera must see the whole body.
    // Each intersecting octree box is enclosed by a sphere and must be wholly
    // contained in actual evidence. Unresolved leaves remain unknown.
    int remaining=160;
    std::function<bool(const Eigen::Vector3d&,double,int)> certify;
    certify=[&](const Eigen::Vector3d& p,double half,int level) {
        if(((p-center).cwiseAbs()-Eigen::Vector3d::Constant(half)).cwiseMax(0).squaredNorm()>radius*radius)return true;
        if(covered(p,std::sqrt(3.0)*half))return true;
        if((p-center).squaredNorm()<radius*radius && !covered(p,0))return false;
        if(level>=5 || --remaining<=0)return false;
        const double h=.5*half;
        for(int z:{-1,1})for(int y:{-1,1})for(int x:{-1,1})
            if(!certify(p+h*Eigen::Vector3d(x,y,z),h,level+1))return false;
        return true;
    };
    return certify(center,radius,0);
}
bool PaperGuidance::segmentKnown(const DepthObservation& o,const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,double radius,double now,bool pending) const {
    // Exact containment in a convex stored capsule/ball needs both end balls.
    for(auto it=verified_tubes_.rbegin();it!=verified_tubes_.rend();++it)
        if(tubeContains(*it,a,radius)&&tubeContains(*it,b,radius))return true;
    auto in_ball=[&](const auto& ball) {
        return std::max((a-ball.first).norm(),(b-ball.first).norm())+radius<=ball.second;
    };
    for(const auto& ball:seeds_)if(in_ball(ball))return true;
    for(const auto& ball:verified_travel_)if(in_ball(ball))return true;
    if(pending)for(const auto& ball:pending_motion_)if(in_ball(ball))return true;
    // Refine only a failed enclosing-ball approximation. Bounded work and no
    // endpoint-only acceptance, even when both endpoints are individually free.
    std::function<bool(const Eigen::Vector3d&,const Eigen::Vector3d&,int)> certify;
    certify=[&](const Eigen::Vector3d& x,const Eigen::Vector3d& y,int level) {
        const double length=(y-x).norm();
        if(envelopeKnownImpl(o,.5*(x+y),radius+.5*length,now,pending))return true;
        if(length<=cfg_.certificate_resolution || level>=5)return false;
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
        if(t>=transition&&v.norm()<1e-6) return true;
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
    std::vector<double> clearances(w*h,0);
    // Each worker has a frame-local positive certificate cache. Geometry and
    // evidence are immutable until this parallel region has joined.
#ifdef _OPENMP
#pragma omp parallel for num_threads(4) schedule(static) if(o.supporting_views.size()>=4)
#endif
    for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x)
        clearances[idx(x,y,w)]=directionalClearance(o,position,o.rotation*o.camera.ray(x,y),now,lookahead+.05);
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
    for(int y=0;y<coarse.height;++y) for(int x=0;x<coarse.width;++x) {
        int i=idx(x,y,coarse.width);if(coarse.blocked[i]) continue;
        Eigen::Vector2d p=coarse.offset+factor*Eigen::Vector2d(x,y);
        double angle=std::acos(clip((o.rotation*o.camera.rayFromPixel(p)).dot(direction),-1,1));
        double history=goal_hint?(p-*goal_hint).squaredNorm():0;
        if(angle<best_angle-1e-9||(std::abs(angle-best_angle)<1e-9&&history<best_history))
            {goal=i;best_angle=angle;best_history=history;}
    }
    if(goal<0)return false;
    // Source and target must be disjoint. Put the source upstream when the
    // requested state already lies in the target neighborhood.
    if(source==goal) {
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
    coarse.goal=coarse.offset+factor*Eigen::Vector2d(goal%coarse.width,goal/coarse.width);
    Eigen::Vector2d exact_goal;
    if (o.camera.pixelFromDirection(o.rotation.transpose()*direction,&exact_goal) && coarse.contains(exact_goal) &&
        directionalClearance(o,position,direction,now,active_horizon+.05)>active_horizon)
        coarse.goal=exact_goal;
    std::vector<double> boundaries(coarse.blocked.size(),nan);boundaries[source]=1;boundaries[goal]=0;
    coarse=solve(coarse,boundaries);if(!coarse.valid){build_reason_="COARSE_FIELD_DEGENERATE";return false;}
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
    field_=solve(fine,fixed);coupling_error_=0;
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
    const double stopping=measured_speed*cfg_.motion.delay+
        measured_speed*measured_speed/(2*cfg_.motion.brake_accel);
    // Geometric/execution uncertainty is already in B_eff. The remaining
    // distance reserve is explicit and tied to certification resolution.
    const double required=cfg_.adaptive_lookahead?
        std::max(2*cfg_.certificate_resolution,stopping+cfg_.certificate_resolution):
        std::max(cfg_.minimum_lookahead,stopping);
    required_prefix_=required;
    double horizon=std::max(required,lookahead);
    for(int level=0;level<5;++level) {
        active_horizon=horizon;
        for(int i=0;i<w*h;++i)full[i]=clearances[i]<=horizon;
        free_directions_=std::count(full.begin(),full.end(),0);
        build_reason_=free_directions_?"BUILD_PENDING":"NO_CERTIFIED_DIRECTION";
        if(!free_directions_) {
            if(horizon<=required+1e-8)break;
            horizon=level==3?required:std::max(required,.5*horizon);continue;
        }
        const bool state_certified=measured_speed<=.03 ||
            directionalClearance(o,position,o.rotation*o.camera.rayFromPixel(state),now,horizon+.05)>horizon;
        if(state_certified && (attempt(factor)||(factor>1&&attempt(1)))) {field_lookahead_=horizon;return true;}
        if(!state_certified)build_reason_="STATE_DIRECTION_UNCERTIFIED";
        if(horizon<=required+1e-8)break;
        // Keep bounded work, but always evaluate the actual lower bound on
        // the final attempt. Five halvings alone may never reach it when a
        // large joystick magnitude requests a long horizon at near-zero speed.
        horizon=level==3?required:std::max(required,.5*horizon);
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
PaperResult PaperGuidance::step(const DepthObservation& input,const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,const Eigen::Vector3d& intent,double now,double dt) {
    for(auto& cache:envelope_caches_)cache.clear();
    cache_enabled_=true;
    struct CacheScope {bool& enabled;~CacheScope(){enabled=false;}} cache_scope{cache_enabled_};
    DepthObservation next=input;
    next.supporting_views=history_;
    next.supporting_views.insert(next.supporting_views.end(),input.supporting_views.begin(),input.supporting_views.end());
    PaperResult out;
    const bool diagnostic_previous=diagnostic_valid_;
    diagnostic_valid_=false;
    if(!position.allFinite()||!velocity.allFinite()||!intent.allFinite()||!std::isfinite(now)||!std::isfinite(dt)||dt<=0) {out.status="INVALID_STATE";return out;}
    if(intent.norm()<1e-5){reset();out.status="ZERO_INTENT";return out;}
    if(now-next.stamp>cfg_.observation_timeout||now<next.stamp-0.02){out.status="STALE_DEPTH";return out;}
    Eigen::Vector3d direction=intent.normalized();
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
    const bool changed=previous_intent_.norm()<0.5||std::acos(clip(previous_intent_.dot(direction),-1,1))>cfg_.command_change_angle;
    bool refresh=!observation_||changed||next.view!=observation_->view||
        (next.version!=observation_->version&&now-last_update_>=cfg_.field_interval);
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
        if(velocity.norm()<=0.03&&(!field_.segmentFree(new_state,new_state)||
            directionalClearance(next,position,next.rotation*next.camera.rayFromPixel(new_state),now,field_lookahead_+.05)<=field_lookahead_)) {
            double best=std::numeric_limits<double>::infinity();
            Eigen::Vector2d initial=field_.source;
            for(int y=0;y<field_.height;++y)for(int x=0;x<field_.width;++x) {
                const Eigen::Vector2d p=field_.offset+field_.spacing*Eigen::Vector2d(x,y);
                if(!field_.segmentFree(p,p)||directionalClearance(next,position,
                    next.rotation*next.camera.rayFromPixel(p),now,field_lookahead_+.05)<=field_lookahead_)continue;
                const double distance=(p-new_state).squaredNorm();
                if(distance<best){best=distance;initial=p;}
            }
            if(!std::isfinite(best)){reset();out.status="NO_REGULAR_FIELD";return out;}
            new_state=initial;
            measured=next.rotation*next.camera.rayFromPixel(new_state);
            target_holding_=false;
        }
        observation_.reset(new DepthObservation(next));last_update_=now;previous_intent_=direction;
        out.continued=may_continue&&box_.initialize(field_,continued,cfg_);
        double new_chi=0;Eigen::Vector2d j;
        if(out.continued&&!box_.evaluate(field_,new_state,cfg_,&new_chi,&j))out.continued=false;
        if(!out.continued) {
            out.reset_reason=changed?"COMMAND_CHANGE":"CONTINUATION_INFEASIBLE";
            box_.initialize(field_,new_state,cfg_);
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
    if((target_holding_ || (state-field_.goal).norm()<=2.0) && field_.segmentFree(state,field_.goal)) {
        target_holding_=true; candidate=field_.goal;out.status="TARGET_HOLD";
    }
    else {
        Eigen::Vector2d u=field_.flow(state),j;double chi=0;
        if(u.norm()<cfg_.minimum_gradient && (state-field_.source).norm()<=2.5) {
            // A source Dirichlet neighborhood can have zero gradient. Leave it
            // through a short, free segment into a regular lower-potential cell.
            double best=std::numeric_limits<double>::infinity();
            Eigen::Vector2d escape=state;
            for(int dy=-3;dy<=3;++dy) for(int dx=-3;dx<=3;++dx) {
                Eigen::Vector2d p=state+Eigen::Vector2d(dx,dy);
                if(!field_.segmentFree(state,p)||field_.flow(p).norm()<cfg_.minimum_gradient) continue;
                double score=field_.value(p)+0.002*(p-field_.goal).norm();
                if(std::isfinite(score)&&score<best){best=score;escape=p;}
            }
            u=escape-state;
        }
        if(u.norm()<cfg_.minimum_gradient){out.status="SINGULAR_FIELD";return out;}
        if(!box_.valid && smoothCell(field_,state) &&
            field_.flow(state).norm()>=cfg_.minimum_gradient) {
            // Initialization at a Dirichlet source need not admit a flow box.
            // Establish its reference once, on entering the first regular cell.
            if(box_.initialize(field_,state,cfg_)) {
                out.reset_reason="SOURCE_EXIT";
                reference_world_=referenceCurve(field_,state,o,cfg_.minimum_gradient);
            }
        }
        if(!box_.evaluate(field_,state,cfg_,&chi,&j)) {
            // Source/target neighborhoods are not regular flow boxes. Only a
            // checked short initialization step may leave a source neighborhood.
            if((state-field_.source).norm()>2.5){out.status="OUTSIDE_FLOW_BOX";return out;}
            candidate=state+std::min(0.5,cfg_.chart_speed*dt)*u.normalized();
            out.status="REFERENCE_INITIALIZATION";
        } else {
            out.tracking=true;out.chi=chi;out.ju=j.dot(u.normalized());out.jacobian_norm=j.norm();
            Eigen::Vector2d rate=cfg_.chart_speed*u.normalized();
            if(cfg_.feedback)rate-=j*(cfg_.transverse_gain*chi/j.squaredNorm());
            candidate=state+std::min(dt,0.1)*rate;out.status="TRACKING";
            diagnostic_state_=state;diagnostic_rate_=rate;diagnostic_time_=now;
        }
    }
    bool safe_angle=false;
    for(int attempt=0;attempt<12;++attempt) {
        if(field_.segmentFree(state,candidate)&&angularDistance(o.camera,state,candidate)<=cfg_.motion.max_direction_rate*dt+1e-8)
            {safe_angle=true;break;}
        candidate=0.5*(candidate+state);
    }
    if(!safe_angle){out.status="ANGULAR_STEP_BLOCKED";return out;}
    out.command_pixel=candidate;
    Eigen::Vector3d ray=o.rotation*o.camera.rayFromPixel(candidate);
    const double requested_speed=std::min(intent.norm(),cfg_.max_speed);
    const double speed_certificate=std::max(.5,requested_speed*cfg_.motion.delay+
        requested_speed*requested_speed/(2*cfg_.motion.brake_accel)+.05);
    out.clearance=directionalClearance(o,position,ray,now,speed_certificate);
    const double residual=cfg_.adaptive_lookahead?cfg_.certificate_resolution:.05;
    const double available=std::max(0.,out.clearance-residual);
    const double at=cfg_.motion.brake_accel*cfg_.motion.delay;
    double speed=std::min(requested_speed,-at+std::sqrt(at*at+2*cfg_.motion.brake_accel*available));
    if(std::abs(ray.z())>1e-9)speed=std::min(speed,cfg_.max_vertical_speed/std::abs(ray.z()));
    // Recheck the actual published vector, including its braking continuation.
    for(int attempt=0;attempt<12;++attempt) {
        if(speed<std::min(.05,requested_speed)-1e-9)break;
        Eigen::Vector3d command=speed*ray;
        std::vector<std::pair<Eigen::Vector3d,double>> certified_prefix;
        if(certifyMotion(o,position,velocity,command,now,
                cfg_.retain_verified_travel?&certified_prefix:nullptr)) {
            if(cfg_.retain_verified_travel)pending_motion_=std::move(certified_prefix);
            out.command=command;out.accepted=true;previous_command_=command;
            diagnostic_valid_=out.tracking && velocity.norm()>.03;return out;
        }
        speed*=0.5;
    }
    out.status="BRAKING_FALLBACK";previous_command_.setZero();return out;
}
} }
