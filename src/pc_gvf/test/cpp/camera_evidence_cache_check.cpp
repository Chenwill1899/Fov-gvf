#include "pc_gvf/paper_guidance.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <iostream>
#include <cstdlib>
using namespace pc_gvf::depth_angular;
namespace reference {
double clip(double x,double a,double b){return std::max(a,std::min(b,x));}
int idx(int x,int y,int w){return y*w+x;}
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
int main(){std::mt19937 gen(420);std::uniform_real_distribution<double>u(0,1);int decisions=0;
for(int pass=0;pass<30;++pass){int w=24+gen()%45,h=18+gen()%31;Camera c(w,h,60.+80.*u(gen),50.+70.*u(gen),20.);if(pass%2)c.setIntrinsics(8.+70.*u(gen),7.+70.*u(gen),c.cx()+u(gen),c.cy()-u(gen));
const auto b=c.frustumBounds(),n=c.frustumNormals();double expected[4]={(-.5-c.cx())/c.fx(),(c.width()-.5-c.cx())/c.fx(),(-.5-c.cy())/c.fy(),(c.height()-.5-c.cy())/c.fy()};
for(int i=0;i<4;++i)if(b[i]!=expected[i]||n[i]!=std::hypot(1.,expected[i]))std::exit(1);
for(int x=0;x<w;++x)if(c.pixelXEdge(x)!=(x-.5-c.cx())/c.fx()||c.pixelXEdge(x+1)!=(x+.5-c.cx())/c.fx())std::exit(1);
DepthObservation o(c,std::vector<double>(w*h,20.),Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),1);
if(pass%3)for(auto& d:o.depth)d=u(gen)<.02?0.:2.+18.*u(gen);
for(int k=0;k<2000;++k){Eigen::Vector3d p((u(gen)-.5)*25,(u(gen)-.5)*20,u(gen)*22);double r=u(gen);if(reference::sphereObserved(o,p,r)!=observedEnvelope(o,p,r)){std::cerr<<"containment decision changed\n";std::exit(1);}++decisions;}
}
std::cout<<decisions<<" old/new sphere decisions and exact camera bounds/norms/edges after intrinsic changes: PASS\n";}
