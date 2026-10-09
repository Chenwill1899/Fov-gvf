#include "pc_gvf/depth_angular_core.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
using namespace pc_gvf::depth_angular;
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
// Original exhaustive implementation: independent oracle and timing baseline.
std::vector<double> exhaustive(const BinaryMask& mask, int width, int height) {
    std::vector<Eigen::Vector2i> blocked;
    for (int y=0;y<height;++y) for (int x=0;x<width;++x)
        if (mask[y*width+x]) blocked.emplace_back(x,y);
    std::vector<double> result(mask.size(),0.0);
    for (int y=0;y<height;++y) for (int x=0;x<width;++x) {
        if (mask[y*width+x]) continue;
        if (blocked.empty()) {result[y*width+x]=std::hypot(double(x),double(y+1));continue;}
        int best=std::numeric_limits<int>::max();
        for (const auto& point:blocked) {
            const int dx=x-point.x(),dy=y-point.y();
            best=std::min(best,dx*dx+dy*dy);
        }
        result[y*width+x]=std::sqrt(double(best));
    }
    return result;
}
void check(const BinaryMask& mask,int width,int height) {
    require(euclideanDistanceToBlocked(mask,width,height)==exhaustive(mask,width,height),
        "clearance differs from exhaustive Euclidean oracle");
}
void checkGoal(const BinaryMask& mask,int width,int height,int sequence) {
    const auto free=std::find(mask.begin(),mask.end(),0);
    const auto blocked=std::find_if(mask.begin(),mask.end(),[](int b){return b!=0;});
    if(free==mask.end()||blocked==mask.end())return;
    const int si=int(free-mask.begin()),ri=int(blocked-mask.begin());
    const Eigen::Vector2d source(si%width,si/width),reference(ri%width,ri/width),previous(width-1.,height-1.);
    SimConfig cfg;cfg.clearance_reward=sequence%3==0?0.:.0135;cfg.goal_clearance_cap=sequence%3==1?2.:0.;
    const auto* history=sequence%2?&previous:nullptr;
    const auto result=chooseSafeGoal(reference,source,mask,width,height,history,cfg);
    require(result.valid,"free source must have a goal");
    const auto clearance=exhaustive(mask,width,height);
    // Existing source recapture can move even a free input toward history.
    const int selected_source=int(result.source.y())*width+int(result.source.x());
    double best=std::numeric_limits<double>::infinity();Eigen::Vector2d expected=source;
    for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
        const int index=y*width+x;
        if(result.labels[index]!=result.labels[selected_source])continue;
        const double dx=(x-reference.x())/width,dy=(y-reference.y())/height;
        double cost=dx*dx+dy*dy;
        cost-=cfg.clearance_reward*(cfg.goal_clearance_cap>0.?std::min(clearance[index],cfg.goal_clearance_cap):clearance[index]);
        if(history) {const double hx=(x-previous.x())/width,hy=(y-previous.y())/height;cost+=cfg.hysteresis_weight*(hx*hx+hy*hy);}
        cost+=cfg.deterministic_left_bias*(x-reference.x())/width;
        if(cost<best){best=cost;expected=Eigen::Vector2d(x,y);}
    }
    require(result.goal==expected,"candidate ranking differs from exhaustive clearance");
}
void regression() {
    for(unsigned bits=0;bits<(1u<<16);++bits) {
        BinaryMask mask(16);for(int i=0;i<16;++i)mask[i]=(bits>>i)&1;check(mask,4,4);
    }
    std::mt19937 random(20261006);
    for(const auto shape:{std::make_pair(1,1),{1,73},{91,1},{3,27},{31,5},{48,36},{64,48},{128,96}}) {
        const int w=shape.first,h=shape.second;
        for(int trial=0;trial<24;++trial) {
            BinaryMask mask(w*h);for(auto& b:mask)b=random()%100<unsigned((trial%6)*20)?255:0;
            if(trial==0)std::fill(mask.begin(),mask.end(),0);
            if(trial==1)std::fill(mask.begin(),mask.end(),255);
            if(trial==2){std::fill(mask.begin(),mask.end(),0);mask.back()=1;}
            check(mask,w,h);checkGoal(mask,w,h,trial);
        }
    }
    // The old int squared-distance calculation overflows on these thin grids.
    for(const auto shape:{std::make_pair(70001,1),{1,70001}}) {
        BinaryMask mask(70001,0);mask[0]=1;const auto values=euclideanDistanceToBlocked(mask,shape.first,shape.second);
        for(int i=0;i<70001;++i)require(values[i]==double(i),"large thin grid distance");
    }
    for(const auto shape:{std::make_pair(0,4),{-1,4},{4,0},{4,-1},{2,3}}) {
        bool threw=false;
        try{euclideanDistanceToBlocked(BinaryMask(4),shape.first,shape.second);}catch(const std::invalid_argument&){threw=true;}
        require(threw,"malformed grid must be rejected");
    }
    std::cout<<"clearance_check: 65536 exhaustive masks, 192 rectangular cases, 2 large thin grids, candidate ranking and invalid dimensions passed\n";
}
volatile double sink=0.;
template<typename Function> double measure(Function fn,const BinaryMask& mask,int w,int h) {
    const auto start=std::chrono::steady_clock::now();
    for(int repeat=0;repeat<16;++repeat){const auto values=fn(mask,w,h);sink+=values[(repeat*97)%values.size()];}
    return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/16.;
}
void benchmark() {
    std::mt19937 random(20261006);bool first=true;
    std::cout<<std::setprecision(10)<<"{\"unit\":\"microseconds_per_clearance_grid\",\"cases\":[";
    for(const auto shape:{std::make_pair(64,48),{128,96}})for(int density:{0,1,25,50,95,100}) {
        BinaryMask mask(shape.first*shape.second);for(auto& b:mask)b=random()%100<unsigned(density);
        check(mask,shape.first,shape.second);std::vector<double> old_times,new_times;
        for(int round=0;round<7;++round) {
            if(round%2==0)old_times.push_back(measure(exhaustive,mask,shape.first,shape.second));
            new_times.push_back(measure(euclideanDistanceToBlocked,mask,shape.first,shape.second));
            if(round%2!=0)old_times.push_back(measure(exhaustive,mask,shape.first,shape.second));
        }
        std::sort(old_times.begin(),old_times.end());std::sort(new_times.begin(),new_times.end());
        if(!first)std::cout<<',';first=false;
        std::cout<<"{\"width\":"<<shape.first<<",\"height\":"<<shape.second<<",\"blocked_percent\":"<<density
            <<",\"exhaustive_median_us\":"<<old_times[3]<<",\"optimized_median_us\":"<<new_times[3]<<",\"speedup\":"<<old_times[3]/new_times[3]<<'}';
    }
    std::cout<<"]}\n";
}
}
int main(int argc,char** argv) {
    try {
        if(argc==2&&std::string(argv[1])=="--benchmark")benchmark();
        else if(argc==1)regression();else throw std::invalid_argument("usage: clearance_check [--benchmark]");
    }catch(const std::exception& e){std::cerr<<"clearance_check: "<<e.what()<<'\n';return 1;}
}
