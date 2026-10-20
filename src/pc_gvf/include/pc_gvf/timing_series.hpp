#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
namespace pc_gvf { namespace depth_angular {
// Lifetime mean/max stay exact. Percentiles use the latest 8192 valid samples;
// complete solver CSV is streamed separately for exact offline percentiles.
struct TimingSeries {
    static constexpr std::size_t capacity=8192;
    void add(double v) {
        if(!std::isfinite(v)||v<0)return;
        if(values.size()<capacity)values.push_back(v);else values[next]=v;
        next=(next+1)%capacity;++count;sum+=v;maximum=std::max(maximum,v);
    }
    double mean() const {return count?sum/count:0.;}
    double percentile(double f) const {
        if(values.empty())return 0.;
        auto ordered=values;const auto i=std::min(ordered.size()-1,size_t(std::nearbyint(std::max(0.,std::min(1.,f))*(ordered.size()-1))));
        std::nth_element(ordered.begin(),ordered.begin()+i,ordered.end());return ordered[i];
    }
    std::vector<double> values;std::size_t next=0;std::uint64_t count=0;double sum=0.,maximum=0.;
};
} }
