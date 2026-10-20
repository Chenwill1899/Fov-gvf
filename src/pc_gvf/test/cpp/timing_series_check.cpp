#include "pc_gvf/timing_series.hpp"
#include <iostream>
#include <limits>
#include <cstdlib>
using namespace pc_gvf::depth_angular;
int main(){TimingSeries s;for(int i=0;i<1000000;++i)s.add(i);if(s.count!=1000000||s.values.size()!=8192||s.mean()!=499999.5||s.maximum!=999999||s.percentile(.5)!=995904)std::exit(1);
s.add(-1);s.add(std::numeric_limits<double>::quiet_NaN());if(s.count!=1000000)std::exit(1);
TimingSeries t;for(int i=0;i<500;++i)t.add((i*37)%500);auto sorted=t.values;std::sort(sorted.begin(),sorted.end());for(double f:{0.,.5,.95,1.})if(t.percentile(f)!=sorted[size_t(std::nearbyint(f*(sorted.size()-1)))])std::exit(1);
std::cout<<"1M samples bounded at8192, lifetime mean/max, rolling exact order statistic, invalid samples: PASS\n";}
