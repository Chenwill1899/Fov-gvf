#include "pc_gvf/operator_intent.hpp"
#include <iostream>
#include <iomanip>
using namespace pc_gvf::depth_angular;
int main(){OperatorIntent input;input.update(Eigen::Vector3d::UnitX(),true);double angle,maximum=0.;int i=0,worst=-1;while(std::cin>>angle){Eigen::Vector3d raw(std::cos(angle),std::sin(angle),0);auto out=input.update(raw,true);const double e=std::atan2(out.cross(raw).norm(),out.dot(raw));if(e>maximum){maximum=e;worst=i;}++i;}std::cout<<std::setprecision(17)<<"samples="<<i<<" max_output_lag="<<maximum<<" worst_index="<<worst<<'\n';}
