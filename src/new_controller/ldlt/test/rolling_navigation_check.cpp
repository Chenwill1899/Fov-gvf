#include "rolling_cases.hpp"
#include <iostream>

using namespace ego4p0;
int main(int argc,char **argv){
  try{
    if(argc<2||argc>3)throw std::invalid_argument("rolling_navigation_check ARCHIVE [CSV]");
    uint32_t width,height;
    const auto cal=navigation_calibration(std::filesystem::path(argv[1])/"simulation.sensors.json",width,height);
    NavigationPipeline pipeline;
    pipeline.configure(cal,width,height,1);
    NavigationObservation observation;
    pipeline.allocate(observation);
    rolling_fixture::run(pipeline,observation,argc==3?argv[2]:"",1,false);
    rolling_fixture::verify_faults(pipeline,observation);
    rolling_fixture::verify_mid_ramp_splice();
    const auto rejected_step=rolling_fixture::verify_steep_spatial_protection(pipeline,observation);
    std::cout<<"Steep spatial example rejected at step "<<rejected_step<<'\n';
    std::cout<<"PASS rolling model, geometry splice, held ramp, duplicates, outage, wall veto, reset and recovery\n";
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
