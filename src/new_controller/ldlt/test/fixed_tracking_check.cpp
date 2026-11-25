#include "fixed_tracking_cases.hpp"
#include <iostream>

using namespace ego4p0;

int main(int argc,char **argv) {
  try {
    using tracking_fixture::require;
    tracking_fixture::run(argc==2?argv[1]:"",1,false,true);
    SpatialReference ref;
    tracking_fixture::build(ref,tracking_fixture::cases[0]);
    FixedReferenceTracker tracker;
    const Direction3 p{},v{12,0,0},zero{};
    require(tracker.attach(ref,p,zero,0.,2.,zero,zero)==TrackingStatus::SpeedEntryRequired,"reject stationary");
    require(tracker.attach(ref,{0,.101,0},v,0.,2.,v,zero)==TrackingStatus::OutsideAttachment,"reject lateral attachment");
    const auto tilted=nav_scale(sphere_point(5.01*direction_pi/180.,0.),12.);
    require(tracker.attach(ref,p,tilted,0.,2.,tilted,zero)==TrackingStatus::OutsideAttachment,"reject angular attachment");
    require(tracker.attach(ref,p,v,0.,2.,v,zero)==TrackingStatus::Ready,"valid attachment");
    require(tracker.step(p,v,.011).status==TrackingStatus::InvalidStep&&!tracker.attached(),"reject skipped physics step");
    require(tracker.attach(ref,p,v,0.,.059,v,zero)==TrackingStatus::Ready,"short reference attachment");
    require(tracker.step(p,v,.005).status==TrackingStatus::EndOfReference,"reject unchecked horizon");
    require(tracker.attach(ref,p,v,0.,2.,v,zero)==TrackingStatus::Ready,"reattach");
    require(tracker.step({0,.201,0},v,.005).status==TrackingStatus::OutsideTrackingTube,"reject excessive error");
    require(tracker.attach(ref,p,v,0.,2.,v,zero)==TrackingStatus::Ready,"reattach for bad actuator state");
    require(tracker.step(p,{10,0,0},.005).status==TrackingStatus::PlantAcceleration,"reject infeasible inverse");
    require(tracker.attach(ref,p,v,0.,2.,{16,0,0},zero)==TrackingStatus::Ready,"valid prior input");
    require(tracker.step(p,v,.005).status==TrackingStatus::CompensationLimit,"reject input slew violation");
    std::cout<<"PASS six fixed-reference closed loops, independent error and constraint failures\n";
    return 0;
  } catch(const std::exception &e) {
    std::cerr<<e.what()<<'\n';
    return 1;
  }
}
