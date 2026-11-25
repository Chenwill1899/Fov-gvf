#include "new_controller/fixed_reference_tracker.hpp"
#include <iostream>
#include <random>

using namespace ego4p0;

namespace {
void require(bool value,const char *message) {
  if(!value)throw std::runtime_error(message);
}

void close(const Direction3 &a,const Direction3 &b,double tolerance,const char *message) {
  require(nav_norm(nav_add(a,b,-1.))<=tolerance,message);
}

// Independent, pre-change Cartesian projector.  It has no dependency on the
// normal-frame construction or the new two-coordinate feedback implementation.
Direction3 cartesian_perpendicular(const Direction3 &error,const Direction3 &tangent) {
  const double along=error[0]*tangent[0]+error[1]*tangent[1]+error[2]*tangent[2];
  return {error[0]-along*tangent[0],error[1]-along*tangent[1],error[2]-along*tangent[2]};
}

void check_projection(Direction3 tangent,const Direction3 &error) {
  require(unit_direction(tangent),"unit tangent");
  const TrackingNormalPlane2D frame(tangent);
  const auto first=frame.lift({1.,0.}),second=frame.lift({0.,1.});
  require(std::abs(direction_dot(first,tangent))<8e-16&&
          std::abs(direction_dot(second,tangent))<8e-16&&
          std::abs(direction_dot(first,second))<8e-16,
          "normal frame orthogonality");
  require(std::abs(direction_dot(first,first)-1.)<1e-15&&
          std::abs(direction_dot(second,second)-1.)<1e-15,
          "normal frame is orthonormal");
  const auto local=frame.coordinates(error);
  const auto expected=cartesian_perpendicular(error,tangent);
  const double scale=1.+nav_norm(error);
  close(frame.lift(local),expected,2e-15*scale,"2D lift equals Cartesian projector");
  require(std::abs(std::hypot(local[0],local[1])-nav_norm(expected))<2e-15*scale,
          "two-coordinate transverse norm");
  close(frame.lift({-2.*local[0],-2.*local[1]}),nav_scale(expected,-2.),
        4e-15*scale,"2D correction equals Cartesian feedback");
}

void check_straight_controller(Direction3 tangent,const Direction3 &error) {
  require(unit_direction(tangent),"controller tangent");
  SpatialReference reference;
  const std::array<double,9> identity{1,0,0,0,1,0,0,0,1};
  const Direction3 anchor{3.,-5.,7.};
  require(reference.build({tangent},anchor,identity),"3D straight reference");
  auto lateral=cartesian_perpendicular(error,tangent);
  if(nav_norm(lateral)>.08)lateral=nav_scale(lateral,.08/nav_norm(lateral));
  const auto position=nav_add(anchor,lateral);
  const auto velocity=nav_scale(tangent,12.);
  FixedReferenceTracker tracker;
  require(tracker.attach(reference,position,velocity,0.,4.,velocity,{})==TrackingStatus::Ready,
          "3D controller attach");
  const auto out=tracker.step(position,velocity,.005);
  require(out.status==TrackingStatus::Ready,"3D controller step");
  const auto actual_error=nav_add(position,anchor,-1.);
  const auto perpendicular=cartesian_perpendicular(actual_error,tangent);
  const auto correction=nav_scale(perpendicular,-2.);
  const auto expected=nav_add(nav_scale(tangent,std::sqrt(144.-direction_dot(correction,correction))),correction);
  close(out.desired_W,expected,2e-11,"world desired preserves Cartesian feedback");
  require(std::abs(nav_norm(out.desired_W)-12.)<2e-12,"3D desired speed");
  require(std::abs(nav_norm(out.next_velocity_W)-12.)<2e-12,"3D predicted speed");
  const auto restored=nav_add(velocity,nav_add(out.plant_input_W,velocity,-1.),.05);
  close(restored,out.next_velocity_W,2e-12,"inverse response preserved");
  if(std::abs(tangent[2])>.5)
    require(std::abs(out.desired_W[2])>5.&&std::abs(out.next_velocity_W[2])>5.,
            "vertical 3D motion must remain available");
}
}

int main() {
  try {
    const std::array<Direction3,10> edges{{
      {1,0,0},{0,1,0},{0,0,1},{0,0,-1},{1,1,1},{1,-1,-1},
      {1,1,1e-15},{1,1,-1e-15},{1e-15,-1e-15,1},{1e-15,1e-15,-1}
    }};
    for(const auto &tangent:edges) {
      check_projection(tangent,{.04,-.07,.11});
      check_projection(tangent,{1e100,-2e100,3e100});
      check_straight_controller(tangent,{.04,-.07,.11});
    }
    std::mt19937_64 random(0x45474f345033ULL);
    std::uniform_real_distribution<double> uniform(-1.,1.);
    for(unsigned i=0;i<50000;++i) {
      const Direction3 tangent{uniform(random),uniform(random),uniform(random)};
      const Direction3 error{uniform(random),uniform(random),uniform(random)};
      check_projection(tangent,error);
      if(i<1000)check_straight_controller(tangent,error);
    }
    std::cout<<"PASS 50020 projector comparisons and 1010 3D controller cases; poles, frame branch, climb and descent retained\n";
    return 0;
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
