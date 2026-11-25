#include "input_epoch.hpp"
#include "new_controller/navigation_io.hpp"
#include <future>
#include <iostream>
using namespace ego4p0;
static void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(int argc,char**argv) try {
  require(argc==2,"archive argument required");
  const std::filesystem::path archive=argv[1];
  uint32_t w,h;
  auto cal=navigation_calibration(archive/"simulation.sensors.json",w,h);
  NavigationPipeline pipeline;
  pipeline.configure(cal,w,h,1);
  NavigationObservation obs;
  pipeline.allocate(obs);
  navigation_read_depth(archive/"last_depth.bin",obs);
  navigation_identity(obs,100,1);
  NavigationState state;
  state.actual_valid=true;
  state.actual.t_ns=state.current_simulation_ns=obs.depth.t_ns;
  state.actual.v_B=nav_scale(Direction3{.9337847414874845,-.34912780877884925,.07845909572784494},12.);
  DesiredMotion desired{sphere_point(20*direction_pi/180,0),12,true,true};
  InputEpoch clock;
  clock.advance(state.current_simulation_ns);
  const auto before=clock.ticket();
  const auto &old=pipeline.process_if(obs,state,desired,false,true,[&]{
    // The input has passed validation and preprocessing. Switch timebase before
    // the field solve returns, exactly where a launch-time-only check races.
    auto writer=std::async(std::launch::async,[&]{auto lock=clock.lock();clock.transition(5000000,true);});
    writer.get();
    return NavigationPlan{};
  });
  require(old.angular_ready,"fixture should produce an old ready result");
  require(!clock.can_commit(before,obs.depth.t_ns,10000000),"obsolete result committed");
  pipeline.reset_epoch();
  navigation_identity(obs,1,1);obs.receive_epoch=1;
  state.actual.t_ns=state.current_simulation_ns=obs.depth.t_ns;
  const auto ticket=clock.ticket();
  require(pipeline.process(obs,state,desired,false,true).angular_ready,"first new input rejected");
  require(clock.can_commit(ticket,obs.depth.t_ns,10000000),"new result rejected");
  clock.advance(15000000);state.current_simulation_ns=15000000;
  const auto &duplicate=pipeline.process(obs,state,desired,false,true);
  require(duplicate.angular_ready && std::string(duplicate.cache_event)=="same_group" && duplicate.spatial_reused,"duplicate lost cache");
  require(clock.can_commit(ticket,obs.depth.t_ns,10000000),"inclusive freshness boundary");
  clock.advance(15000001);
  require(!clock.can_commit(ticket,obs.depth.t_ns,10000000),"expired result committed");
  std::cout<<"500000000 -> 5000000 during process_if: old ready discarded, first new ready, duplicate geometry reused, 10 ms inclusive\n";
  return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
