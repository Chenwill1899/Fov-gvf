#include "flight_supervisor.hpp"
#include <iostream>
#include <stdexcept>
using namespace ego4p3_sim;
namespace {void check(bool value,const char*what){if(!value)throw std::runtime_error(what);}template<class F>void rejects(F f,const char*what){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected,what);}}
int main(int argc,char**argv){try{
  const std::filesystem::path scenes=argc>1?argv[1]:std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"src/new_controller/config/isaac_closed_loop";
  if(argc>2){uint32_t width=0,height=0;const auto actual=navigation_calibration(argv[2],width,height);check(simulation_calibration_matches(actual)&&width==320&&height==240,"actual Isaac calibration read rejected");}
  for(const char*name:{"straight","climb","offset_obstacle","blocking_obstacle"}){auto s=Scenario::parse(YAML::LoadFile((scenes/(std::string(name)+".json")).string()));check(s.seed_certified&&s.contains(s.start,.3),"valid spawn rejected");check(!s.contains(nav_add(s.start,Vec{2,0,0}),.3),"seed extrapolated");}
  auto invalid=YAML::LoadFile((scenes/"straight.json").string());invalid["bounds_min"][0]=-1;rejects([&]{Scenario::parse(invalid);},"boundary seed accepted");
  invalid=YAML::LoadFile((scenes/"straight.json").string());invalid["boxes"]=YAML::Load("[{min: [-0.1, -0.1, 3.9], max: [0.1, 0.1, 4.1]}]");rejects([&]{Scenario::parse(invalid);},"obstacle seed accepted");
  invalid=YAML::LoadFile((scenes/"straight.json").string());invalid["static_scene"]=false;rejects([&]{Scenario::parse(invalid);},"dynamic history accepted");
  WireRequest request{};request.magic=0x45474F3450330001ULL;request.version=1;request.sequence=1;request.width=320;request.height=240;request.values[6]=request.values[10]=request.values[14]=1;
  check(validate_request(request,0,320,240).t_ns==0,"t0 rejected");request.simulation_ns=1;rejects([&]{validate_request(request,0,320,240);},"wrong time accepted");request.simulation_ns=0;request.values[0]=NAN;rejects([&]{validate_request(request,0,320,240);},"nonfinite state accepted");
  for(const Vec&direction:{Vec{1,0,0},Vec{.8,0,.6}}){SpatialReference reference;check(reference.build({direction},{0,0,0},{1,0,0,0,1,0,0,0,1}),"reference build failed");VariableReferenceTracker tracker;Vec position{},velocity{};check(tracker.attach(reference,position,velocity),"zero speed attach rejected");double peak=0;bool arrived=false;
    for(int i=0;i<2000;++i){const double distance=nav_norm(nav_add(nav_scale(direction,16),position,-1));const double target=distance<.25&&nav_norm(velocity)<.1?0:std::min(12.,std::sqrt(40*std::max(0.,distance-.12)));
      const auto command=tracker.step(position,velocity,target,.005);check(command.status==TrackingStatus::Ready,"variable tracker command rejected");check(nav_norm(nav_add(command.next_velocity_W,plant_response(velocity,command.plant_input_W,.005),-1))<1e-12,"plant parity");
      tracker.commit(command);velocity=command.next_velocity_W;position=nav_add(position,velocity,.005);peak=std::max(peak,nav_norm(velocity));
      if(distance<.25&&nav_norm(velocity)<.1){arrived=true;break;}
    }
    check(peak>11.99&&arrived,"0 to cruise to goal stop failed");
  }
  check(nav_norm(plant_response({12,0,0},{},.005))==11.25,"zero-input braking model");
  {
    std::array<TubeStatus,3>history{TubeStatus::Unknown,TubeStatus::ObservedClear,TubeStatus::PotentialObstacle};
    do{
      auto decision=static_evidence_union(TubeStatus::Unknown,false,history.size(),[&](size_t i){return history[i];});
      check(decision.status==TubeStatus::ObservedClear&&decision.source==EvidenceSource::History,"strict historical clear depends on frame order");
      size_t queries=0;
      for(auto current:{TubeStatus::PotentialObstacle,TubeStatus::OutsideObservationRange}){
        decision=static_evidence_union(current,true,history.size(),[&](size_t i){++queries;return history[i];});
        check(decision.status==current&&decision.source==EvidenceSource::Current&&queries==0,"current veto reversed by seed/history");
      }
    }while(std::next_permutation(history.begin(),history.end()));
    check(static_evidence_union(TubeStatus::Unknown,false,0,[](size_t){return TubeStatus::ObservedClear;}).status==TubeStatus::Unknown,"absent history fabricated a proof");
    check(static_evidence_union(TubeStatus::Unknown,false,1,[](size_t){return TubeStatus::PotentialObstacle;}).status==TubeStatus::PotentialObstacle,"unopposed history candidate hidden");
  }
  {
    auto scenario=Scenario::parse(YAML::LoadFile((scenes/"straight.json").string()));std::array<CameraCalibration,4>cal;
    for(int camera=0;camera<4;++camera){const double angle=camera*direction_pi/2,c=std::cos(angle),s=std::sin(angle);cal[camera].K={160,0,160,0,160,120,0,0,1};cal[camera].T_BC={s,0,c,0,-c,0,s,0,0,-1,0,0,0,0,0,1};cal[camera].near_clip_z_m=.1;}
    cal[0].near_clip_z_m=double(float(.1));check(simulation_calibration_matches(cal),"actual float USD near plane rejected");check(cal[0].near_clip_z_m==0.10000000149011612,"actual near plane replaced");
    auto bad_calibration=cal;bad_calibration[0].near_clip_z_m=.10001;check(!simulation_calibration_matches(bad_calibration),"changed near plane accepted");bad_calibration=cal;bad_calibration[0].T_BC[3]=1e-5;check(!simulation_calibration_matches(bad_calibration),"camera displacement accepted");
    {
      // A moving state exactly at its goal may preprocess fresh sensors, but
      // the required placeholder must never enter a field solve or tracker.
      BodyState moving;moving.p_W=scenario.goal;moving.v_B={3,0,0};const auto goal=goal_intent(scenario.goal,moving,12);
      check(!goal.reference_allowed&&goal.preprocessing_input.v_desired==0,"coincident moving goal permits nonzero reference");
      NavigationPipeline pipeline;pipeline.configure(cal,320,240,1);NavigationObservation packet;pipeline.allocate(packet);navigation_identity(packet,1,1);
      moving.t_ns=packet.depth.t_ns;packet.observation_pose=moving;
      for(auto&image:packet.depth.D)std::fill(image.begin(),image.end(),10.f);
      NavigationState actual;actual.actual=moving;actual.current_simulation_ns=moving.t_ns;actual.actual_valid=true;
      const auto&output=pipeline.process_if(packet,actual,goal.preprocessing_input,false,true,[&]{return NavigationPlan{goal.reference_allowed,false,nullptr};});
      check(output.input_ready&&!output.field_attempted&&!output.spatial_ready,"coincident goal placeholder entered field solver");
      check(nav_norm(plant_response({3,0,0},{},.005))==2.25,"coincident moving goal brake response");
    }
    FlightSupervisor supervisor(scenario,cal,1);supervisor.pipeline().builder().refresh_worker_schedulers();
    const auto policy=supervisor.pipeline().builder().worker_policies(),priorities=supervisor.pipeline().builder().worker_priorities(),cpus=supervisor.pipeline().builder().worker_cpus();
    for(int i=0;i<4;++i)check(policy[i]==SCHED_OTHER&&priorities[i]==0&&cpus[i]==i+1,"actual correctness worker declaration");
    BodyState state;state.p_W=scenario.start;bool history_used=false,seed_used=false;FlightResult result;
    for(uint64_t seq=1;seq<=130;++seq){
      state.t_ns=int64_t(seq-1)*BaseConfig::period_ns;supervisor.identify(seq,state);
      for(auto&image:supervisor.observation().depth.D)std::fill(image.begin(),image.end(),10.f);
      result=supervisor.step(state);check(result.decision!=6,"synthetic pipeline executable proof failed");check(result.flags&2,"current accepted reference not used");
      seed_used|=result.evidence.seed>0;history_used|=result.evidence.history>0;
      check(supervisor.ball(nav_add(state.p_W,Vec{0,0,3}),.3)==TubeStatus::Unknown,"vertical Unknown invented clear");
      check(supervisor.ball(nav_add(state.p_W,Vec{4,0,0}),.3)==TubeStatus::OutsideObservationRange,"range rejection lost");
      supervisor.retain();state.p_W=nav_add(state.p_W,result.next_velocity,.005);state.v_B=result.next_velocity;
    }
    check(seed_used&&history_used,"startup seed / retained raw near-field evidence not exercised");
    state.t_ns=130*BaseConfig::period_ns;supervisor.identify(131,state);for(auto&image:supervisor.observation().depth.D)std::fill(image.begin(),image.end(),.5f);
    result=supervisor.step(state);
    check(supervisor.ball(nav_add(state.p_W,Vec{1,0,0}),.3)==TubeStatus::PotentialObstacle,"current obstacle hidden by retained raw clear");
    // Frame-input rejection precedes both retained images and a prior certified
    // braking envelope. Do not execute a cached command on a missing image.
    if(result.decision!=6){state.p_W=nav_add(state.p_W,result.next_velocity,.005);state.v_B=result.next_velocity;}
    state.t_ns=131*BaseConfig::period_ns;supervisor.identify(132,state);supervisor.observation().identity.camera_mask=7;
    result=supervisor.step(state);check(result.decision==6&&std::string(result.reason)=="invalid_observation","missing camera reused history");
    // At the certified spawn, a contradictory CURRENT raw obstacle still wins
    // over the ground-truth spawn certificate; no evidence source reverses it.
    FlightSupervisor seed_supervisor(scenario,cal,1);BodyState start;start.p_W=scenario.start;seed_supervisor.identify(1,start);
    for(auto&image:seed_supervisor.observation().depth.D)std::fill(image.begin(),image.end(),.5f);
    seed_supervisor.step(start);check(seed_supervisor.ball(nav_add(start.p_W,Vec{1,0,0}),.3)==TubeStatus::PotentialObstacle,"current obstacle hidden by seed");
  }
  std::cout<<"PASS scenario seed/geometry rejection, protocol t0/time/nonfinite, intrinsic 3D zero-speed/cruise/goal-stop, plant parity, current-obstacle priority, raw history, Unknown/range/missing-frame rejection and actual OTHER worker declarations; no timing or Isaac-flight claim\n";return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
