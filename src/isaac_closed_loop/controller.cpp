#include "flight_supervisor.hpp"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <iostream>
#include <sstream>
#include <map>

using namespace ego4p3_sim;
namespace {
struct Descriptor {int fd=-1;~Descriptor(){if(fd>=0)::close(fd);}};
std::string escaped(const std::string&s){std::string out;for(unsigned char c:s){if(c=='"'||c=='\\')out+='\\';if(c>=32)out+=char(c);}return out;}
void atomic_json(const std::filesystem::path&path,const std::string&text){auto temporary=path;temporary+=".tmp";{std::ofstream stream(temporary);stream<<text<<'\n';if(!stream)throw std::runtime_error("artifact write failed");}std::filesystem::rename(temporary,path);}
void wait_fd(int fd,short event,int64_t deadline){for(;;){const int64_t remaining=deadline-field_now_ns();if(remaining<=0)throw std::runtime_error("IPC bounded timeout");pollfd p{fd,event,0};const int n=::poll(&p,1,int(std::min<int64_t>(30000,(remaining+999999)/1000000)));if(n<0&&errno==EINTR)continue;if(n<0)throw std::runtime_error("IPC poll failed");if(n==0)continue;if(p.revents&(event|POLLHUP))return;throw std::runtime_error("IPC descriptor error");}}
bool read_exact(int fd,void*data,size_t size,bool allow_empty_eof,int64_t deadline){auto*p=static_cast<char*>(data);size_t read=0;while(read<size){wait_fd(fd,POLLIN,deadline);const ssize_t n=::recv(fd,p+read,size-read,0);if(n<0&&errno==EINTR)continue;if(n<0)throw std::runtime_error("IPC read failed");if(n==0){if(read==0&&allow_empty_eof)return false;throw std::runtime_error("IPC truncated request");}read+=size_t(n);}return true;}
void write_exact(int fd,const void*data,size_t size){const auto*p=static_cast<const char*>(data);size_t written=0;const int64_t deadline=field_now_ns()+30000000000LL;while(written<size){wait_fd(fd,POLLOUT,deadline);const ssize_t n=::send(fd,p+written,size-written,MSG_NOSIGNAL);if(n<0&&errno==EINTR)continue;if(n<=0)throw std::runtime_error("IPC write failed");written+=size_t(n);}}
template<class A>void json_array(std::ostream&o,const A&a){o<<'[';bool comma=false;for(auto x:a){if(comma)o<<',';comma=true;o<<x;}o<<']';}
std::string scheduling(NavigationPipeline&p,bool correctness){
  p.builder().refresh_worker_schedulers();int policy=0;sched_param parameter{};if(pthread_getschedparam(pthread_self(),&policy,&parameter))throw std::runtime_error("cannot read coordinator scheduler");
  const auto policies=p.builder().worker_policies(),priorities=p.builder().worker_priorities(),cpus=p.builder().worker_cpus();
  const int required=correctness?SCHED_OTHER:SCHED_FIFO,priority=correctness?0:5;
  if(policy!=required||parameter.sched_priority!=priority||sched_getcpu()!=0)throw std::runtime_error("coordinator scheduler verification failed");
  for(int i=0;i<4;++i)if(policies[i]!=required||priorities[i]!=priority||cpus[i]!=i+1)throw std::runtime_error("camera scheduler verification failed");
  std::ostringstream out;out<<"\"simulation_only\":true,\"correctness_only\":"<<(correctness?"true":"false")<<",\"algorithm_threads\":5,\"coordinator_name\":\"p3_sim_coord\",\"coordinator_policy\":"<<policy<<",\"coordinator_priority\":"<<parameter.sched_priority<<",\"coordinator_cpu\":"<<sched_getcpu()<<",\"worker_policies\":";json_array(out,policies);out<<",\"worker_priorities\":";json_array(out,priorities);out<<",\"worker_cpus\":";json_array(out,cpus);return out.str();
}
}
int main(int argc,char**argv){
  std::filesystem::path output;std::string socket_path;uint64_t frames=0;bool correctness=false,bound_socket=false;int exit_code=0;
  try{
    const uint16_t endian=1;if(*reinterpret_cast<const uint8_t*>(&endian)!=1)throw std::runtime_error("little endian IPC required");
    std::map<std::string,std::string>args;for(int i=1;i<argc;++i){std::string key=argv[i];if(key=="--correctness-only"){correctness=true;continue;}if(key!="--calibration"&&key!="--scenario"&&key!="--output"&&key!="--socket")throw std::invalid_argument("unknown argument "+key);if(++i==argc||args.count(key))throw std::invalid_argument("missing or duplicate argument");args[key]=argv[i];}
    for(const char*key:{"--calibration","--scenario","--output","--socket"})if(!args.count(key))throw std::invalid_argument(std::string("required ")+key);
    output=args["--output"];socket_path=args["--socket"];std::filesystem::create_directories(output);
    const Scenario scenario=Scenario::parse(YAML::LoadFile(args["--scenario"]));uint32_t w=0,h=0;const auto cal=navigation_calibration(args["--calibration"],w,h);
    if(w!=scenario.width||h!=scenario.height)throw std::runtime_error("actual calibration dimensions differ from scenario");
    if(!simulation_calibration_matches(cal))throw std::runtime_error("actual camera center or near plane differs from certified model");
    if(pthread_setname_np(pthread_self(),"p3_sim_coord"))throw std::runtime_error("thread naming failed");
    // The camera pool discovers permitted CPUs during construction. Narrow only
    // the coordinator AFTER workers have selected and verified CPUs 1..4.
    FlightSupervisor supervisor(scenario,cal,navigation_calibration_version(cal,w,h));
    cpu_set_t mask;CPU_ZERO(&mask);CPU_SET(0,&mask);if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))throw std::runtime_error("CPU0 affinity unavailable");
    if(!correctness)supervisor.pipeline().enable_fifo5();
    const auto thread_json=scheduling(supervisor.pipeline(),correctness);
    Descriptor server;server.fd=::socket(AF_UNIX,SOCK_STREAM|SOCK_CLOEXEC,0);if(server.fd<0)throw std::runtime_error("cannot create UNIX server");sockaddr_un address{};address.sun_family=AF_UNIX;
    if(socket_path.empty()||socket_path.size()>=sizeof(address.sun_path)||std::filesystem::exists(socket_path))throw std::runtime_error("socket path exists or is too long");
    std::copy(socket_path.begin(),socket_path.end(),address.sun_path);
    if(::bind(server.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address)))throw std::runtime_error("cannot bind private UNIX server");bound_socket=true;
    if(::chmod(socket_path.c_str(),0600)||::listen(server.fd,1))throw std::runtime_error("cannot prepare private UNIX server");
    const std::string metadata="{"+thread_json+",\"ipc_version\":1,\"scenario\":\""+escaped(args["--scenario"])+"\",\"calibration\":\""+escaped(args["--calibration"])+"\",\"seed_certified\":true,\"seed_radius_m\":"+std::to_string(scenario.seed_radius)+",\"history_limit_frames\":16,\"history_pose_spacing_m\":0.2,\"tracking\":\"variable_speed_intrinsic_normal_plane\",\"plant\":\"first_order_kinematic_zero_target_brake\",\"production_motion_enabled\":false,\"dt_s\":0.005,\"plant_tau_s\":0.1,\"plant_acceleration_mps2\":150,\"reference_speed_acceleration_mps2\":20,\"tube_radius_m\":0.3}";
    atomic_json(output/"execution_metadata.json",metadata);atomic_json(output/"controller_ready.json","{"+thread_json+",\"socket\":\""+escaped(socket_path)+"\",\"ready\":true}");
    std::ofstream csv(output/"controller.csv");csv<<std::setprecision(17);
    csv<<"sequence,simulation_ns,decision,flags,reason,tracking_status,replanned,stationary_available_start,inherited_stop,px,py,pz,vx,vy,vz,ux,uy,uz,next_vx,next_vy,next_vz,requested_speed,goal_distance,cross_track_m,reference_s,certified_ahead_m,current_proofs,history_proofs,seed_proofs,unknown_proofs,obstacle_vetoes,outside_proofs,current_obstacle_vetoes,history_obstacle_candidates,input_ready,angular_ready,geometry_ready,invalid,q0_source,field_status,curve_end,curve_size,field_version,curve_version,geometry_version,active_field_version,active_curve_version,active_geometry_version,q0x,q0y,q0z,preprocess_ms,capture_ms,field_ms,spatial_ms,control_ms,proof_ms,history_ms,controller_compute_ms,"<<navigation_detail_header()<<'\n';
    wait_fd(server.fd,POLLIN,field_now_ns()+60000000000LL);Descriptor client;client.fd=::accept4(server.fd,nullptr,nullptr,SOCK_CLOEXEC);if(client.fd<0)throw std::runtime_error("cannot accept simulator");
    for(;;){WireRequest request{};const auto deadline=field_now_ns()+30000000000LL;if(!read_exact(client.fd,&request,sizeof(request),true,deadline))break;
      const auto state=validate_request(request,frames,w,h);auto&observation=supervisor.observation();for(auto&image:observation.depth.D)read_exact(client.fd,image.data(),image.size()*sizeof(float),false,deadline);
      const auto begin=field_now_ns();supervisor.identify(request.sequence,state);auto result=supervisor.step(state);result.history_ns=supervisor.retain();result.total_ns=field_now_ns()-begin;
      const auto&o=supervisor.pipeline().output();const auto&field=supervisor.pipeline().field();const auto velocity=nav_rotate(state.R_WB,state.v_B);
      WireReply reply;reply.sequence=request.sequence;reply.simulation_ns=request.simulation_ns;reply.decision=result.decision;reply.flags=result.flags;
      for(int k=0;k<3;++k){reply.values[k]=result.input[k];reply.values[3+k]=result.next_velocity[k];}
      reply.values[6]=result.requested_speed;reply.values[7]=result.goal_distance;reply.values[8]=result.cross_track;reply.values[9]=result.progress;reply.values[10]=result.certified_ahead;reply.values[11]=result.total_ns/1e6;
      for(double x:reply.values)if(!std::isfinite(x))throw std::runtime_error("nonfinite controller reply");
      csv<<request.sequence<<','<<request.simulation_ns<<','<<result.decision<<','<<result.flags<<','<<result.reason<<','<<tracking_status_name(result.tracking_status)<<','<<result.replanned<<','<<result.bootstrap<<','<<result.inherited;
      for(const auto&vec:{state.p_W,velocity,result.input,result.next_velocity})for(double x:vec)csv<<','<<x;
      csv<<','<<result.requested_speed<<','<<result.goal_distance<<','<<result.cross_track<<','<<result.progress<<','<<result.certified_ahead<<','<<result.evidence.current<<','<<result.evidence.history<<','<<result.evidence.seed<<','<<result.evidence.unknown<<','<<result.evidence.obstacle<<','<<result.evidence.outside<<','<<result.evidence.current_obstacle<<','<<result.evidence.history_obstacle
         <<','<<o.input_ready<<','<<o.angular_ready<<','<<o.spatial_ready<<','<<navigation_invalid_name(o.invalid)<<','<<o.q0_source<<','<<(o.field_attempted?field_status_name(field.status):"not_requested")<<','<<(o.field_attempted?curve_end_name(field.termination):"not_requested")<<','<<(o.field_attempted?field.curve_size:0)<<','<<o.field_version<<','<<o.curve_version<<','<<o.geometry_version
         <<','<<result.active_field_version<<','<<result.active_curve_version<<','<<result.active_geometry_version;
      for(double x:supervisor.pipeline().intent().q0)csv<<','<<x;
      for(auto ns:{o.preprocessing_ns,o.capture_ns,o.field_ns,o.spatial_ns,result.control_ns,result.proof_ns,result.history_ns,result.total_ns})csv<<','<<ns/1e6;
      navigation_detail_csv(csv,supervisor.pipeline());csv<<'\n';if(!csv)throw std::runtime_error("controller CSV write failed");
      write_exact(client.fd,&reply,sizeof(reply));++frames;
      if(result.decision==6){csv.flush();throw std::runtime_error(result.reason);}
      if(frames%100==0)csv.flush();
    }
    csv.flush();atomic_json(output/"execution_summary.json","{\"simulation_only\":true,\"normal_eof\":true,\"frames\":"+std::to_string(frames)+",\"exit_code\":0}");
  }catch(const std::exception&e){std::cerr<<"simulation controller: "<<e.what()<<'\n';exit_code=2;if(!output.empty()){try{atomic_json(output/"execution_error.json","{\"error\":\""+escaped(e.what())+"\",\"frames\":"+std::to_string(frames)+",\"exit_code\":2}");}catch(...){}}}
  if(bound_socket)::unlink(socket_path.c_str());return exit_code;
}
