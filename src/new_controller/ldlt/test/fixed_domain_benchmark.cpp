#include "new_controller/grounded_system.hpp"
#include "new_controller/sparse_trial_backend.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace fs=std::filesystem;
using namespace ego4p0;
static void require(bool yes,const std::string &message){if(!yes)throw std::runtime_error(message);}
static std::vector<std::string> split(const std::string &line){std::vector<std::string> out;std::istringstream in(line);std::string part;while(std::getline(in,part,','))out.push_back(part);return out;}
static uint32_t u32(const std::string &s){size_t consumed=0;const auto value=std::stoull(s,&consumed);require(consumed==s.size()&&value<=UINT32_MAX,"invalid uint32 CSV value: "+s);return uint32_t(value);}
static double real(const std::string &s){size_t consumed=0;const auto value=std::stod(s,&consumed);require(consumed==s.size(),"invalid real CSV value: "+s);return value;}
static std::string quote(const std::string &value){std::string out="\"";for(char c:value){if(c=='\"')out+='\"';out+=c;}return out+'\"';}
static std::ifstream input(const fs::path &path,const std::string &header){std::ifstream in(path);require(bool(in),"cannot open "+path.string());std::string actual;require(bool(std::getline(in,actual))&&actual==header,"unexpected CSV header: "+path.string());return in;}
static std::ofstream output(const fs::path &path){fs::create_directories(path.parent_path());std::ofstream out(path);require(bool(out),"cannot write "+path.string());out<<std::setprecision(17);return out;}

struct Fixture {
  std::string name,layer,error;GroundedSystem system;
  std::vector<double> reference;std::vector<uint8_t> boundary;
  std::vector<double> source_phi;
  uint32_t source_nodes=0,raw_faces=0;
  int64_t load_ns=0,assembly_ns=0;bool skipped=false,ok=false;
};
static AdaptiveCoarseField load_adaptive(const fs::path &directory,uint32_t &raw_faces) {
  auto nodes=input(directory/"adaptive_nodes.csv","node,representative,refined,boundary,phi");
  std::vector<uint32_t> representative;std::vector<uint8_t> boundary,refined;std::vector<double> phi;std::string line;
  while(std::getline(nodes,line))if(!line.empty()) {const auto fields=split(line);require(fields.size()==5,"invalid adaptive node row");require(u32(fields[0])==phi.size(),"adaptive node IDs not contiguous");representative.push_back(u32(fields[1]));const auto r=u32(fields[2]),b=u32(fields[3]);require(r<=1&&b>=1&&b<=3,"invalid adaptive node label");refined.push_back(uint8_t(r));boundary.push_back(uint8_t(b));phi.push_back(real(fields[4]));require(std::isfinite(phi.back()),"nonfinite adaptive potential");}
  struct Face{uint32_t from,to;double weight;};std::vector<Face> faces;
  auto edges=input(directory/"adaptive_faces.csv","from,to,conductance");
  while(std::getline(edges,line))if(!line.empty()){const auto fields=split(line);require(fields.size()==3,"invalid adaptive face row");const Face e{u32(fields[0]),u32(fields[1]),real(fields[2])};require(e.from<phi.size()&&e.to<phi.size()&&e.from!=e.to&&e.weight>0&&std::isfinite(e.weight),"invalid adaptive face");faces.push_back(e);}
  require(faces.size()<=UINT32_MAX,"too many exported faces");AdaptiveCoarseField a;a.allocate(uint32_t(phi.size()));a.nodes=uint32_t(phi.size());a.edges=raw_faces=uint32_t(faces.size());
  a.representative=std::move(representative);a.boundary=std::move(boundary);a.refined=std::move(refined);a.phi=std::move(phi);a.refined_nodes=uint32_t(std::count(a.refined.begin(),a.refined.end(),uint8_t(1)));
  a.neighbor.resize(faces.size());a.weight.resize(faces.size());
  for(const auto &e:faces)++a.offset[e.from+1];
  for(uint32_t i=0;i<a.nodes;++i)a.offset[i+1]+=a.offset[i];
  auto cursor=a.offset;
  for(const auto &e:faces){const auto index=cursor[e.from]++;a.neighbor[index]=e.to;a.weight[index]=e.weight;}
  return a;
}
static FieldLayer load_fine(const fs::path &directory,const SphericalStencil &stencil) {
  auto in=input(directory/"fine_field.csv","cell,alpha_deg,beta_deg,state,boundary,phi,cameras,adaptive_owner");
  FieldLayer layer;layer.allocate(stencil.layout);std::string line;uint32_t count=0;
  while(std::getline(in,line))if(!line.empty()) {
    const auto fields=split(line);require(fields.size()==8,"invalid fine field row");const auto i=u32(fields[0]);require(i==count&&i<layer.phi.size(),"fine field IDs not contiguous");
    const double alpha=-180+(i%360+.5),beta=-90+(i/360+.5);require(std::abs(real(fields[1])-alpha)<1e-10&&std::abs(real(fields[2])-beta)<1e-10,"fine grid differs from fixed 1 degree stencil");
    const auto state=u32(fields[3]),boundary=u32(fields[4]),cameras=u32(fields[6]);require(state<=2&&boundary<=4&&cameras<=15,"invalid fine field labels");
    layer.phi[i]=real(fields[5]);layer.state[i]=uint8_t(state);layer.boundary[i]=uint8_t(boundary);layer.cameras[i]=uint8_t(cameras);
    if(boundary){require(state==uint8_t(CandidateStatus::Available),"active fine node is not observed available");require(std::isfinite(layer.phi[i]),"active fine potential is nonfinite");}++count;
  }
  require(count==stencil.layout.cells(),"incomplete fixed fine grid export");return layer;
}
static Fixture load_fixture(const fs::path &directory,const std::string &layer_name,const SphericalStencil &stencil) {
  Fixture f;f.name=directory.filename().string();f.layer=layer_name;const auto load_begin=field_now_ns();
  try {
    AdaptiveCoarseField a;FieldLayer l;
    if(layer_name=="coarse"){a=load_adaptive(directory,f.raw_faces);f.source_nodes=a.nodes;f.boundary=a.boundary;f.source_phi=a.phi;}
    else{l=load_fine(directory,stencil);f.source_nodes=uint32_t(std::count_if(l.boundary.begin(),l.boundary.end(),[](uint8_t b){return b!=0;}));f.boundary=l.boundary;f.source_phi=l.phi;}
    f.load_ns=field_now_ns()-load_begin;f.system.reserve(layer_name=="coarse"?a.nodes:uint32_t(l.phi.size()));const auto assembly_begin=field_now_ns();
    const bool assembled=layer_name=="coarse"?f.system.adaptive(a):f.system.layer(stencil,l);f.assembly_ns=field_now_ns()-assembly_begin;
    if(!assembled){f.error=f.system.error;f.skipped=f.source_nodes==0&&f.system.node.empty();return f;}
    for(const auto node:f.system.node){require(node<f.source_phi.size(),"grounded source node out of range");f.reference.push_back(f.source_phi[node]);require(std::isfinite(f.reference.back()),"grounded reference is nonfinite");}
    f.ok=true;
  }catch(const std::exception &e){f.error=e.what();f.load_ns=field_now_ns()-load_begin;}
  return f;
}
static void export_system(const fs::path &root,const Fixture &f) {
  const auto directory=root/"matrices"/f.name/f.layer;
  auto matrix=output(directory/"system_csr.csv");matrix<<"entry,row,column,value\n";
  for(uint32_t i=0;i<f.system.rhs.size();++i)for(int e=f.system.ptr[i];e<f.system.ptr[i+1];++e)matrix<<e<<','<<i<<','<<f.system.col[e]<<','<<f.system.val[e]<<'\n';
  auto rhs=output(directory/"system_rhs_reference.csv");rhs<<"unknown,source_node,rhs,diagonal,reference_phi\n";
  for(uint32_t i=0;i<f.system.rhs.size();++i)rhs<<i<<','<<f.system.node[i]<<','<<f.system.rhs[i]<<','<<f.system.diagonal[i]<<','<<f.reference[i]<<'\n';
  auto boundary=output(directory/"source_boundary.csv");boundary<<"source_node,boundary,original_phi\n";
  for(uint32_t i=0;i<f.boundary.size();++i)boundary<<i<<','<<uint32_t(f.boundary[i])<<','<<f.source_phi[i]<<'\n';
}
static SparseTrialMode parse_mode(const std::string &name) {
  for(const auto mode:{SparseTrialMode::EigenLDLT,SparseTrialMode::EigenLLT,SparseTrialMode::Cholmod,SparseTrialMode::Amg,SparseTrialMode::IC0,SparseTrialMode::SGS})if(name==sparse_trial_name(mode))return mode;
  throw std::invalid_argument("unknown solver mode: "+name);
}
static void header(std::ostream &out) {
  out<<"case,layer,solver,scenario,iteration,status,unknowns,nonzeros,factor_nonzeros,total_ms,backend_total_ms,reset_ms,import_ms,analysis_ms,factor_ms,solve_ms,verify_ms,iterations,residual,symbolic_reused,numeric_reused,a_scale,b_scale,max_phi_error,error\n";
}
static void export_solution(const fs::path &root,const Fixture &f,const std::string &solver,const std::string &scenario,uint32_t parity,double a_scale,double b_scale,const std::vector<double> &x,bool ok) {
  auto out=output(root/"solutions"/solver/f.name/f.layer/(scenario+"_parity"+std::to_string(parity)+".csv"));
  out<<"unknown,source_node,x,reference_phi,expected_scaled_phi,a_scale,b_scale,ok\n";
  for(uint32_t i=0;i<x.size();++i)out<<i<<','<<f.system.node[i]<<','<<x[i]<<','<<f.reference[i]<<','<<f.reference[i]*b_scale/a_scale<<','<<a_scale<<','<<b_scale<<','<<ok<<'\n';
}

int main(int argc,char **argv){try{
  require(argc==4||argc==5,"usage: fixed_domain_benchmark OLD_EXPORT_ROOT NEW_OUT REPEATS [all|eigen_ldlt|eigen_llt|cholmod|amgcl|ic0|sgs]");
  const fs::path old_root=argv[1],new_root=argv[2];const auto repeats=u32(argv[3]);require(repeats>=1&&repeats<=10000,"repeats must be in [1,10000]");require(fs::is_directory(old_root),"input root is not a directory");require(!fs::exists(new_root),"output directory already exists");
  std::vector<SparseTrialMode> modes;if(argc==5&&std::string(argv[4])!="all")modes.push_back(parse_mode(argv[4]));else modes={SparseTrialMode::EigenLDLT,SparseTrialMode::EigenLLT,SparseTrialMode::Cholmod,SparseTrialMode::Amg,SparseTrialMode::IC0,SparseTrialMode::SGS};
  std::vector<fs::path> cases;for(const auto &entry:fs::directory_iterator(old_root))if(entry.is_directory()&&fs::is_regular_file(entry.path()/"harmonic_meta.json"))cases.push_back(entry.path());std::sort(cases.begin(),cases.end());require(cases.size()==14,"expected exactly fourteen original exported cases");
  fs::create_directories(new_root);SphericalStencil stencil;stencil.configure(1.);std::vector<Fixture> fixtures;fixtures.reserve(cases.size()*2);
  auto manifest=output(new_root/"matrix_manifest.csv");manifest<<"case,layer,status,source_nodes,unknowns,nonzeros,raw_faces,load_ms,assembly_ms,error\n";
  size_t failures=0,skipped=0,solves=0;
  for(const auto &path:cases)for(const auto &layer:{"coarse","fine"}) {
    fixtures.push_back(load_fixture(path,layer,stencil));const auto &f=fixtures.back();
    manifest<<f.name<<','<<f.layer<<','<<(f.ok?"loaded":f.skipped?"skipped":"failed")<<','<<f.source_nodes<<','<<f.system.node.size()<<','<<f.system.val.size()<<','<<f.raw_faces<<','<<f.load_ns*1e-6<<','<<f.assembly_ns*1e-6<<','<<quote(f.error)<<'\n';
    if(f.ok)export_system(new_root,f);else if(!f.skipped)++failures;
  }
  auto timings=output(new_root/"timings.csv"),warmups=output(new_root/"warmups.csv");header(timings);header(warmups);
  for(const auto mode:modes)for(const auto &f:fixtures)for(const std::string scenario:{"cold","numeric_reuse","symbolic_reuse"}) {
    const std::string name=sparse_trial_name(mode);SparseTrialBackend backend(mode);std::vector<double> values=f.system.val,rhs=f.system.rhs,x(f.system.rhs.size(),.5);
    for(uint32_t iteration=0;iteration<repeats+3;++iteration) {
      const bool warm=iteration<3;const uint32_t ordinal=warm?iteration:iteration-3;auto &out=warm?warmups:timings;
      const auto parity=iteration%2;const double a_scale=scenario=="symbolic_reuse"&&parity?1.001:1.;const double b_scale=scenario=="numeric_reuse"?(parity?1.:.9):a_scale;
      if(!f.ok) {
        out<<f.name<<','<<f.layer<<','<<name<<','<<scenario<<','<<ordinal<<','<<(f.skipped?"skipped":"failed")<<','<<f.system.node.size()<<','<<f.system.val.size()<<",0,0,0,0,0,0,0,0,0,0,nan,0,0,"<<a_scale<<','<<b_scale<<",nan,"<<quote(f.error)<<'\n';if(!warm&&f.skipped)++skipped;continue;
      }
      for(size_t i=0;i<values.size();++i)values[i]=f.system.val[i]*a_scale;
      for(size_t i=0;i<rhs.size();++i)rhs[i]=f.system.rhs[i]*b_scale;
      std::fill(x.begin(),x.end(),.5);
      const auto begin=field_now_ns();int64_t reset_ns=0;
      if(scenario=="cold"){const auto reset_begin=field_now_ns();backend.reset();reset_ns=field_now_ns()-reset_begin;}
      bool ok=false;std::string exception;
      try{ok=backend.solve(f.system.ptr,f.system.col,values,rhs,x,1e-7);}catch(const std::exception &e){exception=e.what();}
      const auto total_ns=field_now_ns()-begin;const auto &s=backend.stats();double maximum=0.;
      for(size_t i=0;i<x.size();++i){const double error=std::abs(x[i]-f.reference[i]*b_scale/a_scale);if(!std::isfinite(error)){maximum=std::numeric_limits<double>::infinity();break;}maximum=std::max(maximum,error);}
      const std::string error=exception.empty()?s.error:exception;
      out<<f.name<<','<<f.layer<<','<<name<<','<<scenario<<','<<ordinal<<','<<(ok?"valid":"failed")<<','<<s.unknowns<<','<<s.nonzeros<<','<<s.factor_nonzeros<<','<<total_ns*1e-6<<','<<s.total_ns*1e-6<<','<<reset_ns*1e-6<<','<<s.import_ns*1e-6<<','<<s.analysis_ns*1e-6<<','<<s.factor_ns*1e-6<<','<<s.solve_ns*1e-6<<','<<s.verify_ns*1e-6<<','<<s.iterations<<','<<s.residual<<','<<s.symbolic_reused<<','<<s.numeric_reused<<','<<a_scale<<','<<b_scale<<','<<maximum<<','<<quote(error)<<'\n';
      if(!ok){++failures;std::cerr<<"FAILED "<<name<<' '<<f.name<<' '<<f.layer<<' '<<scenario<<" iteration="<<iteration<<": "<<error<<'\n';}
      if(!warm){++solves;if(ordinal<2||!ok)export_solution(new_root,f,name,scenario,parity,a_scale,b_scale,x,ok);}
    }
  }
  auto summary=output(new_root/"run_summary.txt");summary<<"cases="<<cases.size()<<"\nmatrices="<<fixtures.size()<<"\nsolvers="<<modes.size()<<"\nrepeats="<<repeats<<"\nwarmups_per_valid_matrix_solver_scenario=3\nmeasured_solves="<<solves<<"\nmeasured_skipped_rows="<<skipped<<"\nfailures_including_warmups="<<failures<<"\n";
  std::cout<<"fixed-domain checks completed: solves="<<solves<<" skipped_rows="<<skipped<<" failures="<<failures<<" output="<<new_root<<'\n';return failures?1:0;
}catch(const std::exception &e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 2;}}
