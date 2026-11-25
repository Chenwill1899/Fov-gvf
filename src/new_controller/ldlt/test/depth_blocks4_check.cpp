#include "new_controller/depth_observations.hpp"
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <new>
#include <numeric>

using namespace ego4p0;
static std::atomic<bool> counting{false};static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n){if(counting)++allocations;if(void *p=std::malloc(n?n:1))return p;throw std::bad_alloc();}
void operator delete(void *p)noexcept{std::free(p);}void operator delete(void *p,std::size_t)noexcept{std::free(p);}
void *operator new[](std::size_t n){return ::operator new(n);}void operator delete[](void *p)noexcept{::operator delete(p);}void operator delete[](void *p,std::size_t)noexcept{::operator delete(p);}
static void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
template<class T>static bool same_bits(T a,T b){return std::memcmp(&a,&b,sizeof(T))==0;}
static bool close(double a,double b){return (!std::isfinite(a)&&!std::isfinite(b))||std::abs(a-b)<=2e-12*std::max({1.,std::abs(a),std::abs(b)});}
static std::array<CameraCalibration,4> cameras(uint32_t w,uint32_t h){
  std::array<CameraCalibration,4> cal;
  for(uint32_t c=0;c<4;++c){const double angle=c*direction_pi/2,co=std::cos(angle),si=std::sin(angle);cal[c].K={.6*w,.13,w*.5+.17*c,0,.65*w,h*.5+.11*c,0,0,1};cal[c].T_BC={si,0,co,.2*co,-co,0,si,.2*si,0,-1,0,.02,0,0,0,1};}
  return cal;
}
static std::array<double,3> optical(const CameraCalibration &cal,double u,double v){const double by=(v-cal.K[5])/cal.K[4];return {(u-cal.K[2]-cal.K[1]*by)/cal.K[0],by,1.};}
static std::array<double,3> rotated(const CameraCalibration &cal,const std::array<double,3>&b){std::array<double,3> q{};for(uint32_t k=0;k<3;++k)q[k]=cal.T_BC[k*4]*b[0]+cal.T_BC[k*4+1]*b[1]+cal.T_BC[k*4+2]*b[2];return q;}
static DepthRepairStats manual_repair(const std::vector<float>&raw,std::vector<float>&filled,uint32_t w,uint32_t h){
  filled=raw;DepthRepairStats stats;
  for(uint32_t v=0;v<h;++v)for(uint32_t u=0;u<w;++u){const auto p=v*w+u;if(valid_depth(raw[p]))continue;++stats.invalid;double sum=0;uint32_t count=0;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){const int a=int(u)+dx,b=int(v)+dy;if(a<0||b<0||a>=int(w)||b>=int(h))continue;const float z=raw[uint32_t(b)*w+uint32_t(a)];if(valid_depth(z)){sum+=z;++count;}}
    if(count){filled[p]=float(sum/count);++stats.filled;}else ++stats.unresolved;
  }return stats;
}
static void compare(const DepthFrame &a,const DepthFrame &b){
  const auto &x=a.observations,&y=b.observations;
  require(x.block_edge==y.block_edge&&x.block_columns==y.block_columns&&x.block_rows==y.block_rows&&x.offsets==y.offsets&&x.cell_counts==y.cell_counts&&x.camera_coverage==y.camera_coverage&&x.source_count==y.source_count&&x.observed_cells==y.observed_cells,"serial/parallel/fused direction index differs");
  require(std::equal(x.sources.begin(),x.sources.begin()+x.source_count,y.sources.begin()),"provenance differs");
  require(x.valid_depth_pixels==y.valid_depth_pixels&&x.usable_blocks==y.usable_blocks&&x.camera_observations==y.camera_observations&&x.camera_covered_cells==y.camera_covered_cells,"camera source counts differ");
  for(uint32_t c=0;c<4;++c){require(a.D[c].size()==b.D[c].size()&&std::memcmp(a.D[c].data(),b.D[c].data(),a.D[c].size()*sizeof(float))==0,"raw images differ");require(std::memcmp(a.D_filled[c].data(),b.D_filled[c].data(),a.D_filled[c].size()*sizeof(float))==0,"filled images differ");
    require(a.repair[c].invalid==b.repair[c].invalid&&a.repair[c].filled==b.repair[c].filled&&a.repair[c].unresolved==b.repair[c].unresolved,"repair counts differ");
    for(uint32_t id=0;id<x.blocks_per_camera;++id){const auto &p=x.blocks[c][id],&q=y.blocks[c][id];require(same_bits(p.mean_z,q.mean_z)&&same_bits(p.min_z,q.min_z)&&same_bits(p.max_z,q.max_z)&&same_bits(p.raw_min_z,q.raw_min_z)&&same_bits(p.min_rho,q.min_rho)&&same_bits(p.raw_min_rho,q.raw_min_rho)&&p.valid_pixels==q.valid_pixels&&p.raw_valid_pixels==q.raw_valid_pixels&&p.direction_cell==q.direction_cell&&p.raw_complete==q.raw_complete&&p.mean_in_local_range==q.mean_in_local_range,"block summary differs across execution paths");}
    for(uint32_t id=0;id<x.tiles[c].size();++id){require(x.tiles[c][id].valid_pixels==y.tiles[c][id].valid_pixels&&same_bits(x.tiles[c][id].min_z,y.tiles[c][id].min_z),"filled exact-region tiles differ");require(x.raw_tiles[c][id].valid_pixels==y.raw_tiles[c][id].valid_pixels&&same_bits(x.raw_tiles[c][id].min_z,y.raw_tiles[c][id].min_z),"raw exact-region tiles differ");}
  }
  require(a.local.node_state==b.local.node_state&&a.local.excluded_camera_mask==b.local.excluded_camera_mask&&a.local.observation_cameras==b.local.observation_cameras&&a.local.complete_cameras==b.local.complete_cameras&&a.local.missing_prefix==b.local.missing_prefix,"panorama domain or completeness differs");
  require(std::memcmp(a.local.min_range.data(),b.local.min_range.data(),a.local.min_range.size()*sizeof(double))==0,"panorama range differs");
  require(a.local.near_blocks==b.local.near_blocks&&a.local.complete_far_blocks==b.local.complete_far_blocks&&a.local.unknown_blocks==b.local.unknown_blocks&&a.local.excluded_cells==b.local.excluded_cells&&a.local.available_cells==b.local.available_cells&&a.local.unknown_cells==b.local.unknown_cells,"local evidence counts differ");
}
static uint64_t checked_blocks=0,checked_rois=0,checked_radius_points=0;
static void manual_summaries(const DepthFrame &f,const FrameObservationBuilder &builder,const std::array<CameraCalibration,4>&cal,uint32_t w,uint32_t h,uint32_t edge){
  const auto &index=f.observations;const auto &table=builder.local_geometry().virtual_tables();const auto &layout=builder.layout();DepthObservationView view(f);
  std::vector<uint8_t> provenance(4*index.blocks_per_camera);
  for(uint32_t cell=0;cell<layout.cells();++cell)for(uint32_t k=index.offsets[cell];k<index.offsets[cell+1];++k){const uint32_t code=index.sources[k],camera=code/index.blocks_per_camera,id=code%index.blocks_per_camera;require(camera<4&&++provenance[code]==1,"representative provenance duplicated or invalid");const auto o=view.observation(cell,k-index.offsets[cell]);require(o&&o->camera==camera&&o->u0==id%index.block_columns*edge&&o->v0==id/index.block_columns*edge&&index.blocks[camera][id].direction_cell==cell,"full source CSR uses stale block3 coordinates");}
  for(uint32_t c=0;c<4;++c){std::vector<uint8_t> visits(w*h);uint32_t expected_near=0,expected_far=0,expected_unknown=0;
    for(uint32_t id=0;id<index.blocks_per_camera;++id){const uint32_t u0=id%index.block_columns*edge,v0=id/index.block_columns*edge,u1=std::min(u0+edge,w),v1=std::min(v0+edge,h);double sum=0,minrho=std::numeric_limits<double>::infinity(),rawrho=minrho;float minimum=std::numeric_limits<float>::infinity(),maximum=0,rawminimum=minimum;uint32_t valid=0,rawvalid=0;
      for(uint32_t v=v0;v<v1;++v)for(uint32_t u=u0;u<u1;++u){const uint32_t p=v*w+u;require(++visits[p]==1,"blocks overlap original pixels");const auto b=optical(cal[c],u,v);const double coefficient=std::hypot(b[0],b[1],b[2]);require(close(coefficient,table.coefficient(c,p)),"rho coefficient is not original pixel optical ray norm");const float raw=f.D[c][p],z=f.D_filled[c][p];if(valid_depth(raw)){++rawvalid;rawminimum=std::min(rawminimum,raw);rawrho=std::min(rawrho,double(raw)*coefficient);}if(!valid_depth(z))continue;++valid;sum+=z;minimum=std::min(minimum,z);maximum=std::max(maximum,z);const double rho=double(z)*coefficient;minrho=std::min(minrho,rho);
        if(rho<=ControllerLimits::local_range_m){const auto q=rotated(cal[c],b);const double norm=std::hypot(q[0],q[1],q[2]);for(uint32_t k=0;k<7;++k){std::array<double,3> point{};for(uint32_t a=0;a<3;++a)point[a]=rho*q[a]/norm;if(k)point[(k-1)/2]+=(k%2?-.1:.1);const auto cell=*layout.cell(std::atan2(point[1],point[0]),std::atan2(point[2],std::hypot(point[0],point[1])));require(f.local.excluded_camera_mask[cell]&(1U<<c),"near pixel plus .1m body radius escaped block4 precomputed template");++checked_radius_points;}}
      }
      const auto &block=index.blocks[c][id];const uint32_t total=(u1-u0)*(v1-v0);require(provenance[c*index.blocks_per_camera+id]==uint8_t(valid>0),"valid/empty block missing from or invented in provenance");require(block.valid_pixels==valid&&block.raw_valid_pixels==rawvalid,"block valid counts differ from raw pixel scan");require(valid?same_bits(block.mean_z,float(sum/valid))&&!std::isnan(block.min_z)&&block.min_z==minimum&&block.max_z==maximum:std::isnan(block.mean_z)&&std::isnan(block.min_z)&&std::isnan(block.max_z),"block mean/min/max differs from hand calculation");require(rawvalid?block.raw_min_z==rawminimum:std::isnan(block.raw_min_z),"raw optical minimum differs from hand scan");require(close(block.min_rho,minrho)&&close(block.raw_min_rho,rawrho),"block filled/raw rho minimum differs from independent rays");
      const bool complete=rawvalid==total&&std::isfinite(rawrho);require(block.raw_complete==complete,"partial/missing raw block was promoted to complete");
      if(valid){const auto observation=view.block(c,id);require(observation&&observation->u0==u0&&observation->u1==u1&&observation->v0==v0&&observation->v1==v1&&observation->total_pixels==total&&observation->u==(u0+u1-1)*.5&&observation->v==(v0+v1-1)*.5,"edge block provenance crosses or drops pixels");const auto q=rotated(cal[c],optical(cal[c],observation->u,observation->v));const auto cell=*layout.cell(std::atan2(q[1],q[0]),std::atan2(q[2],std::hypot(q[0],q[1])));require(block.direction_cell==cell&&table.block_cell(c,id)==cell,"block center rotation or precomputed cell differs");}else require(!view.block(c,id),"empty block fabricated a representative");
      if(valid&&std::isfinite(minrho)&&minrho<=3.)++expected_near;else if(complete&&rawrho>3.)++expected_far;else ++expected_unknown;++checked_blocks;
    }
    require(std::all_of(visits.begin(),visits.end(),[](uint8_t n){return n==1;}),"edge blocks lost original pixels");require(f.local.near_blocks[c]==expected_near&&f.local.complete_far_blocks[c]==expected_far&&f.local.unknown_blocks[c]==expected_unknown,"near/complete/unknown block classification differs from manual summary");
    for(uint32_t k=0;k<48;++k){const uint32_t u0=k%w,v0=(k*7)%h,u1=u0+1+(k*11)%(w-u0),v1=v0+1+(k*3)%(h-v0);for(const auto kind:{DepthImageKind::Raw,DepthImageKind::Filled}){const auto &image=(kind==DepthImageKind::Raw?f.D:f.D_filled)[c];std::optional<float> minimum;uint32_t valid=0;for(uint32_t v=v0;v<v1;++v)for(uint32_t u=u0;u<u1;++u){const auto z=image[v*w+u];if(valid_depth(z)){++valid;if(!minimum||z<*minimum)minimum=z;}}const auto query=view.region(c,{int(u0),int(v0),int(u1),int(v1)},kind);require(query.z==minimum&&query.valid_pixels==valid&&query.total_pixels==(u1-u0)*(v1-v0),"exact raw/filled ROI replaced by block mean");++checked_rois;}}
    const auto whole=view.region(c,{0,0,int(w),int(h)},DepthImageKind::Raw);require(whole.total_pixels==w*h,"whole original image query clipped edges");
    for(uint32_t p=0;p<w*h;p+=std::max(1U,w*h/31))for(const auto kind:{DepthImageKind::Raw,DepthImageKind::Filled}){const auto z=(kind==DepthImageKind::Raw?f.D:f.D_filled)[c][p];const auto query=view.pixel(c,p%w,p/w,kind);require(query.valid_pixels==uint32_t(valid_depth(z))&&bool(query.z)==valid_depth(z)&&(!query.z||*query.z==z),"exact pixel query differs from retained image");}
  }
  for(const uint32_t cell:table.active_cells()){const auto &sources=table.node(cell);for(uint32_t c=0;c<4;++c)if(sources.cameras&(1U<<c))require(bool(f.local.complete_cameras[cell]&(1U<<c))==index.blocks[c][sources.blocks[c]].raw_complete,"filled pixels changed raw camera completeness");}
}
int main(){try{
  float payload_nan;const uint32_t payload=0x7fc03691;std::memcpy(&payload_nan,&payload,4);const float inf=std::numeric_limits<float>::infinity();uint32_t shape_cases=0;
  for(uint32_t edge:{3U,4U})for(const auto shape:std::array<std::array<uint32_t,2>,8>{{{1,1},{2,5},{4,4},{5,7},{15,17},{17,19},{319,239},{320,240}}}){
    const uint32_t w=shape[0],h=shape[1],n=w*h,tile=w==320?16:7;const auto cal=cameras(w,h);PixelRayTables rays;rays.configure(cal,w,h);
    std::array<std::unique_ptr<FrameObservationBuilder>,4> builders;std::array<DepthFrame,4> frames;std::array<std::vector<float>,4> raw,expected;std::array<DepthRepairStats,4> repair;
    for(uint32_t c=0;c<4;++c){raw[c].resize(n);for(uint32_t p=0;p<n;++p){float z=4.f+c+float(p%31)*.03f;switch((p+5*c)%29){case 0:z=payload_nan;break;case 1:z=inf;break;case 2:z=-inf;break;case 3:z=0;break;case 4:z=-.1f;break;default:break;}raw[c][p]=z;}}
    std::fill(raw[2].begin(),raw[2].end(),payload_nan);raw[2][(h/2)*w+w/2]=.7f;
    raw[3][std::min(n-1U,w+1)]=.6f;
    for(uint32_t v=0;v<std::min(h,5U);++v)for(uint32_t u=0;u<std::min(w,5U);++u)raw[0][v*w+u]=payload_nan;
    for(uint32_t c=0;c<4;++c)repair[c]=manual_repair(raw[c],expected[c],w,h);
    for(uint32_t mode=0;mode<4;++mode){builders[mode]=std::make_unique<FrameObservationBuilder>();const bool parallel=mode&1,fused=mode&2;builders[mode]->configure(cal,w,h,1,1,tile,edge,5,parallel,true,fused,true,true,true);frames[mode].allocate(n);builders[mode]->allocate(frames[mode]);frames[mode].D=raw;frames[mode].sequence=1;frames[mode].t_ns=2000000;require(builders[mode]->process(frames[mode],rays),"first group was not processed");for(uint32_t c=0;c<4;++c){require(std::memcmp(frames[mode].D[c].data(),raw[c].data(),n*sizeof(float))==0,"raw original image overwritten");require(std::memcmp(frames[mode].D_filled[c].data(),expected[c].data(),n*sizeof(float))==0,"repair changed from original 3x3-neighborhood rule");require(frames[mode].repair[c].invalid==repair[c].invalid&&frames[mode].repair[c].filled==repair[c].filled&&frames[mode].repair[c].unresolved==repair[c].unresolved,"independent repair counts differ");}
      require(!builders[mode]->configure(cal,w,h,1,1,tile,edge,5,parallel,true,fused,true,true,true)&&builders[mode]->table_builds()==1,"unchanged block geometry regenerated tables");if(mode)compare(frames[0],frames[mode]);}
    if(w==320&&edge==4)require(frames[0].observations.block_columns==80&&frames[0].observations.block_rows==60&&frames[0].observations.blocks_per_camera==4800&&frames[0].observations.sources.size()==19200,"320x240 block4 layout does not have 19200 four-camera representatives");
    DepthFrame prefilled;prefilled.allocate(n);builders[0]->allocate(prefilled);prefilled.D=raw;prefilled.D_filled=expected;prefilled.repair=repair;prefilled.sequence=1;prefilled.t_ns=2000000;require(builders[0]->build(prefilled,rays),"already-filled reference path rejected block4");compare(frames[0],prefilled);
    manual_summaries(frames[0],*builders[0],cal,w,h,edge);
    allocations=0;counting=true;
    for(uint32_t sequence=2;sequence<=4;++sequence)for(uint32_t mode=0;mode<4;++mode){frames[mode].sequence=sequence;require(builders[mode]->process(frames[mode],rays),"new group was not processed");require(!builders[mode]->process(frames[mode],rays),"same synchronized group rebuilt");}
    counting=false;require(allocations==0,"steady block4 processing allocated memory");for(uint32_t mode=1;mode<4;++mode)compare(frames[0],frames[mode]);
    for(uint32_t mode=0;mode<4;++mode){for(auto &image:frames[mode].D)std::fill(image.begin(),image.end(),payload_nan);++frames[mode].sequence;builders[mode]->process(frames[mode],rays);require(frames[mode].local.available_cells==0&&frames[mode].local.unknown_cells==builders[mode]->layout().cells()&&frames[mode].local.excluded_cells==0,"all-missing frame fabricated free space or retained stale obstacle");}
    for(uint32_t mode=1;mode<4;++mode)compare(frames[0],frames[mode]);
    ++shape_cases;
  }
  bool rejected=false;FrameObservationIndex storage;DirectionGridLayout grid;grid.configure(1,1);try{storage.allocate(4,4,grid,16,2);}catch(const std::invalid_argument&){rejected=true;}require(rejected,"unsupported block size silently accepted");
  std::cout<<"PASS: "<<shape_cases<<" shape/block cases (3 baseline and 4), "<<checked_blocks<<" hand-scanned blocks, "<<checked_rois<<" exact ROIs, "<<checked_radius_points<<" near/radius template samples; raw 3x3 repair/payload, fused/unfused serial/four-worker equivalence, edge coverage, raw completeness/Unknown and zero steady allocations\n";return 0;
}catch(const std::exception &e){counting=false;std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
