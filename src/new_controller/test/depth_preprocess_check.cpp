#include "new_controller/depth_preprocess.hpp"
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <new>

static std::atomic<bool> count_allocations{false};
static std::atomic<size_t> allocations{0};
void *operator new(std::size_t n) {
  if(count_allocations.load()) ++allocations;
  if(void *p=std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete(void *p,std::size_t) noexcept { std::free(p); }
void *operator new[](std::size_t n) { return ::operator new(n); }
void operator delete[](void *p) noexcept { ::operator delete(p); }
void operator delete[](void *p,std::size_t) noexcept { ::operator delete(p); }
using namespace ego4p0;
static void require(bool condition,const char *message) { if(!condition) throw std::runtime_error(message); }
static bool equal_bits(float a,float b) { return std::memcmp(&a,&b,sizeof(float))==0; }

int main() {
  const float nan=std::numeric_limits<float>::quiet_NaN(),inf=std::numeric_limits<float>::infinity();
  {
    std::array<float,9> raw{1,2,3,4,nan,6,7,8,9},out{};
    const auto stats=fill_invalid_depth(raw.data(),out.data(),3,3);
    require(stats.invalid==1 && stats.filled==1 && stats.unresolved==0 && out[4]==5.f,"3x3 original-neighbor mean");
    for(size_t i=0;i<9;++i) if(i!=4) require(equal_bits(raw[i],out[i]),"valid pixels changed");
    require(std::isnan(raw[4]),"source was modified");
  }
  {
    std::array<float,4> raw{nan,2,4,8},out{};
    fill_invalid_depth(raw.data(),out.data(),2,2);
    require(out[0]==static_cast<float>(14./3.),"corner did not crop to image bounds");
  }
  {
    std::array<float,6> raw{nan,inf,-inf,0.f,-0.f,-3.f},out{};
    const auto stats=fill_invalid_depth(raw.data(),out.data(),3,2);
    require(stats.invalid==6 && stats.filled==0 && stats.unresolved==6,"all-invalid stats");
    require(std::memcmp(raw.data(),out.data(),sizeof(raw))==0,"unresolved bits not preserved");
  }
  {
    // The center must remain zero, even after its left neighbor is repaired.
    std::array<float,5> raw{8,0,0,0,16},out{};
    const auto stats=fill_invalid_depth(raw.data(),out.data(),5,1);
    require(out==std::array<float,5>{8,8,0,16,16},"repair propagated across original invalid pixels");
    require(stats.filled==2 && stats.unresolved==1,"non-propagating stats");
    fill_invalid_depth(raw.data(),out.data(),1,5);
    require(out==std::array<float,5>{8,8,0,16,16},"single-column edge handling");
  }
  {
    const float largest=std::numeric_limits<float>::max();
    std::array<float,9> raw;raw.fill(largest);raw[4]=nan;
    std::array<float,9> out{};fill_invalid_depth(raw.data(),out.data(),3,3);
    require(out[4]==largest,"neighbor sum overflowed in float");
    float positive=std::numeric_limits<float>::denorm_min(),zero=0;
    fill_invalid_depth(&positive,&zero,1,1);require(equal_bits(positive,zero),"positive subnormal changed");
    float missing=nan;fill_invalid_depth(&missing,&zero,1,1);require(equal_bits(missing,zero),"single invalid pixel changed");
  }
  {
    DepthFrame frame;frame.allocate(4);
    for(size_t i=0;i<4;++i) {
      frame.D[i]={static_cast<float>(i+1),nan,0.f,0.f};
      frame.repair[i]=fill_invalid_depth(frame.D[i].data(),frame.D_filled[i].data(),2,2);
      for(float z:frame.D_filled[i]) require(z==static_cast<float>(i+1),"cross-camera depth contamination");
    }
    bool rejected=false;
    try {fill_invalid_depth(frame.D[0].data(),frame.D[0].data(),2,2);}catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"in-place repair was accepted");
  }
  {
    std::array<CameraCalibration,4> cal;
    for(auto &c:cal) c.K={2,1,3,0,4,5,0,0,1}; // nonzero skew exercises full inverse
    PixelRayTables tables;require(tables.configure(cal,8,12),"initial ray build missing");
    const auto &ray=tables.at(0,5,9);
    require(std::abs(ray.b[0]-.5)<1e-12 && std::abs(ray.b[1]-1)<1e-12 && std::abs(ray.b[2]-1)<1e-12,"K inverse with skew");
    require(std::abs(ray.d[0]-1./3)<1e-12 && std::abs(ray.d[1]-2./3)<1e-12 && std::abs(ray.d[2]-2./3)<1e-12,"unit pixel direction");
    const auto *address=tables.camera(0).data();std::array<float,4> raw{nan,3,6,9},out{};
    allocations=0;count_allocations=true;
    bool regenerated=false;
    for(int i=0;i<1000;++i) {
      regenerated=regenerated || tables.configure(cal,8,12);
      fill_invalid_depth(raw.data(),out.data(),2,2);
      (void)tables.at(3,7,11);
    }
    count_allocations=false;
    require(!regenerated && tables.builds()==1 && address==tables.camera(0).data(),"unchanged K regenerated rays");
    require(allocations==0,"steady processing/query allocated memory");
    cal[2].K[0]=4;
    require(tables.configure(cal,8,12) && tables.builds()==2,"changed K did not rebuild");
    require(tables.at(0,5,9).b[0]!=tables.at(2,5,9).b[0],"camera-specific K ignored");
    bool rejected=false;cal[1].K.fill(0);
    try {tables.configure(cal,8,12);}catch(const std::invalid_argument&) {rejected=true;}
    require(rejected && tables.builds()==2,"singular K or atomic rebuild validation");
    rejected=false;
    try {(void)tables.at(4,0,0);}catch(const std::out_of_range&) {rejected=true;}
    require(rejected,"out-of-bounds ray query accepted");
  }
  std::cout<<"PASS: original-only 3x3, valid/unresolved bit preservation, NaN/Inf/zero/negative, cropped/1D borders, no diffusion, four-camera independence, overflow-safe mean, full K inverse/unit rays, cached rebuild/zero steady allocations\n";
}
