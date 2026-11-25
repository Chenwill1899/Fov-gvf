#include "new_controller/depth_input.hpp"
#include <thread>
#include <iostream>
using namespace ego4p0;
void require(bool value) { if(!value) throw std::runtime_error("test assertion failed"); }
struct TestFrame { uint64_t sequence=0; std::array<uint64_t,256> pixels{}; };
int main() {
  TripleBuffer<TestFrame> p;
  require(p.latest()==nullptr);
  p.writer().sequence=1; p.writer().pixels.fill(1); p.publish();
  const auto *held=p.latest();
  for(uint64_t i=2;i<100;++i) { p.writer().sequence=i;p.writer().pixels.fill(i);p.publish(); }
  require(held->sequence==1 && held->pixels[100]==1);
  require(p.latest()->sequence==99);  // latest replaces unread intermediate data
  TripleBuffer<TestFrame> concurrent;
  std::atomic<bool> done{false};
  std::thread writer([&]{
    for(uint64_t i=1;i<=200000;++i) {
      auto &w=concurrent.writer();w.sequence=i;w.pixels.fill(i);concurrent.publish();
    }
    done.store(true,std::memory_order_release);
  });
  uint64_t previous=0, reads=0;
  do {
    if(const auto *f=concurrent.latest()) {
      const auto seq=f->sequence;
      require(seq>=previous);
      for(auto x:f->pixels) require(x==seq);
      std::this_thread::yield();
      require(f->pixels[127]==seq);  // writer cannot overwrite a borrowed frame
      previous=seq; ++reads;
    }
  } while(!done.load(std::memory_order_acquire));
  writer.join();require(concurrent.latest()->sequence==200000 && reads>0);
  FourWaySynchronizer<int> sync;
  FourWaySynchronizer<int>::Group g;
  for(int i:{2,0,3}) require(!sync.push(i,10,std::make_shared<int>(i),g));
  require(!sync.push(0,11,std::make_shared<int>(0),g));
  require(sync.push(1,10,std::make_shared<int>(1),g));
  require(!sync.push(0,10,std::make_shared<int>(0),g));
  require(sync.late_or_duplicate==1);
  for(int i=0;i<4;++i) { bool complete=sync.push(i,20,std::make_shared<int>(i),g);require(complete==(i==3)); }
  require(sync.partial_discarded==1);
  for(int i=21;i<100;++i) sync.push(0,i,std::make_shared<int>(0),g);
  require(sync.pending()==4);
  const auto body=body_state(0,0,std::sqrt(.5),std::sqrt(.5),{0,2,0},10);
  require(std::abs(body.v_B[0]-2)<1e-12 && std::abs(body.v_B[1])<1e-12);
  bool rejected=false;
  try {body_state(0,0,0,0,{0,0,0},0);}catch(const std::exception&){rejected=true;}
  require(rejected);
  std::cout<<"triple ownership, latest-only, 200000 concurrent publications, exact sync, bounded partials, state conversion, bad state rejection: PASS\n";
}
