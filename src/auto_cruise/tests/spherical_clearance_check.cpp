#include "../image_depth_clearance.hpp"
#include <iostream>
#include <random>

using namespace ego4p0;
namespace {
void require(bool condition,const char *message) {
  if(!condition)throw std::runtime_error(message);
}
constexpr int width=48,height=36;
constexpr double yaw=.84;
const Direction3 position{14.,-8.,3.},mount{.15,.1,-.05};

spherical::Radial world(Direction3 body) {
  return spherical::from_xyz(position[0]+std::cos(yaw)*body[0]-std::sin(yaw)*body[1],
    position[1]+std::sin(yaw)*body[0]+std::cos(yaw)*body[1],position[2]+body[2]);
}
}

int main() try {
  std::array<CameraCalibration,4> cal{};
  for(auto &c:cal) {
    c.K={38.,3.,24.,0.,35.,18.,0.,0.,1.};
    c.T_BC={0.,0.,1.,mount[0],1.,0.,0.,mount[1],0.,1.,0.,mount[2],0.,0.,0.,1.};
    c.near_clip_z_m=.15;
  }
  FrameObservationBuilder builder;
  builder.configure(cal,width,height,5.,5.,8,4,0,false,true,true,true,false,false);
  PixelRayTables rays;
  rays.configure(cal,width,height);
  CruiseDepthFrame frame;
  frame.depth.allocate(width*height);
  builder.allocate(frame.depth);
  const auto &geometry=builder.local_geometry();
  ImageDepthRig rig;
  rig.configure(geometry);
  frame.observation_pose.p_W=position;
  const double c=std::cos(yaw),s=std::sin(yaw);
  frame.observation_pose.R_WB={c,-s,0.,s,c,0.,0.,0.,1.};
  frame.prepare(geometry,rig,spherical::from_xyz(position[0],position[1],position[2]),yaw);
  const auto center=world(nav_add(mount,{1.,0.,0.}));
  auto classify=[&](spherical::Radial p,double r){return (*frame.raw)(p,r);};
  auto rebuild=[&](){
    ++frame.depth.sequence;
    builder.build(frame.depth,rays);
  };
  auto fill=[&](float value) {
    for(auto &image:frame.depth.D)std::fill(image.begin(),image.end(),value);
    frame.depth.D_filled=frame.depth.D;
  };
  fill(20.f);
  require(classify(center,.11)==TubeStatus::Unknown,"stale frame certified clear");
  rebuild();
  require(classify(center,.11)==TubeStatus::ObservedClear,"complete raw image should certify");
  require(classify(world(nav_add(mount,{.18,0.,0.})),.09)==TubeStatus::Unknown,"near clip ignored");
  require(classify(world(nav_add(mount,{1.,1.,0.})),.09)==TubeStatus::Unknown,"outside FOV certified");
  require(classify(world({4.,0.,0.}),.1)==TubeStatus::OutsideObservationRange,"range limit ignored");
  require(classify(center,0.)==TubeStatus::Unknown,"nonpositive radius certified");
  require(classify({{0.,2.},1.},.1)==TubeStatus::Unknown,"invalid latitude certified");

  auto corridor=[&](Direction3 body,Direction3 direction,double length,double radius){
    const auto axis=spherical::from_xyz(direction[0],direction[1],direction[2]).dir;
    return rig.direction_clearance(frame,geometry,world(body),{axis.az+yaw,axis.el},length,radius);
  };
  const auto test_start=nav_add(mount,{.9,0.,0.});
  require(corridor(test_start,{1,0,0},.6,.12)==TubeStatus::ObservedClear,"complete corridor not certified");
  require(corridor(mount,{1,0,0},.6,.12)==TubeStatus::Unknown,"blind start certified");
  fill(20.f);
  frame.depth.D[0][18*width+24]=0.f;
  for(int i=1;i<4;++i)frame.depth.D[i][18*width+24]=0.f;
  rebuild();
  require(corridor(test_start,{1,0,0},.6,.12)==TubeStatus::Unknown,"filled data hid raw hole");
  fill(20.f);
  frame.depth.D[2][18*width+26]=1.2f;
  rebuild();
  require(corridor(test_start,{1,0,0},.6,.12)==TubeStatus::PotentialObstacle,"nearby pixel grazes tube");
  fill(20.f);rebuild();

  const int principal=18*width+24;
  for(auto &im:frame.depth.D)im[principal]=std::numeric_limits<float>::quiet_NaN();
  rebuild();
  require(classify(center,.11)==TubeStatus::Unknown,"filled-only evidence certified clear");
  frame.depth.D_filled[3][principal]=1.f;
  rebuild();
  require(classify(center,.11)==TubeStatus::PotentialObstacle,"filled obstacle failed to veto");
  require(rig.surface_intersects(frame,geometry,center,.11),"filled surface missed");
  fill(20.f);
  frame.depth.D[3][principal]=.4f;
  frame.depth.D_filled[3][principal]=.4f;
  rebuild();
  require(classify(center,.11)==TubeStatus::PotentialObstacle,"other camera cleared occlusion");
  require(!rig.surface_intersects(frame,geometry,center,.11),"occlusion treated as surface contact");
  frame.depth.D[3][principal]=1.f;
  frame.depth.D_filled[3][principal]=20.f;
  rebuild();
  require(classify(center,.11)==TubeStatus::PotentialObstacle,"raw obstacle discarded");
  require(rig.surface_intersects(frame,geometry,center,.11),"raw surface missed");
  const double half_pixel=.5*(1./38.+std::hypot(3./(38.*35.),1./35.))+1e-9;
  const auto grazing=world(nav_add(mount,{1.,.05+half_pixel+5e-8,0.}));
  require(rig.surface_intersects(frame,geometry,grazing,.05),"contact band at tangency lost");

  // Independently reconstruct only the synthetic truth in Cartesian coordinates.
  // Scan every raw and repaired pixel, including angles outside the nominal cap.
  fill(20.f);
  std::mt19937 rng(491);
  std::uniform_real_distribution<double> range(.01,2.8),unit(-1.,1.);
  struct Sample { Direction3 point; double z; };
  std::vector<Sample> truth;
  for(unsigned i=0;i<4;++i)for(int v=0;v<height;++v)for(int u=0;u<width;++u) {
    if(rng()%73)continue;
    const float z=float(range(rng));
    (rng()%2 ? frame.depth.D[i] : frame.depth.D_filled[i])[v*width+u]=z;
    const double y=(v-18.)/35.,x=(u-24.-3.*y)/38.;
    truth.push_back({nav_add(mount,{z,z*x,z*y}),z});
  }
  rebuild();
  const RawDepthBallOracle baseline(frame.depth,geometry,frame.observation_pose);
  int hit_count=0;
  for(int i=0;i<4000;++i) {
    const Direction3 query=nav_add(mount,{range(rng)-.4,unit(rng)*1.5,unit(rng)});
    const double radius=.01+.25*std::abs(unit(rng));
    bool contact=false;
    for(const auto &sample:truth)if(nav_norm(nav_add(query,sample.point,-1.))<=radius+sample.z*half_pixel+1e-7) {
      contact=true;
      break;
    }
    require(rig.surface_intersects(frame,geometry,world(query),radius)==contact,
            "spherical veto differs from exhaustive metric truth");
    const auto destination=world(query);
    const auto old=baseline(spherical::to_xyz(destination),radius),current=classify(destination,radius);
    require(current!=TubeStatus::ObservedClear || old==TubeStatus::ObservedClear,
            "new clear permission lacked baseline raw coverage");
    require(old!=TubeStatus::PotentialObstacle || current==TubeStatus::PotentialObstacle,
            "baseline obstacle veto lost");
    hit_count+=contact;
  }
  int swept_hits=0;
  for(int i=0;i<1500;++i){
    const auto a=nav_add(mount,{range(rng)-.3,unit(rng),unit(rng)});
    Direction3 dir{unit(rng),unit(rng),unit(rng)};
    const double n=nav_norm(dir);
    for(auto &component:dir)component/=n;
    const double length=.65*std::abs(unit(rng)),radius=.03+.2*std::abs(unit(rng));
    bool hit=false;
    for(const auto &sample:truth){
      const auto delta=nav_add(sample.point,a,-1.);
      const double along=std::clamp(direction_dot(delta,dir),0.,length);
      if(nav_norm(nav_add(delta,dir,-along))<=radius+sample.z*half_pixel+1e-7){hit=true;break;}
    }
    const auto status=corridor(a,dir,length,radius);
    require((status==TubeStatus::PotentialObstacle)==hit,"continuous swept ray differs from exhaustive metric truth");
    swept_hits+=hit;
  }
  require(swept_hits>40 && swept_hits<1470,"random rays did not cover both outcomes");
  require(hit_count>200 && hit_count<3500,"randomized cases did not cover both outcomes");
  std::cout<<"PASS raw coverage, clip, skew, translated camera, yaw, holes and occlusion; "
           <<hit_count<<" contacts in 4000 exhaustive comparisons\n";
  return 0;
} catch(const std::exception &e) {
  std::cerr<<e.what()<<'\n';
  return 1;
}
