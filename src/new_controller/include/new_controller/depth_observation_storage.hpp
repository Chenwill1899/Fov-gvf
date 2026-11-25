#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace ego4p0 {
constexpr double direction_pi=3.141592653589793238462643383279502884;

struct DirectionGridLayout {
  double azimuth_deg=1.,elevation_deg=1.;
  uint32_t azimuth_bins=0,elevation_bins=0;
  double azimuth_step=0.,elevation_step=0.;
  void configure(double alpha_deg,double beta_deg) {
    if(!std::isfinite(alpha_deg) || !std::isfinite(beta_deg) || alpha_deg<=0. || alpha_deg>360. || beta_deg<=0. || beta_deg>180.)
      throw std::invalid_argument("angular resolutions must be finite positive degrees within 360/180");
    const double a=std::ceil(360./alpha_deg),b=std::ceil(180./beta_deg);
    if(a*b>1048576.) throw std::invalid_argument("direction grid exceeds 1048576-cell allocation limit");
    azimuth_deg=alpha_deg;elevation_deg=beta_deg;
    azimuth_bins=static_cast<uint32_t>(a);elevation_bins=static_cast<uint32_t>(b);
    azimuth_step=alpha_deg*direction_pi/180.;elevation_step=beta_deg*direction_pi/180.;
  }
  uint32_t cells() const noexcept { return azimuth_bins*elevation_bins; }
  // [-pi,pi) x [-pi/2,pi/2]. +pi aliases -pi; upper beta belongs to last row.
  std::optional<uint32_t> cell(double alpha,double beta) const noexcept {
    if(!cells() || !std::isfinite(alpha) || !std::isfinite(beta) || alpha< -direction_pi || alpha>direction_pi || beta< -direction_pi/2 || beta>direction_pi/2)
      return std::nullopt;
    if(alpha==direction_pi) alpha=-direction_pi;
    const uint32_t a=std::min(static_cast<uint32_t>((alpha+direction_pi)/azimuth_step),azimuth_bins-1);
    const uint32_t b=std::min(static_cast<uint32_t>((beta+direction_pi/2)/elevation_step),elevation_bins-1);
    return b*azimuth_bins+a;
  }
};

struct DepthTile {
  float min_z=std::numeric_limits<float>::quiet_NaN();
  uint32_t valid_pixels=0;
};

struct BlockDepth {
  double min_rho=std::numeric_limits<double>::quiet_NaN(),raw_min_rho=std::numeric_limits<double>::quiet_NaN();
  float mean_z=std::numeric_limits<float>::quiet_NaN();
  float max_z=std::numeric_limits<float>::quiet_NaN();
  float min_z=std::numeric_limits<float>::quiet_NaN(),raw_min_z=std::numeric_limits<float>::quiet_NaN();
  uint32_t valid_pixels=0;
  uint32_t raw_valid_pixels=0;
  uint32_t direction_cell=std::numeric_limits<uint32_t>::max();
  bool mean_in_local_range=false,raw_complete=false;
};

// Owned with raw+filled images in ONE protected frame slot. V2 sources encode
// camera*blocks_per_camera + block_v*block_columns+block_u.
// No 3-D points, per-pixel validity mask, unknown/free label or nearest-camera Z.
struct FrameObservationIndex {
  DirectionGridLayout layout;
  uint32_t width=0,height=0,tile_edge=16,tile_columns=0,tile_rows=0;
  uint32_t block_edge=3,block_columns=0,block_rows=0,blocks_per_camera=0;
  std::vector<uint32_t> offsets,sources;
  std::vector<uint32_t> cell_counts;
  std::vector<uint8_t> camera_coverage; // actual source-camera bits per ANGULAR cell
  std::array<std::vector<DepthTile>,4> tiles;
  std::array<std::vector<DepthTile>,4> raw_tiles;
  std::array<std::vector<BlockDepth>,4> blocks;
  std::array<uint32_t,4> valid_depth_pixels{},geometry_rejected{},camera_observations{},camera_covered_cells{};
  std::array<uint32_t,4> usable_blocks{},exact_angle_observations{},fast_angle_observations{};
  std::array<int64_t,4> compression_ns{},projection_ns{},region_camera_ns{};
  uint64_t configuration_id=0;
  int64_t merge_ns=0,camera_work_ns=0,build_total_ns=0;
  uint32_t source_count=0,observed_cells=0;
  // V1 fields retained only for the frozen profiler's source compatibility.
  // V2 timing exports use camera_work_ns/merge_ns/build_total_ns and per-camera kernels.
  int64_t stamp_ns=0,direction_build_ns=0,region_build_ns=0;
  uint64_t sequence=0;
  bool ready=false,repaired_from_raw=false,fused_scan=false,raw_summaries_ready=false;
  bool full_provenance=false,virtual_center=false;
  void allocate(uint32_t w,uint32_t h,const DirectionGridLayout &grid,uint32_t tile=16,uint32_t block=3,bool full_sources=false) {
    if(!w || !h || w>4096 || h>4096 || !grid.cells() || !tile || tile>256 || (block!=1 && block!=3))
      throw std::invalid_argument("invalid frame observation index dimensions");
    width=w;height=h;layout=grid;tile_edge=tile;
    tile_columns=(w+tile-1)/tile;tile_rows=(h+tile-1)/tile;
    block_edge=block;block_columns=(w+block-1)/block;block_rows=(h+block-1)/block;blocks_per_camera=block_columns*block_rows;
    full_provenance=full_sources;
    offsets.resize(full_sources ? grid.cells()+1 : 0);sources.resize(full_sources ? 4*blocks_per_camera : 0);
    if(!full_sources) {offsets.shrink_to_fit();sources.shrink_to_fit();} // initialization only
    cell_counts.resize(grid.cells());camera_coverage.resize(grid.cells());
    for(auto &t:tiles) t.resize(tile_columns*tile_rows);
    for(auto &t:raw_tiles) t.resize(tile_columns*tile_rows);
    for(auto &b:blocks) b.resize(blocks_per_camera);
    ready=false;
  }
};

struct PixelRegion { int u0=0,v0=0,u1=0,v1=0; }; // half-open rectangle, no implicit clipping
enum class DepthImageKind { Filled,Raw };
enum class DepthQueryStatus { Ok,NoUsableDepth,InvalidCamera,OutsideImage,InvalidRegion,EmptyRegion,IndexNotReady };
inline const char *query_status_name(DepthQueryStatus status) noexcept {
  switch(status) {
    case DepthQueryStatus::Ok:return "ok";
    case DepthQueryStatus::NoUsableDepth:return "no_usable_depth";
    case DepthQueryStatus::InvalidCamera:return "invalid_camera";
    case DepthQueryStatus::OutsideImage:return "outside_image";
    case DepthQueryStatus::InvalidRegion:return "invalid_region";
    case DepthQueryStatus::EmptyRegion:return "empty_region";
    default:return "index_not_ready";
  }
}
struct DepthQueryResult {
  DepthQueryStatus status=DepthQueryStatus::IndexNotReady;
  std::optional<float> z;
  int camera=-1;
  PixelRegion region;
  uint32_t valid_pixels=0,total_pixels=0,tiles_used=0,pixels_scanned=0;
  int64_t stamp_ns=0;
  uint64_t sequence=0;
};
struct DirectionObservation {
  uint32_t camera=0;
  double u=0.,v=0.; // geometric center in original pixel coordinates (can be half-pixel)
  float optical_z=0.f;
  int64_t stamp_ns=0;
  uint64_t sequence=0;
  uint32_t u0=0,v0=0,u1=0,v1=0; // original half-open block range
  uint32_t valid_pixels=0,total_pixels=0;
};
} // namespace ego4p0
