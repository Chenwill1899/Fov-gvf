#pragma once
#include "new_controller/depth_observations.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace ego4p0 {
// Diagnostics only, after receiver/control have stopped; never called in 2ms reads.
inline void save_observation_index(const std::filesystem::path &out,const std::string &prefix,const DepthFrame &frame) {
  const DepthObservationView view(frame);const auto &index=frame.observations;
  if(!view.ready()) throw std::runtime_error("cannot export unready/stale observation index");
  // Full provenance is reconstructed only after the timed run when compact.
  std::vector<uint32_t> offsets,sources;
  if(index.full_provenance) {offsets=index.offsets;sources.assign(index.sources.begin(),index.sources.begin()+index.source_count);}
  else {
    offsets.resize(index.layout.cells()+1);sources.resize(index.source_count);
    for(uint32_t cell=0;cell<index.layout.cells();++cell) offsets[cell+1]=offsets[cell]+index.cell_counts[cell];
    auto cursor=offsets;
    for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
      const uint32_t cell=index.blocks[c][id].direction_cell;
      if(cell<index.layout.cells()) sources[cursor[cell]++]=c*index.blocks_per_camera+id;
    }
  }
  auto binary=[&](const std::string &suffix,const auto *data,size_t size) {
    std::ofstream f(out/(prefix+suffix),std::ios::binary);
    f.write(reinterpret_cast<const char*>(data),static_cast<std::streamsize>(size));
    if(!f) throw std::runtime_error("cannot export direction index");
  };
  binary("_direction_offsets.bin",offsets.data(),offsets.size()*sizeof(uint32_t));
  binary("_direction_sources.bin",sources.data(),sources.size()*sizeof(uint32_t));
  binary("_direction_coverage.bin",index.camera_coverage.data(),index.camera_coverage.size());
  binary("_direction_counts.bin",index.cell_counts.data(),index.cell_counts.size()*sizeof(uint32_t));
  std::ofstream block_cells(out/(prefix+"_block_cells.bin"),std::ios::binary),block_local(out/(prefix+"_block_local_flags.bin"),std::ios::binary);
  for(const auto &camera:index.blocks) for(const auto &block:camera) {
    block_cells.write(reinterpret_cast<const char*>(&block.direction_cell),sizeof(uint32_t));
    const uint8_t local=block.mean_in_local_range;block_local.write(reinterpret_cast<const char*>(&local),1);
  }
  std::ofstream meta(out/(prefix+"_observation_index.json"));meta<<std::setprecision(17);
  auto array=[&](const auto &values) {meta<<'[';bool first=true;for(auto v:values){if(!first)meta<<',';meta<<v;first=false;}meta<<']';};
  meta<<"{\"version\":\""<<(index.virtual_center ? "ego4p0_direction_queries_v3_virtual_blocks" : "ego4p0_direction_queries_v2_blocks")<<"\",\"virtual_center_model\":"<<(index.virtual_center ? "true":"false")<<",\"width\":"<<index.width<<",\"height\":"<<index.height
      <<",\"hot_full_provenance\":"<<(index.full_provenance?"true":"false")<<",\"provenance_exported_after_run\":true"
      <<",\"block_edge\":"<<index.block_edge<<",\"block_columns\":"<<index.block_columns<<",\"block_rows\":"<<index.block_rows
      <<",\"blocks_per_camera\":"<<index.blocks_per_camera<<",\"block_mean_input\":\"full_resolution_filled_finite_positive\""
      <<",\"repaired_in_camera_workers\":"<<(index.repaired_from_raw?"true":"false")<<",\"fused_block_region_scan\":"<<(index.fused_scan?"true":"false")
      <<",\"compression_timing_scope\":\"compression_ns includes region summary when fused_block_region_scan=true; region_camera_ns=0 means included, not zero cost\""
      <<",\"azimuth_resolution_deg\":"<<index.layout.azimuth_deg<<",\"elevation_resolution_deg\":"<<index.layout.elevation_deg
      <<",\"azimuth_bins\":"<<index.layout.azimuth_bins<<",\"elevation_bins\":"<<index.layout.elevation_bins
      <<",\"stamp_ns\":"<<index.stamp_ns<<",\"sequence\":"<<index.sequence<<",\"source_count\":"<<index.source_count
      <<",\"observed_cells\":"<<index.observed_cells<<",\"unobserved_cells\":"<<index.layout.cells()-index.observed_cells
      <<",\"tile_edge\":"<<index.tile_edge<<",\"tile_columns\":"<<index.tile_columns<<",\"tile_rows\":"<<index.tile_rows
      <<",\"camera_work_ns\":"<<index.camera_work_ns<<",\"build_total_ns\":"<<index.build_total_ns
      <<",\"timing_scope\":\"camera_work_wall includes parallel compression/projection/full_resolution_ROI; merge follows once; per_camera times overlap\""
      <<",\"valid_depth_pixels\":";array(index.valid_depth_pixels);
  meta<<",\"geometry_rejected\":";array(index.geometry_rejected);
  meta<<",\"camera_observations\":";array(index.camera_observations);
  meta<<",\"camera_covered_cells\":";array(index.camera_covered_cells);
  meta<<",\"usable_blocks\":";array(index.usable_blocks);
  meta<<",\"exact_angle_observations\":";array(index.exact_angle_observations);
  meta<<",\"fast_angle_observations\":";array(index.fast_angle_observations);
  meta<<",\"compression_ns\":";array(index.compression_ns);
  meta<<",\"projection_ns\":";array(index.projection_ns);
  meta<<",\"region_camera_ns\":";array(index.region_camera_ns);
  meta<<",\"merge_ns\":"<<index.merge_ns;
  meta<<",\"source_encoding\":\"uint32_le: camera_index*blocks_per_camera+block_v*block_columns+block_u\",\"camera_indices\":[\"Front=0\",\"Left=1\",\"Back=2\",\"Right=3\"]"
      <<",\"grid_layout\":\"beta_row*azimuth_bins+alpha_column; alpha[-pi,pi), beta[-pi/2,pi/2], top beta included\""
      <<",\"coverage_scope\":\"finite-positive block-mean center directions; not full-block angular coverage or free space\""
      <<",\"per_pixel_validity_mask_created\":false,\"point_cloud_created\":false,\"environment_map_created\":false}\n";
  std::ofstream blocks(out/(prefix+"_block_observations.csv"));
  blocks<<"camera,block_id,u0,v0,u1,v1,center_u,center_v,valid_pixels,total_pixels,mean_z,stamp_ns,sequence\n"<<std::setprecision(9);
  for(uint32_t camera=0;camera<4;++camera) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
    const uint32_t u0=id%index.block_columns*index.block_edge,v0=id/index.block_columns*index.block_edge;
    const uint32_t u1=std::min(u0+index.block_edge,index.width),v1=std::min(v0+index.block_edge,index.height);
    const auto &b=index.blocks[camera][id];
    blocks<<camera<<','<<id<<','<<u0<<','<<v0<<','<<u1<<','<<v1<<','<<(u0+u1-1)*.5<<','<<(v0+v1-1)*.5<<','<<b.valid_pixels<<','<<(u1-u0)*(v1-v0)<<',';
    if(b.valid_pixels) blocks<<b.mean_z;
    blocks<<','<<index.stamp_ns<<','<<index.sequence<<'\n';
  }
  std::ofstream tiles(out/(prefix+"_region_tiles.csv"));tiles<<"camera,tile_u,tile_v,valid_pixels,min_z\n"<<std::setprecision(9);
  for(uint32_t camera=0;camera<4;++camera) for(uint32_t y=0;y<index.tile_rows;++y) for(uint32_t x=0;x<index.tile_columns;++x) {
    const auto &tile=index.tiles[camera][y*index.tile_columns+x];tiles<<camera<<','<<x<<','<<y<<','<<tile.valid_pixels<<',';
    if(tile.valid_pixels) tiles<<tile.min_z;
    tiles<<'\n';
  }
  std::ofstream queries(out/(prefix+"_query_examples.csv"));
  queries<<"kind,camera,u0,v0,u1,v1,status,z,valid_pixels,total_pixels,stamp_ns,sequence,tiles_used,pixels_scanned\n"<<std::setprecision(9);
  auto row=[&](const char *kind,const DepthQueryResult &q) {
    queries<<kind<<','<<q.camera<<','<<q.region.u0<<','<<q.region.v0<<','<<q.region.u1<<','<<q.region.v1<<','<<query_status_name(q.status)<<',';
    if(q.z) queries<<*q.z;
    queries<<','<<q.valid_pixels<<','<<q.total_pixels<<','<<q.stamp_ns<<','<<q.sequence<<','<<q.tiles_used<<','<<q.pixels_scanned<<'\n';
  };
  for(int camera=0;camera<4;++camera) {
    row("pixel",view.pixel(camera,index.width/2,index.height/2));
    row("region",view.region(camera,{0,0,static_cast<int>(index.width),static_cast<int>(index.height)}));
    row("region",view.region(camera,{1,1,static_cast<int>(index.width)-1,static_cast<int>(index.height)-1}));
    for(uint32_t p=0;p<index.width*index.height;++p) if(!valid_depth(frame.D_filled[camera][p])) {
      const int x=p%index.width,y=p/index.width;row("pixel",view.pixel(camera,x,y));row("region",view.region(camera,{x,y,x+1,y+1}));break;
    }
  }
  row("pixel",view.pixel(0,-1,0));row("region",view.region(0,{-1,0,1,1}));
  row("region",view.region(0,{0,0,0,1}));row("region",view.region(0,{1,0,0,1}));row("pixel",view.pixel(4,0,0));
  if(!meta || !blocks || !tiles || !queries || !block_cells || !block_local) throw std::runtime_error("cannot export observation metadata/queries");
}
} // namespace ego4p0
