#pragma once
#include "new_controller/depth_size_query.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <type_traits>
namespace ego4p0 {
inline void save_virtual_panorama(const std::filesystem::path &out,const std::string &prefix,
                                 const DepthFrame &frame,const LocalGeometryTables &geometry) {
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  const LocalFieldView view(frame,geometry);if(!view.ready() || !geometry.virtual_center()) throw std::runtime_error("virtual frame not ready");
  const auto &d=frame.local;const auto &index=frame.observations;const auto &table=geometry.virtual_tables();
  auto binary=[&](const std::string &suffix,const auto &values) {
    std::ofstream file(out/(prefix+suffix),std::ios::binary);file.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(typename std::decay_t<decltype(values)>::value_type));
    if(!file) throw std::runtime_error("virtual export failed");
  };
  binary("_panorama_rho.bin",d.min_range);binary("_panorama_camera_mask.bin",d.observation_cameras);
  binary("_panorama_complete_mask.bin",d.complete_cameras);binary("_panorama_state.bin",d.node_state);
  binary("_panorama_missing_prefix.bin",d.missing_prefix);binary("_local_excluded_camera_mask.bin",d.excluded_camera_mask);
  std::ofstream coefficients(out/(prefix+"_virtual_coefficients.bin"),std::ios::binary),pixel_cells(out/(prefix+"_virtual_pixel_cells.bin"),std::ios::binary),node_sources(out/(prefix+"_virtual_node_blocks.bin"),std::ios::binary),block_cells(out/(prefix+"_virtual_block_cells.bin"),std::ios::binary);
  for(uint32_t c=0;c<4;++c) {
    const auto &values=table.coefficients(c);coefficients.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(double));
    for(uint32_t p=0;p<index.width*index.height;++p) {const auto cell=table.pixel_cell(c,p);pixel_cells.write(reinterpret_cast<const char*>(&cell),4);}
    for(uint32_t id=0;id<index.blocks_per_camera;++id) {const auto cell=table.block_cell(c,id);block_cells.write(reinterpret_cast<const char*>(&cell),4);}
  }
  for(uint32_t cell=0;cell<index.layout.cells();++cell) {const auto &node=table.node(cell);node_sources.write(reinterpret_cast<const char*>(node.blocks.data()),4*sizeof(uint32_t));}
  std::ofstream blocks(out/(prefix+"_virtual_blocks.csv"));blocks<<std::setprecision(17)<<"camera,block_id,min_rho,raw_min_rho,valid_pixels,raw_valid_pixels,total_pixels\n";
  for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
    const auto &b=index.blocks[c][id];const uint32_t u=id%index.block_columns*index.block_edge,v=id/index.block_columns*index.block_edge;
    blocks<<c<<','<<id<<',';if(valid_range(b.min_rho)) blocks<<b.min_rho;blocks<<',';if(valid_range(b.raw_min_rho)) blocks<<b.raw_min_rho;
    blocks<<','<<b.valid_pixels<<','<<b.raw_valid_pixels<<','<<(std::min(u+index.block_edge,index.width)-u)*(std::min(v+index.block_edge,index.height)-v)<<'\n';
  }
  std::ofstream windows(out/(prefix+"_virtual_templates.csv"));windows<<"obstacle,row,bin,a0,a1,b0,b1\n";
  for(uint32_t obstacle=0;obstacle<2;++obstacle) for(uint32_t b=0;b<index.layout.elevation_bins;++b) for(uint32_t k=0;k<table.distance_bins;++k) {
    const auto box=table.window(b*index.layout.azimuth_bins,k,obstacle!=0);windows<<obstacle<<','<<b<<','<<k<<','<<box.a0<<','<<box.a1<<','<<box.b0<<','<<box.b1<<'\n';
  }
  std::ofstream tiles(out/(prefix+"_panorama_tiles.csv"));tiles<<std::setprecision(17)<<"tile,valid,min_rho\n";
  for(uint32_t i=0;i<d.range_tiles.size();++i) {const auto &tile=d.range_tiles[i];tiles<<i<<','<<tile.valid<<',';if(tile.valid) tiles<<tile.minimum;tiles<<'\n';}
  std::ofstream queries(out/(prefix+"_virtual_queries.csv"));queries<<std::setprecision(17)<<"x,y,z,state,cell,a0,a1,b0,b1,min_rho,missing,total\n";
  for(const auto &center:std::array<std::array<double,3>,12>{{{.05,0,0},{.2,0,0},{.5,0,0},{1,0,0},{0,1,0},{-1,0,0},{0,-1,0},{2,1,.1},{0,0,1},{ControllerLimits::local_range_m-.1,0,0},{ControllerLimits::local_range_m,0,0},{-.8,-.01,0}}}) {
    const double range=std::hypot(center[0],center[1],center[2]);const auto cell=*index.layout.cell(std::atan2(center[1],center[0]),std::atan2(center[2],std::hypot(center[0],center[1])));
    const auto box=table.window(cell,table.distance_bin(range));const auto query=view.panorama_region(box);
    queries<<center[0]<<','<<center[1]<<','<<center[2]<<','<<int(view.candidate(center))<<','<<cell<<','<<box.a0<<','<<box.a1<<','<<box.b0<<','<<box.b1<<',';
    if(query.minimum) queries<<*query.minimum;
    queries<<','<<query.missing<<','<<query.total<<'\n';
  }
  std::ofstream meta(out/(prefix+"_local_field.json"));meta<<std::setprecision(17)
    <<"{\"version\":\"ego4p0_local_field_v4_virtual_center\",\"planning_model\":\"body_origin_virtual_omnidirectional\",\"translation_ignored\":true,\"physical_near_clip_checked\":false,\"rotations_retained\":true,\"radial_distance_unit\":\"m\",\"radius_m\":"<<ControllerLimits::radius_m<<",\"local_range_m\":"<<ControllerLimits::local_range_m<<",\"distance_bin_m\":0.05,\"distance_bins\":"<<table.distance_bins<<",\"stamp_ns\":"<<frame.t_ns<<",\"sequence\":"<<frame.sequence
    <<",\"maximum_block_half_angle_rad\":"<<table.maximum_block_half_angle()<<",\"excluded_blocks\":"<<d.excluded_blocks<<",\"excluded_cells\":"<<d.excluded_cells<<",\"available_cells\":"<<d.available_cells<<",\"unknown_cells\":"<<d.unknown_cells
    <<",\"hot_dynamic_angle_binning\":false,\"hot_physical_projection_queries\":false,\"hot_pixel_coordinate_transforms\":0,\"physical_extrinsics_retained_in\":\"simulation.sensors.json\",\"omega_node_states\":{\"0\":\"unknown\",\"1\":\"unavailable\",\"2\":\"available_in_virtual_model\"},\"minimum_range_covers\":\"block min at sampled grid-center direction, not exact continuous panorama\",\"raw_complete_required_for_available\":true,\"solver_implemented\":false,\"motion_output\":false}\n";
  if(!coefficients || !pixel_cells || !node_sources || !block_cells || !blocks || !windows || !tiles || !queries || !meta) throw std::runtime_error("virtual diagnostic export failed");
}
} // namespace ego4p0
