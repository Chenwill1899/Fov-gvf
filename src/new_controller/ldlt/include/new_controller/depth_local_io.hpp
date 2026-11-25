#pragma once
#include "new_controller/depth_size_query.hpp"
#include "new_controller/depth_virtual_io.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <type_traits>

namespace ego4p0 {
// Offline export only. No persistent 3-D surface points or navigation outputs.
inline void save_local_field(const std::filesystem::path &out,const std::string &prefix,
                             const DepthFrame &frame,const LocalGeometryTables &geometry) {
  if(geometry.virtual_center()) {save_virtual_panorama(out,prefix,frame,geometry);return;}
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  std::ofstream(out/"base_config.json")<<BaseConfig::json<<'\n';
  const LocalFieldView view(frame,geometry);if(!view.ready()) throw std::runtime_error("local frame not ready for export");
  const auto &d=frame.local;const auto &index=frame.observations;
  auto binary=[&](const std::string &name,const auto &values,size_t count) {
    std::ofstream f(out/(prefix+name),std::ios::binary);
    f.write(reinterpret_cast<const char*>(values.data()),count*sizeof(typename std::decay_t<decltype(values)>::value_type));
    if(!f) throw std::runtime_error("local index export failed");
  };
  // Explicit stopped-run diagnostics: reconstruct pixel provenance on demand.
  // None of these CSR arrays is owned or maintained by a live frame slot.
  std::vector<uint32_t> offsets(index.layout.cells()+1),sources,cells,codes;
  for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
    const uint32_t u0=id%index.block_columns*index.block_edge,v0=id/index.block_columns*index.block_edge;
    for(uint32_t v=v0;v<std::min(v0+index.block_edge,index.height);++v) for(uint32_t u=u0;u<std::min(u0+index.block_edge,index.width);++u) {
      const auto source=view.pixel_obstacle(c,u,v);if(!source) continue;
      cells.push_back(source->direction_cell);codes.push_back(c*index.width*index.height+v*index.width+u);++offsets[source->direction_cell+1];
    }
  }
  for(uint32_t cell=0;cell<index.layout.cells();++cell) offsets[cell+1]+=offsets[cell];
  auto cursor=offsets;sources.resize(codes.size());
  for(size_t i=0;i<codes.size();++i) sources[cursor[cells[i]]++]=codes[i];
  binary("_local_obstacle_offsets.bin",offsets,offsets.size());binary("_local_obstacle_sources.bin",sources,sources.size());
  binary("_local_excluded_camera_mask.bin",d.excluded_camera_mask,d.excluded_camera_mask.size());
  std::ofstream domain(out/(prefix+"_block_domain.csv"));domain<<std::setprecision(17)
    <<"camera,block_id,evidence,min_z,max_z,a0,a1,b0,b1,q_norm,q_deviation\n";
  for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
    const auto &b=index.blocks[c][id];const auto &g=geometry.block(c,id);
    const auto box=block_exclusion(b,g,geometry.camera(c).T_BC,index.layout);
    domain<<c<<','<<id<<','<<int(*view.block_evidence(c,id))<<',';
    if(b.valid_pixels) domain<<b.min_z;
    domain<<',';if(b.valid_pixels) domain<<b.max_z;
    domain<<','<<box.a0<<','<<box.a1<<','<<box.b0<<','<<box.b1<<','<<g.q_norm<<','<<g.q_deviation<<'\n';
  }
  std::ofstream limits(out/(prefix+"_z_limits.bin"),std::ios::binary),q(out/(prefix+"_rotated_pixel_rays.bin"),std::ios::binary);
  for(uint32_t c=0;c<4;++c) for(uint32_t p=0;p<index.width*index.height;++p) {
    const auto &r=geometry.pixel(c,p);limits.write(reinterpret_cast<const char*>(&r.z_limit),sizeof(double));
    q.write(reinterpret_cast<const char*>(r.q.data()),3*sizeof(double));
  }
  std::ofstream rays(out/(prefix+"_local_rays.csv"));rays<<std::setprecision(17);
  rays<<"camera,block_id,u0,v0,u1,v1,raw_valid_pixels,total_pixels,z_begin,z_end,begin_known,complete_raw,center_z_limit,min_z_limit,max_z_limit\n";
  uint32_t available_intervals=0;
  for(uint32_t c=0;c<4;++c) for(uint32_t id=0;id<index.blocks_per_camera;++id) {
    const auto query=*view.block_ray(c,id);const auto &r=query.evidence;const auto &g=geometry.block(c,id);
    if(r.raw_valid_pixels && r.z_end>r.z_begin) ++available_intervals;
    const uint32_t u=id%index.block_columns*index.block_edge,v=id/index.block_columns*index.block_edge;
    rays<<c<<','<<id<<','<<u<<','<<v<<','<<std::min(u+index.block_edge,index.width)<<','<<std::min(v+index.block_edge,index.height)
      <<','<<r.raw_valid_pixels<<','<<r.total_pixels<<','<<r.z_begin<<','<<r.z_end<<','<<r.begin_known<<','<<r.complete_raw
      <<','<<g.z_limit<<','<<g.min_limit<<','<<g.max_limit<<'\n';
  }
  std::ofstream tiles(out/(prefix+"_raw_region_tiles.csv"));tiles<<std::setprecision(9)<<"camera,tile_u,tile_v,valid_pixels,min_z\n";
  for(uint32_t c=0;c<4;++c) for(uint32_t v=0;v<index.tile_rows;++v) for(uint32_t u=0;u<index.tile_columns;++u) {
    const auto &t=index.raw_tiles[c][v*index.tile_columns+u];tiles<<c<<','<<u<<','<<v<<','<<t.valid_pixels<<',';
    if(t.valid_pixels) tiles<<t.min_z;
    tiles<<'\n';
  }
  std::ofstream examples(out/(prefix+"_envelope_queries.csv"));examples<<std::setprecision(17);
  examples<<"x_B,y_B,z_B,status,camera,projection_status,fully_inside,u0,v0,u1,v1,z_near,z_far,raw_valid,total_pixels,raw_min_z,filled_min_z,potential_obstacle,observed_clear\n";
  for(const auto &center:std::array<std::array<double,3>,8>{{{0,0,0},{.05,0,0},{.5,0,0},{1,0,0},{0,1,0},{-1,0,0},{0,-1,0},{2,1,.1}}}) {
    const auto result=view.sphere(center);
    for(uint32_t c=0;c<4;++c) {
      const auto &p=result.cameras[c];const auto &f=p.footprint;
      examples<<center[0]<<','<<center[1]<<','<<center[2]<<','<<static_cast<int>(result.status)<<','<<c<<','<<static_cast<int>(f.status)<<','<<f.fully_inside
        <<','<<f.region.u0<<','<<f.region.v0<<','<<f.region.u1<<','<<f.region.v1<<','<<f.z_near<<','<<f.z_far<<','<<p.raw.valid_pixels<<','<<p.raw.total_pixels<<',';
      if(p.raw.z) examples<<*p.raw.z;
      examples<<',';
      if(p.filled.z) examples<<*p.filled.z;
      examples<<','<<p.potential_obstacle<<','<<p.observed_clear<<'\n';
    }
  }
  std::ofstream meta(out/(prefix+"_local_field.json"));meta<<std::setprecision(17);
  meta<<"{\"version\":\"ego4p0_local_field_v3_block_exclusion\",\"radius_m\":"<<ControllerLimits::radius_m<<",\"local_range_m\":"<<ControllerLimits::local_range_m<<",\"stamp_ns\":"<<frame.t_ns<<",\"sequence\":"<<frame.sequence
    <<",\"obstacle_count\":"<<sources.size()<<",\"on_demand_ray_intervals_exported\":"<<available_intervals<<",\"excluded_cells\":"<<d.excluded_cells
    <<",\"near_clip_z_m\":[";
  for(uint32_t c=0;c<4;++c) {if(c) meta<<',';const double near=geometry.camera(c).near_clip_z_m;if(std::isfinite(near)) meta<<near;else meta<<"null";}
  meta<<"],\"local_merge_ns\":"<<d.merge_ns<<",\"source_encoding\":\"debug obstacle only: camera*pixels+v*width+u; rays queried by camera/block_id\""
    <<",\"free_evidence_scope\":\"camera-image ray bundle, z_begin<=Z<z_end only when begin_known; partial raw needs full-resolution confirmation; never entire angular cell\""
    <<",\"free_endpoint_index_generated\":false,\"source_boundary_generated\":false,\"solver_domain_constructed\":false,\"hot_pixel_source_bytes\":0,\"hot_free_index_bytes\":0,\"debug_pixel_sources_reconstructed_after_run\":true,\"hot_local_storage_bytes\":"<<d.excluded_camera_mask.size()
    <<",\"excluded_blocks\":"<<d.excluded_blocks<<",\"hot_near_pixel_transforms\":0,\"block_exclusion_enabled\":true,\"block_radius_inflation_m\":0.1,\"unexcluded_cell_default\":\"unknown\",\"unknown_near_field\":true,\"initial_basis_supplied\":false,\"current_position_checked\":false,\"straight_distance_image_generated\":false,\"point_cloud_generated\":false,\"environment_map_generated\":false}\n";
  if(!domain || !limits || !q || !rays || !tiles || !examples || !meta) throw std::runtime_error("local evidence metadata export failed");
}
} // namespace ego4p0
