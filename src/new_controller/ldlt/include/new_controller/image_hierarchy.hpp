#pragma once
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace ego4p0 {
// Image aggregation geometry only. It does not change either angular grid.
struct ImageHierarchyGeometry {
  struct Placement {uint32_t coarse=0,pixels=0;};
  uint32_t width=0,height=0,first_edge=4,second_edge=3;
  uint32_t first_columns=0,first_rows=0,columns=0,rows=0;
  std::vector<Placement> first_to_second;
  void configure(uint32_t w,uint32_t h,uint32_t first=4,uint32_t second=3) {
    if(!w||!h||(first!=3&&first!=4)||second!=3)throw std::invalid_argument("image hierarchy requires first edge 3/4 and second edge 3");
    width=w;height=h;first_edge=first;second_edge=second;
    first_columns=(w-1)/first+1;first_rows=(h-1)/first+1;
    columns=(first_columns-1)/second+1;rows=(first_rows-1)/second+1;
    if(uint64_t(first_columns)*first_rows>std::numeric_limits<uint32_t>::max()||uint64_t(columns)*rows>std::numeric_limits<uint32_t>::max())throw std::invalid_argument("image hierarchy too large");
    first_to_second.resize(size_t(first_columns)*first_rows);
    for(uint32_t y=0;y<first_rows;++y)for(uint32_t x=0;x<first_columns;++x) {
      first_to_second[y*first_columns+x]={(y/second)*columns+x/second,
        std::min(w-x*first,first)*std::min(h-y*first,first)};
    }
  }
  uint32_t pixel_span()const{return first_edge*second_edge;}
  bool matches(uint32_t w,uint32_t h,uint32_t first,uint32_t second)const{return width==w&&height==h&&first_edge==first&&second_edge==second;}
};
} // namespace ego4p0
