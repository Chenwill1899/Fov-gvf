#define main archived_auto_cruise_entry
#include "before/controller.cpp"
#undef main

bool archived_surface(const ego4p0::NavigationObservation &obs,const ego4p0::LocalGeometryTables &geometry,
                      const ego4p0::Direction3 &world,double radius){
  return measured_surface_intersects(obs,geometry,world,radius);
}
