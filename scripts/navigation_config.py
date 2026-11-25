#!/usr/bin/python3
"""Single parameter contract for the no-motion Step2 input-to-angular-Q chain."""
import argparse,hashlib,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
EXPECTED=dict(reference_turn_acceleration_mps2=80.,tracking_error_target_m=.20,normal_attach_position_error_m=.10,normal_attach_angle_deg=5.,lateral_gain_per_s=2.,heading_gain_per_s=20.,heading_rate_limit_rad_s=10.,heading_acceleration_limit_rad_s2=100.,compensation_input_limit_mps=17.,compensation_slew_limit_mps2=450.,max_observation_age_sim_s=.010,motion_output_enabled=False)
def load():
 c=json.loads((ROOT/'src/new_controller/config/navigation_step2.json').read_text());b=json.loads((ROOT/'src/new_controller/config/base_step1.json').read_text())
 if any(c.get(k)!=v for k,v in EXPECTED.items()):raise ValueError('navigation contract mismatch')
 if b['max_speed_mps']!=12 or b['body_radius_m']!=.10 or b['local_range_m']!=4 or b['motion_output_enabled']:raise ValueError('unsupported base configuration')
 derived=dict(reference_max_curvature_per_m=c['reference_turn_acceleration_mps2']/b['max_speed_mps']**2,reference_min_radius_m=b['max_speed_mps']**2/c['reference_turn_acceleration_mps2'],occupancy_tube_radius_m=b['body_radius_m']+c['tracking_error_target_m'])
 for k,v in derived.items():
  if not math.isclose(c.get(k,float('nan')),v,rel_tol=0,abs_tol=1e-14):raise ValueError('inconsistent derived parameter '+k)
 joined=dict(base=b,navigation=c);canonical=json.dumps(joined,separators=(',',':'),sort_keys=True);sha=hashlib.sha256(canonical.encode()).hexdigest()
 return c,b,sha,json.dumps(joined,separators=(',',':'))
def header(c,sha,metadata):
 constants='\n'.join('  static constexpr double '+k+'='+repr(v)+';' for k,v in c.items() if isinstance(v,(int,float)) and not isinstance(v,bool))
 return '#pragma once\n#include "new_controller/base_config.hpp"\nnamespace ego4p0 {\nstruct NavigationConfig {\n'+constants+'\n  static constexpr bool motion_output_enabled=false;\n  static constexpr uint64_t version=0x'+sha[:16]+'ULL;\n  static constexpr const char *sha256="'+sha+'";\n  static constexpr const char *json=R"nav('+metadata+')nav";\n};\nstatic_assert(!NavigationConfig::motion_output_enabled);\n} // namespace ego4p0\n'
def main():
 p=argparse.ArgumentParser();p.add_argument('--sync',action='store_true');p.add_argument('--check',action='store_true');a=p.parse_args();c,b,sha,meta=load();h=header(c,sha,meta)
 for package in ['src/new_controller/ldlt','src/new_controller']:
  path=ROOT/package/'include/new_controller/navigation_config.hpp'
  if a.sync:path.write_text(h)
  elif a.check and (not path.exists() or path.read_text()!=h):raise SystemExit('stale navigation configuration '+str(path))
 print(json.dumps(dict(base=b,navigation=c,configuration_sha256=sha,checked=a.check),indent=2))
if __name__=='__main__':main()
