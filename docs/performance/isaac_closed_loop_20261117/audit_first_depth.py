"""Independent pinhole rays vs scene boxes, sampled from original float depth."""
import json,math,struct,sys
from pathlib import Path
folder=Path(sys.argv[1]);s=json.loads((folder/'scenario.json').read_text());cal=json.loads((folder/'simulation.sensors.json').read_text())
w,h=s['image_width'],s['image_height'];raw=(folder/'first_depth.bin').read_bytes();assert len(raw)==4*w*h*4
p=s['start'];records=[]
def raybox(d,box):
 near,far=-math.inf,math.inf
 for i in range(3):
  if abs(d[i])<1e-12:
   if not box['min'][i]<=p[i]<=box['max'][i]:return math.inf
  else:
   x=(box['min'][i]-p[i])/d[i];y=(box['max'][i]-p[i])/d[i]
   near=max(near,min(x,y));far=min(far,max(x,y))
 if far<max(near,0):return math.inf
 return near if near>0 else far
for c,camera in enumerate(cal['cameras']):
 R=camera['robot_from_optical_rotation']
 for y in (30,75,120,165,210):
  for x in (40,100,160,220,280):
   q=[(x-camera['cx'])/camera['fx'],(y-camera['cy'])/camera['fy'],1.]
   d=[sum(R[i][j]*q[j] for j in range(3)) for i in range(3)]
   expected=min((s['bounds_max'][i]-p[i])/d[i] if d[i]>0 else (s['bounds_min'][i]-p[i])/d[i] for i in range(3) if abs(d[i])>1e-12)
   for box in s['boxes']:expected=min(expected,raybox(d,box))
   actual=struct.unpack_from('<f',raw,4*((c*h+y)*w+x))[0]
   records.append(dict(camera=c,x=x,y=y,expected_optical_z_m=expected,actual_optical_z_m=actual,error_m=abs(expected-actual)))
maximum=max(r['error_m'] for r in records);result=dict(passed=all(math.isfinite(r['actual_optical_z_m']) and r['error_m']<.05 for r in records),samples=len(records),maximum_error_m=maximum,method='independent analytic pinhole rays intersect finite room/axis-aligned boxes at initial pose',records=records)
(folder/'independent_first_depth_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='records'}));raise SystemExit(0 if result['passed'] else 2)
