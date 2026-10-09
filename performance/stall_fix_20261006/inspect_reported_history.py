import struct,re
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[2]
headers=(root/'src/pc_gvf/include/pc_gvf/depth_angular_core.hpp').read_text()+(root/'src/pc_gvf/include/pc_gvf/paper_guidance.hpp').read_text()
types={name:kind for kind,name in re.findall(r'\b(double|int|bool)\s+(\w+)\s*=',headers)}
codec=(root/'src/pc_gvf/src/paper_replay.cpp').read_text().split('    static void state(')[0].split('    static void config(')[1]
fields=[f for f in re.findall(r'io.number\(c\.([\w.]+)\)',codec) if f != 'local_history_repair']  # Captured V17 has no V18 flag.
class Reader:
 def __init__(self,path):self.f=path.open('rb');assert self.f.read(8)==b'EGOPAPRH'
 def get(self,fmt):return struct.unpack('<'+fmt,self.f.read(struct.calcsize('<'+fmt)))[0]
 def arr(self,n):return np.frombuffer(self.f.read(8*n),dtype='<f8').copy()
 def observation(self):
  w,h=self.get('i'),self.get('i');params=self.arr(7);n=self.get('I');depth=self.arr(n).reshape(h,w)
  origin=self.arr(3);rotation=self.arr(9).reshape(3,3,order='F');stamp=self.get('d');version=self.get('Q');view=self.get('i');revoked=self.get('?')
  supporting=[self.observation() for _ in range(self.get('I'))]
  return dict(w=w,h=h,params=params,depth=depth,origin=origin,rotation=rotation,stamp=stamp,view=view,revoked=revoked)
 def history(self):
  self.cfg={name:self.get({'double':'d','int':'i','bool':'?'}[types[name.split('.')[-1]]]) for name in fields}
  return [self.observation() for _ in range(self.get('I'))]
def sees(o,point):
 p=o['rotation'].T@(point-o['origin']);hf,vf,md,fx,fy,cx,cy=o['params']
 if p[2]<=0 or o['revoked']:return False
 u=int(np.floor(fx*p[0]/p[2]+cx+.5));v=int(np.floor(fy*p[1]/p[2]+cy+.5))
 return 0<=u<o['w'] and 0<=v<o['h'] and o['depth'][v,u]>p[2]
position=np.array([-43.6852,-10.0982,2.33919]);direction=np.array([1.92717,.534791,0.]);direction/=np.linalg.norm(direction)
points=[position+.1*direction+np.array(a) for a in ((.106147,.106147,-.560237),(.0750575,.280118,.502295),(.150115,0,.560237))]
for index in (0,1,2,3,4):
 reader=Reader(root/f'performance/stall_20261006_205620/replays/frame_{index}.bin');history=reader.history()
 print('FRAME',index,'view_count',len(history),'history_capacity',reader.cfg['history_max_observations'])
 for o in history:print('history_view',o['view'],'stamp',o['stamp'],'origin',np.round(o['origin'],3).tolist())
 for pi,p in enumerate(points):
  print('point',pi,[(o['view'],o['stamp'],np.round(o['origin'],3).tolist()) for o in history if sees(o,p)])
