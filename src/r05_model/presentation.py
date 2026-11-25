"""R05 reference-video camera, ARL appearance and shared 1600x900 HUD.

Camera rules recovered from the 2026-11-07 reference renderer. Presentation
never supplies navigation, clearance, collision or actuator data.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WIDTH, HEIGHT, FPS = 1600, 900, 30
ASSET = ROOT / 'scenes/robots/arl_robot_1'
LEGACY = ['Body','ArmForward','ArmSide']+[f'{kind}_{i}' for kind in ['Motor','Propeller'] for i in range(1,5)]


def attach_arl(stage):
    """Session-only appearance replacement; keep source USD and camera rig exact."""
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics
    provenance = json.loads((ASSET/'provenance.json').read_text())
    if hashlib.sha256((ASSET/'visual.usdc').read_bytes()).hexdigest()!=provenance['visual_sha256']:
        raise ValueError('ARL model identity mismatch')
    root_before = stage.GetRootLayer().ExportToString()
    source = Usd.Stage.Open(str(ASSET/'visual.usdc'))
    destination = Sdf.Path('/World/Robot/Visuals')
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.DefinePrim(str(destination),'Xform')
    for name in LEGACY:
        if stage.GetPrimAtPath('/World/Robot/'+name):
            stage.OverridePrim('/World/Robot/'+name).SetActive(False)
    old = source.GetDefaultPrim().GetPath()
    if not Sdf.CopySpec(source.GetRootLayer(),old,stage.GetSessionLayer(),destination):
        raise ValueError('ARL appearance copy failed')
    for prim in Usd.PrimRange(stage.GetPrimAtPath(destination)):
        for rel in prim.GetRelationships():
            if rel.GetTargets():rel.SetTargets([p.ReplacePrefix(old,destination) for p in rel.GetTargets()])
        for attr in prim.GetAttributes():
            if attr.GetConnections():attr.SetConnections([p.ReplacePrefix(old,destination) for p in attr.GetConnections()])
        if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('visual asset unexpectedly imports physics')
    if stage.GetRootLayer().ExportToString()!=root_before:
        raise ValueError('appearance replacement modified the source scene layer')
    provenance.update(session_only=True,physical_parameters_changed=False,
        simulation_body_radius_m=.10,full_arl_physical_envelope_validated=False)
    return provenance


class ReferenceCamera:
    offsets=np.array([(-d,l,h) for d in (4.3,3.5,2.8,2.1,1.6,1.1)
        for l in (-2.4,-1.6,-.8,0.,.8,1.6,2.4) for h in (.8,1.4,2.0,2.6)])
    preferred=np.array([-4.3,1.6,2.6])
    def __init__(self,grid):
        self.grid=grid;self.origin=np.array([-80.,-60.,0.]);self.shape=np.array(grid.shape);self.previous=None
    def occluded(self,eyes,position):
        line=position+(eyes[:,None,:]-position)*np.linspace(.08,1.,28)[None,:,None]
        ijk=np.floor((line-self.origin)/.4+.5).astype(int)
        valid=((ijk>=0)&(ijk<self.shape)).all(axis=2)
        at=np.clip(ijk,0,self.shape-1)
        return np.count_nonzero((self.grid[at[:,:,0],at[:,:,1],at[:,:,2]]!=0)&valid,axis=1)
    def sample(self,position,yaw):
        position=np.asarray(position,dtype=float)
        forward=np.array([math.cos(yaw),math.sin(yaw),0.]);left=np.array([-forward[1],forward[0],0.])
        candidates=position+self.offsets[:,0,None]*forward+self.offsets[:,1,None]*left+self.offsets[:,2,None]*np.array([0.,0.,1.])
        cost=np.sum((self.offsets-self.preferred)**2*np.array([.030,.010,.020]),axis=1)+self.occluded(candidates,position)*1.e5
        if self.previous is not None:cost+=np.sum((candidates-self.previous)**2,axis=1)*6.
        eye=candidates[np.argmin(cost)]
        if self.previous is not None:
            blend=.72*self.previous+.28*eye
            if not self.occluded(blend[None,:],position)[0]:eye=blend
        self.previous=eye
        aim=position+forward*.65+np.array([0.,0.,.15])
        return eye,aim


    def path(self,position,angles):
        """Exact final reference-video DP and symmetric Gaussian smoothing."""
        position=np.asarray(position);angles=np.asarray(angles)
        forwards=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(len(angles))))
        lefts=np.column_stack((-np.sin(angles),np.cos(angles),np.zeros(len(angles))))
        cameras=position[:,None,:]+forwards[:,None,:]*self.offsets[None,:,0,None]+lefts[:,None,:]*self.offsets[None,:,1,None]+self.offsets[None,:,2,None]*np.array([0.,0.,1.])
        occs=np.asarray([self.occluded(c,at) for c,at in zip(cameras,position)])
        preference=np.sum((self.offsets-self.preferred)**2*np.array([.030,.010,.020]),axis=1)
        score=occs[0]*1.e5+preference;parents=np.zeros(occs.shape,dtype=np.int16);ar=np.arange(len(self.offsets))
        for n in range(1,len(position)):
            change=cameras[n,None,:,:]-cameras[n-1,:,None,:]
            jumps=np.einsum("ijk,ijk->ij",change,change)
            transitions=score[:,None]+jumps*6.;parents[n]=transitions.argmin(axis=0)
            score=transitions[parents[n],ar]+preference+occs[n]*1.e5
        choice=int(score.argmin());eyes=np.zeros((len(position),3))
        for n in reversed(range(len(position))):
            eyes[n]=cameras[n,choice];choice=parents[n,choice]
        window=np.exp(-.5*(np.arange(-7,8)/3.)**2);window/=window.sum()
        smooth=np.column_stack([np.convolve(np.pad(axis,7,mode="edge"),window,mode="valid") for axis in eyes.T])
        for n,at in enumerate(position):
            blends=smooth[n]+np.linspace(0.,1.,9)[:,None]*(eyes[n]-smooth[n]);good=np.flatnonzero(self.occluded(blends,at)==0)
            if len(good):eyes[n]=blends[good[0]]
        return eyes,position+forwards*.65+np.array([0.,0.,.15])


def configure_camera(stage,path):
    from pxr import Gf,UsdGeom
    camera=UsdGeom.Camera.Define(stage,path)
    camera.CreateFocalLengthAttr().Set(25.)
    camera.CreateHorizontalApertureAttr().Set(36.)
    camera.CreateVerticalApertureAttr().Set(20.25)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(.035,800.))
    return camera


def set_camera(camera,eye,aim):
    from pxr import Gf,UsdGeom
    UsdGeom.Xformable(camera).MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*aim),Gf.Vec3d(0,0,1)).GetInverse())


def make_hud(trace, result, replay=True):
    from PIL import Image,ImageDraw,ImageFont
    fps = 30
    width, height = 1600, 900
    fps_pad = 60
    white = "#F4F8FC"
    pale = "#B8CAD7"
    muted = "#829CAA"
    teal = "#60E6CB"
    gold = "#FFE3A1"
    dark = (8, 23, 34, 218)
    edge = (171, 211, 230, 56)
    font_root = Path("/usr/share/fonts/opentype/noto")
    latin = Path("/usr/share/fonts/truetype/dejavu")
    def font(size, bold=False, mono=False):
        if mono:
            return ImageFont.truetype(str(latin/"DejaVuSansMono.ttf"),size)
        return ImageFont.truetype(str(font_root/("NotoSansCJK-Bold.ttc" if bold else "NotoSansCJK-Regular.ttc")),size,index=2)
    tiny, sm, md, title, numbers, clock = font(14),font(17),font(21,True),font(34,True),font(28,mono=True),font(18,mono=True)
    loc = np.array([[float(r[k]) for k in ("x","y","z")] for r in trace])
    cumulative = np.r_[0,np.cumsum(np.linalg.norm(np.diff(loc,axis=0),axis=1))]
    xy = loc[:,:2]
    span = np.ptp(xy,axis=0)
    map_scale = min(248/span[0],94/span[1])
    map_center = (xy.max(axis=0) + xy.min(axis=0)) * .5
    coords = np.column_stack((1398+(xy[:,0]-map_center[0])*map_scale, 770-(xy[:,1]-map_center[1])*map_scale))
    stride = 12
    whole = [tuple(row) for row in coords[::stride]] + [tuple(coords[-1])]
    def compose(image, frame, closing=False):
        n, step = int(frame["frame"]), int(frame["trace_step"])
        row = frame.get("row", trace[step])
        t = float(row["t"])
        speed = math.sqrt(sum(float(row[k])**2 for k in ("vx","vy","vz")))
        mode = int(row["mode"])
        phase = {0:"准备起飞",1:"滚动跟踪",2:"路线引导",6:"到达并停稳"}.get(mode,"飞行中")
        base = image.convert("RGBA")
        hud = Image.new("RGBA",base.size)
        draw = ImageDraw.Draw(hud)
        def label(x,y,value,f=sm,fill=white,anchor="la"):
            draw.text((x,y),value,font=f,fill=fill,anchor=anchor)
        def panel(box):
            draw.rounded_rectangle(box, radius=13, fill=dark,outline=edge,width=1)

        panel((40,34,384,143))
        draw.rounded_rectangle((58,53,63,120),radius=2,fill=teal)
        label(77,45,"EGO4P3 / R05",title)
        label(78,99,"ISAAC SIM  ·  "+("引导飞行回放" if replay else "引导飞行仿真"),sm,pale)

        panel((1124,34,1560,131))
        for x,name,value,unit in ((1145,"速度",speed,"m/s"),(1282,"高度",float(row["z"]),"m"),(1416,"仿真时间",t,"s")):
            label(x,48,name,tiny,pale)
            label(x,78,f"{value:05.2f}",numbers)
            label(x+87,91,unit,tiny,muted)
        draw.line((1265,56,1265,110),fill=edge)
        draw.line((1400,56,1400,110),fill=edge)

        panel((1237,692,1560,850))
        label(1256,705,"R05  ·  XY 路径",tiny,pale)
        label(1538,705,"Y ↑",tiny,muted,anchor="ra")
        for y in (748,771,794):
            draw.line((1259,y,1538,y),fill=(165,204,218,36))
        draw.line(whole, fill=(152,188,205,103),width=3,joint="curve")
        done=[tuple(point) for point in coords[:step+1:stride]] + [tuple(coords[step])]
        draw.line(done,fill=teal,width=3,joint="curve")
        for x,y in (coords[0], coords[-1]):
            draw.ellipse((x-3,y-3,x+3,y+3),outline=white,width=1)
        x,y = coords[step]
        draw.ellipse((x-4,y-4,x+4,y+4),fill=white,outline=teal,width=2)
        label(1257,816,f"{frame.get('distance_m', cumulative[step]):06.1f} m",clock,white)
        label(1538,821,f"{result['distance_m']:.1f} m",tiny,pale,anchor="ra")

        if closing:
            panel((40,691,628,850))
            label(61,705,"任务完成  /  ARRIVED",title,teal)
            label(62,765,f"全程 {result['distance_m']:.1f} m · 到达误差 {result['remaining_m']:.3f} m",md,white)
            label(62,804,f"扫掠碰撞 {result['swept_overlap_segments']} 次 · 终点速度 {result['final_speed_mps']:.2f} m/s",sm,pale)
        else:
            panel((40,768,290,850))
            label(61,779,"当前状态",tiny,muted)
            label(60,807,phase,md,teal)
        draw.rounded_rectangle((40,869,1560,874),radius=2,fill=(222,238,244,94))
        if t > 0:
            draw.rounded_rectangle((40,869,max(45,40+1520*min(1.,t/result["simulation_s"] if replay else frame.get("distance_m",0)/max(result["distance_m"],1e-9))),874),radius=2,fill=teal)
        return Image.alpha_composite(base,hud)
    return compose


class LiveOverlay:
    """The same HUD image in a transparent viewport layer; no sensor geometry."""
    def __init__(self,window,route,simulation_limit_s):
        import omni.ui as ui
        self.trace=[dict(x=float(p[0]),y=float(p[1]),z=float(p[2]),t=0,vx=0,vy=0,vz=0,mode=0) for p in route]
        if len(self.trace)<2:self.trace.append(dict(self.trace[0],x=self.trace[0]['x']+.001))
        total=float(np.linalg.norm(np.diff(np.array(route),axis=0),axis=1).sum()) if len(route)>1 else .001
        self.result=dict(distance_m=total,simulation_s=simulation_limit_s,remaining_m=0,swept_overlap_segments=0,final_speed_mps=0)
        self.draw=make_hud(self.trace,self.result,replay=False)
        self.provider=ui.ByteImageProvider();self.frame=window.get_frame('r05_reference_hud')
        with self.frame:
            self.widget=ui.ImageWithProvider(self.provider)
    def update(self,row,waypoint,distance):
        from PIL import Image
        i=min(len(self.trace)-1,max(0,int(waypoint)))
        image=self.draw(Image.new('RGBA',(WIDTH,HEIGHT)),dict(frame=0,trace_step=i,row=row,distance_m=distance))
        # The UI API requires a byte list; this runs only at the presentation rate.
        self.last_image=image
        self.provider.set_bytes_data(list(image.tobytes()),[WIDTH,HEIGHT])
