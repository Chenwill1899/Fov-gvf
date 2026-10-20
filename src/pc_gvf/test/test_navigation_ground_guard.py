"""Run production collector/collides AST with scene stubs; no Isaac or ROS launch."""
import ast
import math
import os
from pathlib import Path
from types import SimpleNamespace

SOURCE = (Path(os.environ['FOV_GVF_GUARD_SOURCE']) if 'FOV_GVF_GUARD_SOURCE' in os.environ
          else Path(__file__).resolve().parents[3] / 'scripts/isaac/run_fov_gvf_navigation.py')


class Prim:
    def __init__(self, path, minimum, maximum, role='', mesh=True):
        self.path, self.minimum, self.maximum = path, minimum, maximum
        self.role, self.mesh = role, mesh
    def GetPath(self): return self.path
    def IsA(self, kind): return self.mesh
    def GetAttribute(self, name): return SimpleNamespace(Get=lambda: self.role)


def construct(prims, esdf, sample_value=2.):
    tree = ast.parse(SOURCE.read_text())
    start = next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign)
                 and any(isinstance(t,ast.Name) and t.id=='obstacle_bounds' for t in n.targets))
    end = next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign)
               and any(isinstance(t,ast.Name) and t.id=='first_person_camera_path' for t in n.targets))
    queries, samples = [], []
    def boundable(prim):
        queries.append(prim.path)
        box=SimpleNamespace(GetMin=lambda:prim.minimum, GetMax=lambda:prim.maximum)
        return SimpleNamespace(ComputeWorldBound=lambda *args: SimpleNamespace(ComputeAlignedBox=lambda:box))
    def sample(point):
        samples.append(point)
        return sample_value
    env=dict(stage=SimpleNamespace(Traverse=lambda:prims), esdf=esdf, body_radius=.48,
        UsdGeom=SimpleNamespace(Mesh='Mesh', Boundable=boundable, Tokens=SimpleNamespace(default_='default')),
        Usd=SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda:0)),
        Gf=SimpleNamespace(Vec3d=tuple), math=math, sample_esdf=sample)
    body=tree.body[start:end]+[next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='collides')]
    exec(compile(ast.Module(body=body,type_ignores=[]),str(SOURCE),'exec'),env)
    return env,queries,samples


def ground(path='/World/Ground',top=-.2):
    return Prim(path,(-80.,-60.,-1.),(80.,60.,top))


def test_esdf_ground_all_tiles_checked_before_esdf_and_no_cloud_bounds():
    grounds=[ground(),ground('/World/SpeedTile1/Ground',-.3),ground('/World/SpeedTile2/Ground',-.4)]
    cloud=Prim('/World/Cloud_Obstacles',(-80.,-60.,0.),(80.,60.,20.))
    robot=ground('/World/Robot/Ground',50.)
    env,queries,samples=construct([cloud,*grounds,robot],object())
    assert queries==[p.path for p in grounds]
    assert env['ground_top_z']==-.2 and env['obstacle_bounds']==[]
    threshold=env['ground_top_z']+env['body_radius']
    assert env['collides']((0.,0.,threshold-.01))
    assert env['collides']((0.,0.,threshold))
    assert not samples  # Ground checks cannot be bypassed by large ESDF clearance.
    assert not env['collides']((0.,0.,threshold+1e-6))
    assert len(samples)==1
    # Recorded diagnostic point remains above actual USD floor; audit z=0 is separate.
    assert not env['collides']((196.422007388,7.852959616,.403402701))


def test_explicit_ground_role_is_used_in_esdf_mode():
    role=Prim('/World/CustomFloor',(-2.,-2.,-.5),(2.,2.,.1),role='static_ground')
    env,queries,_=construct([role],object())
    assert queries==[role.path] and env['ground_top_z']==.1
    assert env['collides']((0.,0.,.57))


def test_fallback_keeps_ground_and_obstacle_collision_checks():
    cloud=Prim('/World/Box',(1.,0.,1.),(2.,1.,2.))
    env,queries,_=construct([ground(),cloud],None)
    assert queries==['/World/Ground','/World/Box']
    assert len(env['obstacle_bounds'])==1
    assert env['collides']((0.,0.,.27))
    assert env['collides']((2.4,.5,1.5))
    assert not env['collides']((2.6,.5,1.5))


def test_no_ground_preserves_esdf_rejection():
    env,queries,_=construct([],object(),sample_value=.47)
    assert env['ground_top_z'] is None and not queries
    assert env['collides']((0.,0.,1.))
    env,_,_=construct([],object(),sample_value=.49)
    assert not env['collides']((0.,0.,1.))
