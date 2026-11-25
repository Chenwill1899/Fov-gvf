#!/usr/bin/env python3
"""Verify imported geometry, conservative collision coverage and rig identity."""
import hashlib,json,unittest
from pathlib import Path
import numpy as np
from pxr import Gf,Usd,UsdGeom,UsdPhysics
from build_geomonoplan_scenes import Adapter,ASSETS,ROOT,C,C4,NAMES,corners,matrix_points
from reference_scene_geometry import bounds,swept_clearance

class GeoScenes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.adapter=Adapter(ASSETS);cls.maps=[]
        for name in NAMES:
            folder=ROOT/'scenes'/('reference_'+name);m=json.loads((folder/'manifest.json').read_text());cls.maps.append((folder,m,Usd.Stage.Open(str(folder/m['scene']))))
    def test_files_and_unchanged_camera_rig(self):
        template=Usd.Stage.Open(str(ROOT/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd'))
        for folder,m,s in self.maps:
            with self.subTest(map=folder.name):
                self.assertFalse(s.GetCompositionErrors());self.assertEqual(UsdGeom.GetStageUpAxis(s),'Z');self.assertEqual(UsdGeom.GetStageMetersPerUnit(s),1.)
                for f,k in [(m['scene'],'scene_sha256'),('occupancy.bin','occupancy_sha256')]:self.assertEqual(hashlib.sha256((folder/f).read_bytes()).hexdigest(),m[k])
                self.assertEqual((folder/'occupancy.bin').stat().st_size,int(np.prod(m['shape'])))
                for f,h in m['asset_sha256'].items():self.assertEqual(hashlib.sha256((folder/f).read_bytes()).hexdigest(),h)
                metadata=json.loads(s.GetRootLayer().customLayerData['fovNavigationJson']);self.assertEqual(metadata['occupancyShape'],m['shape']);self.assertEqual(metadata['occupancyResolution'],m['resolution']);self.assertEqual(metadata['spawn'],m['spawn'])
                for mount in ['CameraMount','CameraMountLeft','CameraMountBack','CameraMountRight']:
                    path='/World/Robot/'+mount
                    for prim in Usd.PrimRange(template.GetPrimAtPath(path)):
                        actual=s.GetPrimAtPath(prim.GetPath());self.assertTrue(actual)
                        for a in prim.GetAttributes():self.assertEqual(str(a.Get()),str(actual.GetAttribute(a.GetName()).Get()))
                self.assertEqual(m['robot_visual']['name'],'ARL-Robot-1');self.assertEqual(metadata['bodyRadiusM'],.48)
    def test_every_source_triangle_is_covered_by_a_template_proxy(self):
        a=self.adapter;seen=set();count=0
        for scene in a.data['scenes']:
            for entry in a.raw_instances(scene):
                key=(entry['mesh'],entry['tree'])
                if key in seen:continue
                seen.add(key);points=a.points(key[0]);faces=points[np.concatenate(a.triangles(key[0]))];low,high=faces.min(1),faces.max(1);covered=np.zeros(len(faces),bool)
                for lo,hi in a.proxy_boxes(*key):covered|=np.all((low>=lo-1e-6)&(high<=hi+1e-6),axis=1)
                self.assertTrue(covered.all(),key);count+=len(faces)
        self.assertGreater(count,100000)
    def test_original_transforms_and_authored_collision_boxes(self):
        a=self.adapter
        for (folder,m,s),source in zip(self.maps,a.data['scenes']):
            cache=UsdGeom.XformCache();bboxes=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default'],False,True);by_name={o['name']:o for o in m['objects']};world=np.array(m['layout']['world_transform']);proxy_count=0
            entries=a.raw_instances(source)
            self.assertEqual(len(entries),m['layout']['renderer_count'])
            for i,entry in enumerate(entries):
                expected=world@C4@np.array(entry['matrix'])@C4.T;prim=s.GetPrimAtPath('/World/Imported/R_'+str(i));self.assertTrue(prim);actual=np.array(cache.GetLocalToWorldTransform(prim)).T;np.testing.assert_allclose(actual,expected,atol=1e-8)
                if not entry['tree']:
                    obj=by_name[f'Imported_{i}_0'];points=matrix_points(a.points(entry['mesh']),expected);planes=np.array(obj['planes']);self.assertLessEqual(float(np.max(points@planes[:,:3].T+planes[:,3])),1e-4);proxy_count+=1;continue
                for j,(lo,hi) in enumerate(a.proxy_boxes(entry['mesh'],entry['tree'])):
                    vertices=matrix_points(corners(lo,hi),expected);obj=by_name[f'Imported_{i}_{j}'];low,high=bounds(obj);self.assertTrue(np.all(vertices>=low-1e-7)&np.all(vertices<=high+1e-7));proxy_count+=1
            self.assertEqual(proxy_count,m['layout']['collision_proxy_count'])
            for obj in m['objects']:
                prim=s.GetPrimAtPath('/World/Environment/'+obj['name']);self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI));box=bboxes.ComputeWorldBound(prim).ComputeAlignedBox();lo,hi=bounds(obj);np.testing.assert_allclose(box.GetMin(),lo,atol=1e-4);np.testing.assert_allclose(box.GetMax(),hi,atol=1e-4)
    def test_original_tree_counts_and_garage_headroom(self):
        self.assertEqual(self.maps[0][1]['layout']['tree_count'],794);self.assertEqual(self.maps[1][1]['layout']['tree_count'],487);self.assertEqual(self.maps[2][1]['layout']['source_renderer_count'],3042)
        self.assertEqual([m['layout']['vertical_scale'] for _,m,_ in self.maps],[1.,1.,2.])
    def test_grid_occupancy_and_offline_route(self):
        for folder,m,_ in self.maps:
            grid=np.memmap(folder/'occupancy.bin',dtype=np.uint8,mode='r',shape=tuple(m['shape']));box=np.array([bounds(o) for o in m['objects']]);origin=np.array(m['origin'])
            for obj in m['objects']:
                p=np.mean(obj['vertices'],axis=0) if obj['kind']=='convex' else np.array(obj['center']);cell=np.rint((p-origin)/m['resolution']).astype(int)
                if np.all(cell>=0) and np.all(cell<m['shape']):self.assertEqual(grid[tuple(cell)],1,obj['name'])
            path=m['layout']['offline_smoke_connectivity_path'];self.assertEqual(path[0],m['spawn']);self.assertEqual(path[-1],m['goal'])
            for a,b in zip(path,path[1:]):
                low=np.minimum(a,b)-.85;high=np.maximum(a,b)+.85;selected=np.all((box[:,0]<=high)&(box[:,1]>=low),axis=1)
                self.assertGreater(swept_clearance(a,b,[m['objects'][i] for i in np.flatnonzero(selected)],.635),0)

if __name__=='__main__':unittest.main(verbosity=2)
