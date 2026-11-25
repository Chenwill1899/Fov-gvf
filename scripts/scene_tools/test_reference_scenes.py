#!/usr/bin/env python3
"""Geometry, USD, sensor-template and route regression checks (Isaac Python)."""
import hashlib
import json
from pathlib import Path
import unittest
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Sdf,UsdShade
from reference_scene_geometry import bounds,signed_distance,voxelize,swept_clearance
from reference_lab_layout import sample_layout

ROOT=Path(__file__).resolve().parents[2]


class ReferenceScenes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.maps=[]
        for name in ['lab','hangar','forest']:
            folder=ROOT/'scenes'/f'reference_{name}'
            m=json.loads((folder/'manifest.json').read_text())
            cls.maps.append((folder,m,Usd.Stage.Open(str(folder/m['scene']))))

    def test_files_and_sensor_contract(self):
        for folder,m,s in self.maps:
            with self.subTest(scene=m['map_identity']):
                self.assertEqual(UsdGeom.GetStageUpAxis(s),'Z')
                self.assertEqual(UsdGeom.GetStageMetersPerUnit(s),1.)
                self.assertFalse(s.GetRootLayer().subLayerPaths)
                for n,k in [(m['scene'],'scene_sha256'),('occupancy.bin','occupancy_sha256')]:
                    self.assertEqual(hashlib.sha256((folder/n).read_bytes()).hexdigest(),m[k])
                self.assertEqual((folder/'occupancy.bin').stat().st_size,int(np.prod(m['shape'])))
                meta=json.loads(s.GetRootLayer().customLayerData['fovNavigationJson'])
                self.assertEqual(meta['occupancyShape'],m['shape']);self.assertEqual(meta['spawn'],m['spawn'])
                for mount in ['CameraMount','CameraMountLeft','CameraMountBack','CameraMountRight']:
                    camera=UsdGeom.Camera(s.GetPrimAtPath('/World/Robot/'+mount+'/DepthCamera'))
                    self.assertEqual(camera.GetFocalLengthAttr().Get(),12.)
                    self.assertEqual(camera.GetHorizontalApertureAttr().Get(),24.)

    def test_every_solid_has_collision_geometry(self):
        for _,m,s in self.maps:
            cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default'],False,True)
            for obj in m['objects']:
                with self.subTest(object=obj['name']):
                    p=s.GetPrimAtPath('/World/Environment/'+obj['name'])
                    self.assertTrue(p.HasAPI(UsdPhysics.CollisionAPI))
                    self.assertEqual(p.GetAttribute('reference:kind').Get(),obj['kind'])
                    b=cache.ComputeWorldBound(p).ComputeAlignedBox()
                    self.assertFalse(b.IsEmpty())
            self.assertEqual(len([p for p in s.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]),len(m['objects']))

    def test_gate_apertures_and_frames(self):
        for _,m,_ in self.maps:
            for gate in m['gates']:
                with self.subTest(gate=gate['name']):
                    self.assertGreater(min(float(signed_distance(gate['center'],o)) for o in m['objects']),.635)
                    tangent=np.array([0,1,0] if gate['normal']==[1,0,0] else [1,0,0])
                    p=np.array(gate['center'])+tangent*(gate['inner_width']/2+.08)
                    self.assertLess(min(float(signed_distance(p,o)) for o in m['objects']),0)

    def test_all_reference_segments_have_reliable_body_clearance(self):
        for _,m,_ in self.maps:
            # Random lab obstacles may cross the direct goal line. Validate its
            # offline connected route, without assuming online avoidance is solved.
            smoke=m.get('layout',{}).get('offline_smoke_connectivity_path',[m['spawn'],m['goal']])
            self.assertEqual(smoke[0],m['spawn']);self.assertEqual(smoke[-1],m['goal'])
            for points in [smoke,[m['spawn']]+m['waypoints']]:
                for i,(a,b) in enumerate(zip(points,points[1:])):
                    with self.subTest(scene=m['map_identity'],leg=i):
                        self.assertGreater(swept_clearance(a,b,m['objects'],.635),0)

    def test_lab_density_spacing_and_physical_group_separation(self):
        _,m,_=self.maps[0]
        entries=m['layout']['placements']
        self.assertEqual(m['extent'],[140,100,7])
        self.assertEqual(len(entries),250)
        centers=np.array([e['xy'] for e in entries])
        distance=np.linalg.norm(centers[:,None]-centers,axis=2)
        np.fill_diagonal(distance,np.inf)
        self.assertGreaterEqual(float(distance.min()),6.)
        nearest=distance.min(axis=1)
        self.assertLess(float(nearest.std()/nearest.mean()),.2)
        # Count by actual coordinates, not the declared zone labels.
        cells=np.floor((centers+[70,50])/[28,20]).astype(int)
        patterns=[]
        for row in range(5):
            for col in range(5):
                selected=np.flatnonzero(np.all(cells==[col,row],axis=1))
                self.assertEqual(sum(entries[i]['kind']=='stack' for i in selected),6)
                self.assertEqual(sum(entries[i]['kind']=='gate' for i in selected),4)
                local=centers[selected]-[-70+28*col,-50+20*row]
                patterns.append(tuple(sorted(map(tuple,np.round(local,3)))))
        self.assertEqual(len(set(patterns)),25)
        # Bounding rectangles include every piece of each assembled obstacle.
        rectangles=[]
        for e in entries:
            group=[o for o in m['objects'] if o['name'].startswith(e['name']+'_')]
            self.assertEqual(len(group),2 if e['kind']=='stack' else 6)
            low,high=zip(*(bounds(o) for o in group))
            rectangles.append((np.min(low,axis=0)[:2],np.max(high,axis=0)[:2]))
        for i,(low,high) in enumerate(rectangles):
            for other_low,other_high in rectangles[i+1:]:
                gap=np.maximum(np.maximum(low-other_high,other_low-high),0)
                self.assertGreater(np.linalg.norm(gap),2*.635)

    def test_lab_random_seed_is_reproducible_and_changes_layout(self):
        current=self.maps[0][1]['layout']
        repeated=sample_layout(current['seed'])
        self.assertEqual(repeated['placements'],current['placements'])
        self.assertNotEqual(sample_layout(current['seed']+1)['placements'],current['placements'])

    def test_forest_natural_distribution_and_dependencies(self):
        folder,m,s=self.maps[2]
        entries=m['layout']['placements']
        trees=[e for e in entries if e['name'].startswith('Tree_')]
        self.assertEqual(len(trees),288)
        self.assertEqual(sum(e['name'].startswith('Shrub_') for e in entries),144)
        xy=np.array([e['position'][:2] for e in trees])
        d=np.linalg.norm(xy[:,None]-xy,axis=2);np.fill_diagonal(d,np.inf)
        self.assertGreaterEqual(float(d.min()),6.1-1e-7)
        self.assertEqual(len(set(np.round(xy[:,0],4))),len(trees))
        self.assertEqual(len(set(np.round(xy[:,1],4))),len(trees))
        self.assertGreater(np.std([e['yaw'] for e in trees]),70)
        self.assertGreater(np.ptp([e['scale'] for e in trees]),.4)
        self.assertEqual(len({e['asset'] for e in trees}),2)
        self.assertFalse(any('white_base' in o['name'] or 'brace' in o['name'] for o in m['objects']))
        for name,sha in m['asset_sha256'].items():
            self.assertEqual(hashlib.sha256((folder/name).read_bytes()).hexdigest(),sha)
        for e in entries:
            p=s.GetPrimAtPath('/World/Vegetation/'+e['name'])
            self.assertTrue(p.IsInstance())
            self.assertEqual(UsdGeom.Imageable(p).ComputeVisibility(),'inherited')
        for asset in ['tree_small_02','island_tree_01']:
            leaf=UsdShade.Material(s.GetPrimAtPath('/World/TreeLibrary/'+asset+'/_materials/'+asset+'_leaves'))
            source=leaf.GetSurfaceOutput().GetConnectedSource()
            self.assertIsNotNone(source,'Leaf material must export a usable Preview Surface')
            shader=UsdShade.Shader(source[0].GetPrim())
            self.assertTrue(shader.GetInput('diffuseColor').GetConnectedSource())
            self.assertTrue(shader.GetInput('opacity').GetConnectedSource())
        for p in s.TraverseAll():
            for attr in p.GetAttributes():
                if attr.GetTypeName()==Sdf.ValueTypeNames.Asset and attr.Get():
                    if attr.Get().path=='OmniPBR.mdl':continue  # Isaac built-in material library
                    self.assertTrue(Path(attr.Get().resolvedPath).is_file(),str(attr.Get()))

    def test_forest_scale_preserves_original_and_extensions_are_independent(self):
        folder,m,_=self.maps[2]
        self.assertEqual(m['extent'],[360,80,24])
        self.assertEqual(m['spawn'],[-172,0,6.])
        base=json.loads((folder/'assets/base_layout_v2.json').read_text())
        current={e['name']:e for e in m['layout']['placements']}
        for e in base['placements']:
            actual=current[e['name']]
            self.assertEqual(actual['section'],'center')
            self.assertTrue(np.allclose(actual['position'],np.array(e['position'])*2))
            self.assertAlmostEqual(actual['scale'],e['scale']*2)
            self.assertEqual(actual['yaw'],e['yaw']);self.assertEqual(actual['asset'],e['asset'])
        patterns=[]
        for label,offset in [('west',-120),('center',0),('east',120)]:
            entries=[e for e in current.values() if e['section']==label and e['name'].startswith('Tree_')]
            self.assertEqual(len(entries),96)
            self.assertTrue(all(offset-60<=e['position'][0]<=offset+60 for e in entries))
            patterns.append(tuple(sorted((round(e['position'][0]-offset,4),round(e['position'][1],4)) for e in entries)))
        self.assertEqual(len(set(patterns)),3)
        objects={o['name']:o for o in m['objects']}
        for e in base['objects']:
            actual=objects[e['name']]
            self.assertAlmostEqual(actual['radius'],2*e['radius'])
            for key in (['center'] if e['kind']=='sphere' else ['a','b']):
                self.assertTrue(np.allclose(actual[key],np.array(e[key])*2))

    def test_forest_mesh_collision_contains_every_source_triangle(self):
        folder,_,_=self.maps[2]
        for asset in ['tree_small_02','island_tree_01']:
            root=folder/'assets'/asset
            data=np.load(root/'collision_source.npz')
            faces=data['vertices'].astype(float)[data['triangles']]
            keys=np.floor(faces.mean(axis=1)/1.5).astype(int)
            groups,inverse=np.unique(keys,axis=0,return_inverse=True)
            proxy=json.loads((root/'collision.json').read_text())
            self.assertEqual(len(groups),len(proxy['pieces']))
            self.assertEqual(sum(p['triangle_count'] for p in proxy['pieces']),len(faces))
            for i,piece in enumerate(proxy['pieces']):
                points=faces[inverse==i].reshape(-1,3)
                planes=np.asarray(piece['planes'])
                self.assertTrue(np.allclose(np.linalg.norm(planes[:,:3],axis=1),1))
                for plane in planes:
                    self.assertLessEqual(float(np.max(points@plane[:3]+plane[3])),1e-6)

    def test_convex_clearance_and_voxel_supercover(self):
        planes=[[1,0,0,-.1],[-1,0,0,-.1],[0,1,0,-1],
                [0,-1,0,-1],[0,0,1,-1],[0,0,-1,-1]]
        obj=dict(kind='convex',planes=planes,aabb_min=[-.1,-1,-1],aabb_max=[.1,1,1])
        self.assertAlmostEqual(float(signed_distance([0,0,0],obj)),-.1)
        self.assertGreater(swept_clearance([-3,3,0],[3,3,0],[obj],.48),0)
        self.assertLess(swept_clearance([-3,0,0],[3,0,0],[obj],.48),0)
        grid=voxelize([obj],[-.3,-1.2,-1.2],[5,17,17],.15)
        self.assertTrue(np.all(grid[2,2:15,2:15]))

    def test_voxel_supercover_does_not_drop_diagonal_thin_brace(self):
        rod=dict(kind='capsule',a=[.12,.17,.11],b=[1.7,1.4,2.25],radius=.045)
        origin=np.array([-.3,-.3,-.3]);res=.15
        grid=voxelize([rod],origin,[20,20,24],res)
        t=np.linspace(0,1,501);p=np.array(rod['a'])+t[:,None]*(np.array(rod['b'])-rod['a'])
        idx=np.rint((p-origin)/res).astype(int)
        self.assertTrue(np.all(grid[idx[:,0],idx[:,1],idx[:,2]]))
        self.assertLess(swept_clearance([0,.75,1.15],[2,.75,1.15],[rod],.48),0)

    def test_segment_cannot_tunnel_through_box_or_sphere(self):
        box=dict(kind='box',center=[0,0,0],size=[.1,2,2])
        sphere=dict(kind='sphere',center=[0,0,0],radius=.1)
        for obj in [box,sphere]:
            self.assertLess(swept_clearance([-3,0,0],[3,0,0],[obj],.48),0)
            self.assertGreater(swept_clearance([-3,3,0],[3,3,0],[obj],.48),0)


if __name__=='__main__':unittest.main(verbosity=2)
