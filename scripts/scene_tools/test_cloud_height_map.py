#!/usr/bin/env python3
"""Check the 40m Cloud against its frozen 20m geometry and color source."""
import hashlib
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
import numpy as np
from robot_visual import LEGACY
from pxr import Sdf, Usd, UsdGeom

ROOT=Path(__file__).resolve().parents[2]
FOLDER=ROOT/'scenes/ego_swarm_cloud'


class HeightStretch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory(prefix='ego_height_test_')
        with tarfile.open(FOLDER/'height20_source.tar.gz') as a:
            source=Path(cls.temp.name)/'source.usd';source.write_bytes(a.extractfile('ego_swarm_cloud_navigation.usd').read())
            cls.old_grid=np.frombuffer(a.extractfile('occupancy.bin').read(),np.uint8).reshape(1200,300,50)
            cls.old_manifest=json.load(a.extractfile('manifest.json'))
            cls.old_routes=json.load(a.extractfile('routes.json'))
        cls.old=Usd.Stage.Open(str(source))
        cls.new=Usd.Stage.Open(str(FOLDER/'ego_swarm_cloud_navigation.usd'))
        cls.manifest=json.loads((FOLDER/'manifest.json').read_text())

    @classmethod
    def tearDownClass(cls):
        cls.old=None;cls.temp.cleanup()

    def test_all_obstacle_vertices_stretch_about_ground(self):
        for path in self.old_manifest['appearance']['targets']:
            before=UsdGeom.Mesh(self.old.GetPrimAtPath(path));after=UsdGeom.Mesh(self.new.GetPrimAtPath(path))
            points=np.asarray(before.GetPointsAttr().Get(),dtype=float)
            self.assertTrue(np.array_equal(points,np.asarray(after.GetPointsAttr().Get())))
            self.assertEqual(before.GetFaceVertexIndicesAttr().Get(),after.GetFaceVertexIndicesAttr().Get())
            matrices=[np.asarray(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())) for mesh in [before,after]]
            old_world=points@matrices[0][:3,:3]+matrices[0][3,:3]
            new_world=points@matrices[1][:3,:3]+matrices[1][3,:3]
            self.assertTrue(np.array_equal(old_world[:,:2],new_world[:,:2]))
            self.assertTrue(np.allclose(new_world[:,2],2*old_world[:,2]+.2,rtol=0,atol=1e-6))
            self.assertAlmostEqual(float(new_world[:,2].min()),-.2,places=5)
            self.assertAlmostEqual(float(new_world[:,2].max()),39.8,places=5)

    def test_indexed_height_colors_are_preserved_exactly(self):
        for path in self.old_manifest['appearance']['targets']:
            a=UsdGeom.Mesh(self.old.GetPrimAtPath(path)).GetDisplayColorPrimvar()
            b=UsdGeom.Mesh(self.new.GetPrimAtPath(path)).GetDisplayColorPrimvar()
            self.assertEqual(a.Get(),b.Get());self.assertEqual(a.GetIndices(),b.GetIndices())
            self.assertEqual(a.GetInterpolation(),b.GetInterpolation())
        self.assertEqual(self.manifest['appearance']['height_range_m'],[-.2,39.8])
        self.assertEqual(self.manifest['appearance']['bottom_to_top'],self.old_manifest['appearance']['bottom_to_top'])

    def test_ground_robot_cameras_and_lighting_unchanged(self):
        for path in ['/World/Robot','/World/Ground','/World/SpeedTile1/Ground','/World/SpeedTile2/Ground','/World/Looks','/World/Sun','/World/Sky']:
            layers=[]
            for stage in [self.old,self.new]:
                layer=Sdf.Layer.CreateAnonymous()
                Sdf.CreatePrimInLayer(layer,Sdf.Path(path).GetParentPath())
                Sdf.CopySpec(stage.GetRootLayer(),path,layer,path)
                if path=='/World/Robot':
                    temporary_stage=Usd.Stage.Open(layer)
                    for name in LEGACY+['Visuals']:temporary_stage.RemovePrim(path+'/'+name)
                layers.append(layer.ExportToString())
            self.assertEqual(*layers,msg=path)

    def test_collision_cells_match_exact_stretched_volume(self):
        new=np.fromfile(FOLDER/'occupancy.bin',np.uint8).reshape(1200,300,100)
        self.assertTrue(np.array_equal(new[:,:,::2],self.old_grid))
        self.assertTrue(np.array_equal(new[:,:,1::2],self.old_grid))
        self.assertTrue(np.array_equal(new.any(axis=2),self.old_grid.any(axis=2)))

    def test_metadata_files_and_route_transform(self):
        m=self.manifest;meta=json.loads(self.new.GetRootLayer().customLayerData['fovNavigationJson'])
        self.assertEqual(meta['occupancyShape'],m['shape']);self.assertEqual(m['shape'],[1200,300,100])
        self.assertEqual(meta['spawn'],self.old_manifest['start'])
        for name,key in [(m['scene'],'scene_sha256'),('occupancy.bin','occupancy_sha256')]:
            self.assertEqual(hashlib.sha256((FOLDER/name).read_bytes()).hexdigest(),m[key])
        routes=json.loads((ROOT/'src/pc_gvf/config/routes.json').read_text())
        self.assertEqual(routes['scene_sha256'],m['scene_sha256'])
        for old,new in zip(self.old_routes['routes'],routes['routes']):
            for key in ['start','goal']:
                self.assertEqual(new[key][:2],old[key][:2]);self.assertEqual(new[key][2],2*old[key][2]+.2)
        self.assertEqual(m['working_bounds']['max_z'],39.8)
        self.assertFalse(self.new.GetRootLayer().subLayerPaths)


if __name__=='__main__':unittest.main(verbosity=2)
