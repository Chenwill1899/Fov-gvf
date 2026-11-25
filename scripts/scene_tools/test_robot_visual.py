#!/usr/bin/env python3
"""Check embedded ARL geometry and preserved kinematic/sensor contracts."""
import hashlib,json,unittest
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
from robot_visual import ROOT,ASSET,LEGACY


class RobotAppearance(unittest.TestCase):
    def test_prepared_source_and_embedding(self):
        source=json.loads((ASSET/'provenance.json').read_text())
        self.assertEqual(hashlib.sha256((ASSET/'visual.usdc').read_bytes()).hexdigest(),source['visual_sha256'])
        self.assertGreater(source['triangles'],300000)
        for folder in ['ego_swarm_cloud','reference_lab','reference_hangar','reference_forest','reference_geo_forest','reference_geo_clutter','reference_geo_garage']:
            if not (ROOT/'scenes'/folder/'manifest.json').exists():continue
            with self.subTest(map=folder):
                directory=ROOT/'scenes'/folder;m=json.loads((directory/'manifest.json').read_text())
                self.assertEqual(m['robot_visual']['visual_sha256'],source['visual_sha256'])
                s=Usd.Stage.Open(str(directory/m['scene']))
                self.assertFalse(s.GetCompositionErrors())
                self.assertTrue(s.GetPrimAtPath('/World/Robot/Visuals'))
                for name in LEGACY:self.assertFalse(s.GetPrimAtPath('/World/Robot/'+name))
                vertices=[];cache=UsdGeom.XformCache()
                robot=s.GetPrimAtPath('/World/Robot')
                world_to_robot=cache.GetLocalToWorldTransform(robot).GetInverse()
                for p in Usd.PrimRange(robot):
                    self.assertFalse(p.HasAPI(UsdPhysics.CollisionAPI))
                    self.assertFalse(p.HasAPI(UsdPhysics.RigidBodyAPI))
                    self.assertNotIn('Joint',p.GetTypeName())
                    if p.IsA(UsdGeom.Mesh):
                        points=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(),dtype=float)
                        matrix=np.asarray(cache.GetLocalToWorldTransform(p)*world_to_robot)
                        vertices.append(points@matrix[:3,:3]+matrix[3,:3])
                points=np.concatenate(vertices)
                self.assertLessEqual(float(np.linalg.norm(points,axis=1).max()),.460001)
                self.assertLessEqual(float(points[:,2].max()),-.039999)
                for mount in ['CameraMount','CameraMountLeft','CameraMountBack','CameraMountRight']:
                    camera=UsdGeom.Camera(s.GetPrimAtPath('/World/Robot/'+mount+'/DepthCamera'))
                    self.assertEqual(camera.GetFocalLengthAttr().Get(),12.)
                    self.assertEqual(camera.GetHorizontalApertureAttr().Get(),24.)


if __name__=='__main__':unittest.main()
