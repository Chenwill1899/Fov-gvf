#!/usr/bin/env python3
"""Offline sensor-contract check at the default, stationary Cloud spawn.

Scene occupancy is an independent test oracle only. This tool never publishes
free-space evidence or commands, and is never called by the runtime controller.
Input: a compressed NPZ with the four original 320x240 ROS depth arrays.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ORIGIN = np.array([-80., -60., 0.])
SPAWN = np.array([-64., -11.2, 2.8])

def first_hits(grid, origin, rays, maximum=10.):
    """Exact voxel DDA in axial-depth parameter, including simultaneous crossings."""
    shape = np.array(grid.shape)
    cells = np.tile(np.floor((origin - ORIGIN) / .4 + .5).astype(int), (len(rays), 1))
    signs = np.sign(rays).astype(int)
    with np.errstate(divide='ignore', invalid='ignore'):
        boundary = ORIGIN + .4 * (cells + .5 * signs)
        crossing = np.where(signs != 0, (boundary-origin)/rays, np.inf)
        stride = np.where(signs != 0, .4/np.abs(rays), np.inf)
    distance = np.zeros(len(rays))
    hit = np.full(len(rays), np.inf)
    active = np.ones(len(rays), bool)
    for _ in range(160):
        inside = np.all((cells >= 0) & (cells < shape), axis=1)
        active &= inside & (distance <= maximum)
        ids = np.flatnonzero(active)
        if not len(ids):
            break
        c = cells[ids]
        occupied = grid[c[:, 0], c[:, 1], c[:, 2]] != 0
        hit[ids[occupied]] = distance[ids[occupied]]
        active[ids[occupied]] = False
        ids = np.flatnonzero(active)
        next_t = crossing[ids].min(axis=1)
        advance = crossing[ids] <= next_t[:, None] + 1e-10
        cells[ids] += signs[ids] * advance
        crossing[ids] = np.where(advance, crossing[ids] + stride[ids], crossing[ids])
        distance[ids] = next_t
    if np.any(active & (distance <= maximum)):
        raise RuntimeError('DDA budget exhausted')
    return hit

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    expected = {'ego_swarm_cloud_navigation.usd': '43f085ddc46a4391dcd743488af77cf08850a9e279b42252bd744d39057b35b2',
                'occupancy.bin': '567695099c3b518f37bcdd17e22f4238622ed3e3e980020399ad0890c9069e2a'}
    for name, value in expected.items():
        assert hashlib.sha256((ROOT/'scenes/ego_swarm_cloud'/name).read_bytes()).hexdigest() == value, name
    grid = np.fromfile(ROOT/'scenes/ego_swarm_cloud/occupancy.bin', np.uint8).reshape(400, 300, 50)
    capture = np.load(args.capture)
    assert np.allclose(capture['spawn'], SPAWN) and np.allclose(capture['quaternion_wxyz'], [1, 0, 0, 0])
    results = {}
    for view, name in enumerate(['depth', 'depth_left', 'depth_back', 'depth_right']):
        depth = capture[name]
        assert depth.shape == (240, 320)
        y, x = np.where(np.isposinf(depth))
        rays = np.column_stack((np.ones(len(x)), -(x+.5-160)/160, -(y+.5-120)/160))
        yaw = view*np.pi/2
        rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        origin = SPAWN + rotation @ np.array([.22, 0, .02])
        hits = first_hits(grid, origin, rays @ rotation.T)
        count = int(np.isfinite(hits).sum())
        results[name] = {'positive_infinity_pixels': len(x), 'scene_hits_within_10m': count}
    print(json.dumps({'contract': 'isaac_rendered_z', 'pixel_center': '(u+0.5,v+0.5)', 'views': results}, indent=2))
    if any(r['scene_hits_within_10m'] for r in results.values()):
        raise SystemExit('No-return contract conflicts with scene geometry')

if __name__ == '__main__':
    main()
