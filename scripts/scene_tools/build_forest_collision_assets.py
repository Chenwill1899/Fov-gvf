#!/usr/bin/env python3
"""Conservative spatial convex proxies for every triangle of prepared tree meshes."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection
from itertools import product

ROOT = Path(__file__).resolve().parents[2]


def build(folder):
    data = np.load(folder / 'collision_source.npz')
    vertices, triangles = data['vertices'].astype(float), data['triangles']
    faces = vertices[triangles]
    # Each triangle is assigned once, but ALL its vertices enter its group's hull.
    # Thus faces crossing a bin boundary are still fully contained by a proxy.
    keys = np.floor(faces.mean(axis=1) / [1.5, 1.5, 1.5]).astype(int)
    groups, inverse = np.unique(keys, axis=0, return_inverse=True)
    pieces = []
    largest_violation = 0.
    for i, key in enumerate(groups):
        points = np.unique(faces[inverse == i].reshape(-1, 3), axis=0)
        # 26-direction convex envelope bounds plane count for fast offline audits.
        # Supports are taken over ALL vertices, expanded 2 mm for flat leaf cards.
        normals = np.array([v for v in product([-1.,0.,1.], repeat=3) if any(v)])
        normals /= np.linalg.norm(normals, axis=1)[:,None]
        offsets = -(points @ normals.T).max(axis=0) - .002
        planes = np.column_stack([normals, offsets])
        shell = HalfspaceIntersection(planes, points.mean(axis=0)).intersections
        hull = ConvexHull(shell)
        # Verify all input triangle vertices; no sparse point sampling.
        violation = max(float(np.max(points @ p[:3] + p[3])) for p in planes)
        largest_violation = max(largest_violation, violation)
        if violation > 1e-6:
            raise ValueError('Proxy failed full triangle containment')
        pieces.append(dict(vertices=shell.tolist(), faces=hull.simplices.tolist(),
            planes=planes.tolist(), aabb_min=shell.min(axis=0).tolist(),
            aabb_max=shell.max(axis=0).tolist(), triangle_count=int(np.sum(inverse == i))))
    result = dict(method='spatial convex envelopes of every rendered triangle; 2 mm shell',
        triangles=len(triangles), max_vertex_plane_violation_m=largest_violation,
        pieces=pieces)
    (folder / 'collision.json').write_text(json.dumps(result, separators=(',', ':'))+'\n')
    print(folder.name, len(triangles), 'triangles', len(pieces), 'convex pieces', flush=True)


if __name__ == '__main__':
    for name in ['tree_small_02', 'island_tree_01']:
        build(ROOT / 'scenes/reference_forest/assets' / name)
