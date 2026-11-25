"""Analytic geometry shared by the reference map builder and offline audit.

The online controller never imports this module or receives these objects.
Grid samples follow the existing simulator convention: origin + index * resolution.
"""
import numpy as np


def bounds(obj):
    if obj['kind'] == 'convex':
        return np.array(obj['aabb_min']), np.array(obj['aabb_max'])
    if obj['kind'] == 'box':
        c, h = np.array(obj['center']), np.array(obj['size']) / 2
        return c - h, c + h
    if obj['kind'] == 'sphere':
        c = np.array(obj['center'])
        return c - obj['radius'], c + obj['radius']
    a, b = np.array(obj['a']), np.array(obj['b'])
    return np.minimum(a, b) - obj['radius'], np.maximum(a, b) + obj['radius']


def signed_distance(points, obj):
    p = np.asarray(points, dtype=float)
    if obj['kind'] == 'convex':
        # Maximum normalized supporting-plane distance is exact inside a convex
        # polytope and a conservative 1-Lipschitz lower bound outside it. This
        # keeps voxel supercover and continuous-segment certificates conservative.
        planes = np.asarray(obj['planes'])
        shape = p.shape[:-1]
        flat = p.reshape(-1, 3)
        values = np.full(len(flat), -np.inf)
        for plane in planes:
            values = np.maximum(values, flat @ plane[:3] + plane[3])
        return values.reshape(shape)
    if obj['kind'] == 'box':
        q = np.abs(p - obj['center']) - np.array(obj['size']) / 2
        return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(axis=-1), 0)
    if obj['kind'] == 'sphere':
        return np.linalg.norm(p - obj['center'], axis=-1) - obj['radius']
    a, b = np.array(obj['a']), np.array(obj['b'])
    ab = b - a
    t = np.clip(np.sum((p-a)*ab, axis=-1) / np.dot(ab, ab), 0, 1)
    return np.linalg.norm(p - a - t[..., None]*ab, axis=-1) - obj['radius']


def capped_scene_clearance(points, objects, limit):
    """Offline broad-phase clearance for route authoring in detailed forests."""
    points = np.asarray(points)
    result = np.full(len(points), float(limit))
    for obj in objects:
        low, high = bounds(obj)
        near = np.all((points >= low-limit) & (points <= high+limit), axis=1)
        if np.any(near):
            result[near] = np.minimum(result[near], signed_distance(points[near], obj))
    return result


def voxelize(objects, origin, shape, resolution):
    """Conservative supercover: include any cell whose circumsphere hits a solid.

    Unlike center-only sampling, this cannot lose a thin diagonal tree brace.
    Every supported signed-distance function is 1-Lipschitz.
    """
    grid = np.zeros(shape, dtype=np.uint8)
    origin = np.array(origin)
    padding = np.sqrt(3) * resolution / 2
    for obj in objects:
        low, high = bounds(obj)
        lo = np.maximum(0, np.floor((low-padding-origin)/resolution).astype(int))
        hi = np.minimum(shape, np.ceil((high+padding-origin)/resolution).astype(int)+1)
        if np.any(lo >= hi):
            continue
        axes = [origin[i] + np.arange(lo[i], hi[i])*resolution for i in range(3)]
        points = np.stack(np.meshgrid(*axes, indexing='ij'), axis=-1)
        region = tuple(slice(int(a), int(b)) for a, b in zip(lo, hi))
        grid[region] |= (signed_distance(points, obj) <= padding).astype(np.uint8)
    return grid


def swept_clearance(a, b, objects, radius=.48):
    """Conservative continuous segment proof using 1-Lipschitz distance bounds.

    Sample spacing <= 5 cm; subtract half a sample interval and body radius.
    This does not use the simulator's trilinearly interpolated ESDF.
    """
    a, b = np.asarray(a), np.asarray(b)
    length = float(np.linalg.norm(b-a))
    count = max(1, int(np.ceil(length/.05)))
    points = a + np.linspace(0, 1, count+1)[:, None]*(b-a)
    low, high = np.minimum(a,b)-radius-.1, np.maximum(a,b)+radius+.1
    minimum = float('inf')
    for obj in objects:
        ol, oh = bounds(obj)
        # Bound distant solids instead of allocating their full distance arrays.
        separation = np.maximum(np.maximum(ol-high, low-oh), 0)
        if np.any(separation > 0):
            minimum = min(minimum, float(np.linalg.norm(separation)) + .1)
        else:
            minimum = min(minimum, float(signed_distance(points, obj).min())-radius-length/(2*count))
    return minimum
