"""Reproducible, evenly populated random lab layout and offline course authoring.

These helpers are used only when building a scene, never by the flight controller.
"""
import heapq
from collections import deque
import math

import numpy as np

from reference_scene_geometry import bounds, capped_scene_clearance, swept_clearance


def sample_layout(seed=20261022):
    """Ten independently placed groups per 28 x 20 m zone, at least 6 m apart."""
    rng = np.random.default_rng(seed)
    zones = [(row, col) for row in range(5) for col in range(5)]
    points, entries, limits = [], [], []
    # Equal local quotas keep density uniform; zones do not have physical borders.
    kinds = {zone: rng.permutation(['stack'] * 6 + ['gate'] * 4).tolist()
             for zone in zones}
    for round_index in range(10):
        for zone_index in rng.permutation(len(zones)):
            row, col = zones[zone_index]
            lo = np.maximum([-70 + 28 * col, -50 + 20 * row], [-66.5, -46.5])
            hi = np.minimum([-42 + 28 * col, -30 + 20 * row], [66.5, 46.5])
            candidates = rng.uniform(lo, hi, (1600, 2))
            # Full initial free ball and arrival space, without clearing a corridor.
            keep = np.ones(len(candidates), dtype=bool)
            for endpoint in [[-67, -45], [66, -45]]:
                keep &= np.linalg.norm(candidates - endpoint, axis=1) >= 5.0
            candidates = candidates[keep]
            if points:
                nearest = np.linalg.norm(candidates[:, None] - np.array(points), axis=2).min(axis=1)
                best = int(np.argmax(nearest))
            else:
                best = 0
            point = np.round(candidates[best], 6)
            points.append(point)
            limits.append((lo + .0001, hi - .0001))
            entries.append(dict(zone=[row, col], kind=kinds[(row, col)][round_index],
                                xy=point.tolist()))
    # Relax close pairs together; greedy placement alone can trap the last group
    # of a zone. Every step remains inside its zone and the outer wall margin.
    points = np.array(points)
    lo, hi = np.array(limits)[:, 0], np.array(limits)[:, 1]
    for iteration in range(1000):
        delta = points[:, None] - points
        distance = np.linalg.norm(delta, axis=2)
        np.fill_diagonal(distance, np.inf)
        force = np.maximum(6.2 - distance, 0) / distance
        points += .25 * (force[:, :, None] * delta).sum(axis=1)
        for endpoint in [[-67, -45], [66, -45]]:
            away = points - endpoint
            length = np.linalg.norm(away, axis=1)
            points += away * np.maximum(5.01 / length - 1, 0)[:, None]
        points = np.clip(points, lo, hi)
        if distance.min() >= 6.1999:
            break
    points = np.round(points, 6)
    for entry, point in zip(entries, points):
        entry['xy'] = point.tolist()
    heights = rng.permutation([2.2, 3.0, 1.5, 2.6, 1.2, 1.8] * 25).tolist()
    gate_z = rng.permutation([2.2, 2.2, 3.4, 3.6] * 25).tolist()
    # Retain the original 75:25 orientation mix, randomized independently of zones.
    axes = rng.permutation(['x'] * 75 + ['y'] * 25).tolist()
    stack_i = gate_i = 0
    for entry in entries:
        if entry['kind'] == 'stack':
            entry.update(name=f'Stack_{stack_i:03d}', height=heights[stack_i],
                         color='blue' if rng.random() < .5 else 'dark')
            stack_i += 1
        else:
            entry.update(name=f'Gate_{gate_i:03d}', z=gate_z[gate_i], axis=axes[gate_i])
            gate_i += 1
    distances = np.linalg.norm(np.array(points)[:, None] - np.array(points), axis=2)
    np.fill_diagonal(distances, np.inf)
    nearest = distances.min(axis=1)
    if nearest.min() < 6.0:
        raise RuntimeError('Cannot satisfy the lab minimum spacing; scene was not built')
    return dict(method='stratified best-candidate random placement with pair relaxation',
                seed=seed, original_extent=[28, 20, 7], horizontal_scale=[5, 5],
                obstacle_scale=1., zones=[5, 5], zone_extent=[28, 20],
                groups_per_zone=10, stacks_per_zone=6, gates_per_zone=4,
                stack_count=stack_i, gate_count=gate_i, minimum_center_spacing_m=6.,
                relaxation_iterations=iteration + 1,
                nearest_neighbor_m=dict(min=float(nearest.min()), median=float(np.median(nearest)),
                                        max=float(nearest.max()), mean=float(nearest.mean()),
                                        cv=float(nearest.std() / nearest.mean())),
                placements=entries)


def author_routes(objects, spawn, goal, x_limits=(-68., 68.), y_limits=(-48., 48.),
                  anchors=None, grid_step=1., route_z=3., max_shortcut_m=None):
    """Author a connected static course using a conservative configurable grid and altitude.

    The returned smoke path is offline connectivity evidence only. Runtime smoke
    still sends the final goal directly and must avoid obstacles using depth.
    """
    xs = np.arange(x_limits[0], x_limits[1]+grid_step*.5, grid_step)
    ys = np.arange(y_limits[0], y_limits[1]+grid_step*.5, grid_step)
    xx, yy = np.meshgrid(xs, ys, indexing='ij')
    grid_points = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, route_z)])
    required = 1.5 if grid_step == 1. else .8 + math.sqrt(2)*grid_step/2
    clearance = capped_scene_clearance(grid_points, objects, required)
    # Clearance is 1-Lipschitz: subtracting half the diagonal grid edge
    # leaves >=0.79 m, greater than the reliable body radius 0.635 m.
    free = (clearance >= required).reshape(xx.shape)
    valid = np.argwhere(free)
    boxes = np.array([bounds(obj) for obj in objects])

    def nearest(point):
        candidate = np.array([point[0] - xs[0], point[1] - ys[0]]) / grid_step
        return tuple(valid[np.argmin(np.linalg.norm(valid - candidate, axis=1))])

    # Snap exploratory course anchors into the spawn-connected free component.
    # Exact spawn/goal connections still require continuous clearance below.
    first=nearest(spawn)
    connected={first};queue=deque([first])
    while queue:
        here=queue.popleft()
        for dx,dy in [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]:
            nxt=(here[0]+dx,here[1]+dy)
            if (0<=nxt[0]<len(xs) and 0<=nxt[1]<len(ys)
                    and free[nxt] and nxt not in connected):
                connected.add(nxt);queue.append(nxt)
    valid=np.array(sorted(connected))

    def xyz(node):
        return [float(xs[node[0]]), float(ys[node[1]]), float(route_z)]

    def path(start, end):
        first, last = nearest(start), nearest(end)
        frontier = [(0., first)]
        costs, previous = {first: 0.}, {}
        while frontier:
            _, here = heapq.heappop(frontier)
            if here == last:
                break
            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
                nxt = (here[0] + dx, here[1] + dy)
                if not (0 <= nxt[0] < len(xs) and 0 <= nxt[1] < len(ys) and free[nxt]):
                    continue
                cost = costs[here] + math.hypot(dx, dy)
                if cost < costs.get(nxt, math.inf):
                    costs[nxt], previous[nxt] = cost, here
                    heapq.heappush(frontier, (cost + math.dist(nxt, last), nxt))
        else:
            raise RuntimeError(f'Disconnected course endpoints {start}, {end}')
        nodes = [last]
        while nodes[-1] != first:
            nodes.append(previous[nodes[-1]])
        raw = [list(start)] + [xyz(node) for node in reversed(nodes)] + [list(end)]
        # Greedy shortcuts are checked against analytic geometry, not grid alone.
        simplified, index = [raw[0]], 0
        while index < len(raw) - 1:
            for candidate in range(len(raw) - 1, index, -1):
                if max_shortcut_m is not None and np.linalg.norm(np.array(raw[index])-raw[candidate])>max_shortcut_m:
                    continue
                lo=np.minimum(raw[index],raw[candidate])-.85
                hi=np.maximum(raw[index],raw[candidate])+.85
                near=np.all((boxes[:,0]<=hi)&(boxes[:,1]>=lo),axis=1)
                local=[objects[i] for i in np.flatnonzero(near)]
                if swept_clearance(raw[index], raw[candidate], local, .75) > 0:
                    simplified.append(raw[candidate]); index = candidate
                    break
            else:
                raise RuntimeError('No clear course connection to exact endpoint')
        return simplified

    smoke_path = path(spawn, goal)
    if anchors is None:
        anchors = [[60, -40], [60, 0], [-60, 0], [-60, 40], [60, 40]]
    anchors = [xyz(nearest(p)) for p in anchors]
    course = [list(spawn)]
    for anchor in anchors:
        course.extend(path(course[-1], anchor)[1:])
    return smoke_path, course[1:]
