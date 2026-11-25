#!/usr/bin/env python3
"""Independent post-run NumPy checks for stages 5/6; no active algorithm code."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from verify_depth_preprocessing import verify_repair, verify_rays
from verify_direction_queries import verify_index


def verify_block_exclusion(directory,prefix,cal,raw,filled,index,meta,limits,points,sources):
    width,height=index['width'],index['height'];edge=index['block_edge'];columns=index['block_columns'];rows=index['block_rows']
    na,nb=index['azimuth_bins'],index['elevation_bins'];count=rows*columns;n=width*height
    records=list(csv.DictReader((directory/f'{prefix}_block_domain.csv').open()));assert len(records)==4*count
    camera_mask=np.fromfile(directory/f'{prefix}_local_excluded_camera_mask.bin',dtype=np.uint8).reshape(nb,na)
    expected_mask=np.zeros((nb,na),dtype=np.uint8);near_total=0;complete_total=0;unknown_total=0
    uu,vv=np.meshgrid(np.arange(columns)*edge,np.arange(rows)*edge);u0=uu.ravel();v0=vv.ravel();u1=np.minimum(u0+edge,width);v1=np.minimum(v0+edge,height)
    total=(u1-u0)*(v1-v0)
    def blocks(image,fill):
        padded=np.full((rows*edge,columns*edge),fill,dtype=float);padded[:height,:width]=image
        return padded.reshape(rows,edge,columns,edge).transpose(0,2,1,3).reshape(count,edge*edge)
    for c,camera in enumerate(cal['cameras']):
        rec=records[c*count:(c+1)*count];K=np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]])
        R=np.asarray(camera['robot_from_optical_rotation']);t=np.asarray(camera['camera_position_robot_m']);A=R@np.linalg.inv(K)
        centers=np.stack(((u0+u1-1)*.5,(v0+v1-1)*.5,np.ones(count)),axis=-1)@A.T
        norm=np.linalg.norm(centers,axis=-1);deviation=np.zeros(count)
        for u in [u0-.5,u1-.5]:
            for v in [v0-.5,v1-.5]:
                q=np.stack((u,v,np.ones(count)),axis=-1)@A.T
                deviation=np.maximum(deviation,np.linalg.norm(q-centers,axis=-1))
        assert np.allclose(norm,[float(r['q_norm']) for r in rec],rtol=1e-12,atol=1e-12)
        assert np.allclose(deviation,[float(r['q_deviation']) for r in rec],rtol=1e-12,atol=1e-12)
        image=blocks(filled[c],np.nan);valid=np.isfinite(image)&(image>0)
        minimum=np.min(np.where(valid,image,np.inf),axis=-1);maximum=np.max(np.where(valid,image,0),axis=-1)
        raw_image=blocks(raw[c],np.nan);raw_valid=np.isfinite(raw_image)&(raw_image>0);raw_count=raw_valid.sum(axis=-1)
        raw_min=np.min(np.where(raw_valid,raw_image,np.inf),axis=-1);limit=np.max(blocks(limits[c],-np.inf),axis=-1)
        near=valid.any(axis=-1)&(minimum<=limit);complete=(~near)&(raw_count==total)&(raw_min>limit)
        kinds=np.where(near,2,np.where(complete,1,0));assert np.array_equal(kinds,[int(r['evidence']) for r in rec])
        near_total+=int(near.sum());complete_total+=int(complete.sum());unknown_total+=int((kinds==0).sum())
        for bid,r in enumerate(rec):
            if valid[bid].any():assert np.float32(r['min_z'])==minimum[bid] and np.float32(r['max_z'])==maximum[bid]
            else:assert r['min_z']==r['max_z']==''
        # Independent vectorized ball-to-angle bounds and rectangle union.
        select=np.flatnonzero(near);lo=minimum[select];hi=np.minimum(maximum[select],limit[select]);q=centers[select]
        center=((lo+hi)*.5)[:,None]*q+t;horizontal=np.hypot(center[:,0],center[:,1]);distance=np.linalg.norm(center,axis=-1)
        radius=(hi-lo)*.5*norm[select]+hi*deviation[select]+.1+1e-12*(1+distance+hi*norm[select])
        full=distance<=radius;delta=np.arcsin(np.minimum(1,radius/np.maximum(distance,1e-300)));beta=np.arctan2(center[:,2],horizontal)
        b0=np.clip(np.floor((np.maximum(-np.pi/2,beta-delta-1e-10)+np.pi/2)/np.deg2rad(index['elevation_resolution_deg'])).astype(int),0,nb-1)
        b1=np.clip(np.floor((np.minimum(np.pi/2,beta+delta+1e-10)+np.pi/2)/np.deg2rad(index['elevation_resolution_deg'])).astype(int)+1,0,nb)
        beta_full=full|(horizontal<=radius)|(np.abs(beta)+delta+1e-10>=np.pi/2)
        alpha=np.arctan2(center[:,1],center[:,0]);half=np.arcsin(np.minimum(1,radius/np.maximum(horizontal,1e-300)))+1e-10
        lower=(alpha-half+np.pi)%(2*np.pi)-np.pi;upper=(alpha+half+np.pi)%(2*np.pi)-np.pi
        a0=np.clip(np.floor((lower+np.pi)/np.deg2rad(index['azimuth_resolution_deg'])).astype(int),0,na-1)
        a1=np.clip(np.floor((upper+np.pi)/np.deg2rad(index['azimuth_resolution_deg'])).astype(int)+1,0,na)
        az_full=beta_full|((lower>upper)&(a0<=a1));a0[az_full]=0;a1[az_full]=na;b0[full]=0;b1[full]=nb
        saved=np.array([[int(rec[i][k]) for k in ['a0','a1','b0','b1']] for i in select],dtype=int).reshape(-1,4)
        assert np.array_equal(saved,np.stack((a0,a1,b0,b1),axis=-1)), 'conservative block range differs'
        difference=np.zeros((nb+1,na+1),dtype=np.int64)
        def add(x0,x1,y0,y1):
            np.add.at(difference,(y0,x0),1);np.add.at(difference,(y0,x1),-1);np.add.at(difference,(y1,x0),-1);np.add.at(difference,(y1,x1),1)
        wraps=a0>a1;add(a0[~wraps],a1[~wraps],b0[~wraps],b1[~wraps]);add(a0[wraps],np.full(wraps.sum(),na),b0[wraps],b1[wraps]);add(np.zeros(wraps.sum(),dtype=int),a1[wraps],b0[wraps],b1[wraps])
        covered=difference.cumsum(0).cumsum(1)[:nb,:na]>0;expected_mask[covered]|=np.uint8(1<<c)
    assert np.array_equal(camera_mask,expected_mask)
    assert meta['excluded_cells']==int((camera_mask!=0).sum()) and meta['excluded_blocks']==near_total
    assert meta['hot_near_pixel_transforms']==0 and meta['block_exclusion_enabled'] and meta['unexcluded_cell_default']=='unknown'
    # Real close pixels and six body-radius offsets must remain inside exclusion.
    cam=(sources//n).astype(int);checked=0
    for offset in [np.zeros(3)]+[sign*.1*np.eye(3)[axis] for axis in range(3) for sign in [-1,1]]:
        p=points+offset;alpha=np.arctan2(p[:,1],p[:,0]);alpha=np.where(alpha==np.pi,-np.pi,alpha);beta=np.arctan2(p[:,2],np.hypot(p[:,0],p[:,1]))
        a=np.minimum(((alpha+np.pi)/np.deg2rad(index['azimuth_resolution_deg'])).astype(int),na-1);b=np.minimum(((beta+np.pi/2)/np.deg2rad(index['elevation_resolution_deg'])).astype(int),nb-1)
        assert np.all((camera_mask[b,a]&(1<<cam))!=0), 'near pixel/body radius escaped block exclusion';checked+=len(points)
    return dict(block_exclusion=True,excluded_cells=meta['excluded_cells'],excluded_blocks=near_total,complete_far_blocks=complete_total,unknown_blocks=unknown_total,inflated_pixel_samples_verified=checked,remaining_directions_default='unknown',hot_near_pixel_transforms=0)


def verify_local(directory, prefix, cal, filled, width, height):
    directory = Path(directory)
    index = json.loads((directory/f'{prefix}_observation_index.json').read_text())
    meta = json.loads((directory/f'{prefix}_local_field.json').read_text())
    limit = meta['local_range_m']
    block_mode = meta['version']=='ego4p0_local_field_v3_block_exclusion'
    if meta['version']=='ego4p0_local_field_v4_virtual_center':
        from verify_virtual_panorama import verify_virtual
        return verify_virtual(directory,prefix,cal,filled,width,height)
    compact = block_mode or meta['version']=='ego4p0_local_field_v2_compact'
    raw = np.fromfile(directory/'raw_depth.bin' if (directory/'raw_depth.bin').exists()
                      else directory/f'{prefix}_depth.bin', dtype='<f4').reshape(4, height, width)
    b = np.fromfile(directory/'pixel_rays_b.bin', dtype='<f8').reshape(4, height, width, 3)
    q = np.fromfile(directory/f'{prefix}_rotated_pixel_rays.bin', dtype='<f8').reshape(b.shape)
    limits = np.fromfile(directory/f'{prefix}_z_limits.bin', dtype='<f8').reshape(raw.shape)
    n, blocks = width*height, index['blocks_per_camera']
    cells, na, nb = index['azimuth_bins']*index['elevation_bins'], index['azimuth_bins'], index['elevation_bins']
    offsets = np.fromfile(directory/f'{prefix}_local_obstacle_offsets.bin', dtype='<u4').astype(np.int64)
    sources = np.fromfile(directory/f'{prefix}_local_obstacle_sources.bin', dtype='<u4').astype(np.int64)
    collections = [(offsets, sources, 4*n)]
    if not compact:
        free_offsets = np.fromfile(directory/f'{prefix}_local_free_offsets.bin', dtype='<u4').astype(np.int64)
        free_sources = np.fromfile(directory/f'{prefix}_local_free_sources.bin', dtype='<u4').astype(np.int64)
        uncertain = np.fromfile(directory/f'{prefix}_local_uncertain.bin', dtype=np.uint8)
        boundary = np.fromfile(directory/f'{prefix}_local_boundary.bin', dtype=np.uint8)
        collections.append((free_offsets, free_sources, 4*blocks))
    for off, src, capacity in collections:
        assert off.shape == (cells+1,) and off[0] == 0 and np.all(np.diff(off) >= 0) and off[-1] == len(src)
        assert np.unique(src).size == len(src) and np.all((src >= 0) & (src < capacity))
    expected_obstacles = []
    for c, camera in enumerate(cal['cameras']):
        rotation = np.asarray(camera['robot_from_optical_rotation'])
        t = np.asarray(camera['camera_position_robot_m'])
        expected_q = b[c]@rotation.T
        assert np.allclose(q[c], expected_q, rtol=1e-12, atol=1e-12)
        pb_limit = limits[c, ..., None]*expected_q+t
        assert np.allclose(np.linalg.norm(pb_limit, axis=-1), limit, rtol=0, atol=2e-12)
        # Independent direct Cartesian-radius filter, rather than Z<=saved limit.
        valid = np.isfinite(filled[c]) & (filled[c] > 0)
        with np.errstate(invalid='ignore'):
            pb = filled[c, ..., None].astype(float)*expected_q+t
        inside = valid & (np.linalg.norm(pb, axis=-1) <= limit)
        expected_obstacles.extend((c*n+np.flatnonzero(inside)).tolist())
    assert np.array_equal(np.sort(sources), expected_obstacles), 'local edge pixel missing or fabricated'
    expected_uncertain = np.zeros(cells, dtype=np.uint8)

    def verify_angles(actual_cells, points):
        alpha = np.arctan2(points[:, 1], points[:, 0])
        beta = np.arctan2(points[:, 2], np.hypot(points[:, 0], points[:, 1]))
        a, br = actual_cells % na, actual_cells//na
        alo = -np.pi+a*np.deg2rad(index['azimuth_resolution_deg'])
        ahi = np.minimum(alo+np.deg2rad(index['azimuth_resolution_deg']), np.pi)
        blo = -np.pi/2+br*np.deg2rad(index['elevation_resolution_deg'])
        bhi = np.minimum(blo+np.deg2rad(index['elevation_resolution_deg']), np.pi/2)
        delta = np.arctan2(np.sin(alpha-(alo+ahi)/2), np.cos(alpha-(alo+ahi)/2))
        assert np.all(np.abs(delta) <= (ahi-alo)/2+1e-11) and np.all((beta >= blo-1e-11) & (beta <= bhi+1e-11))

    obstacle_cells = np.repeat(np.arange(cells), np.diff(offsets))
    points = np.empty((len(sources), 3))
    for c, camera in enumerate(cal['cameras']):
        selected = sources//n == c
        ids = sources[selected] % n
        points[selected] = filled[c].reshape(-1)[ids, None]*q[c].reshape(-1, 3)[ids]+camera['camera_position_robot_m']
        original = raw[c].reshape(-1)[ids]
        np.bitwise_or.at(expected_uncertain, obstacle_cells[selected][~(np.isfinite(original) & (original > 0))], 1)
    verify_angles(obstacle_cells, points)
    free_rows = list(csv.DictReader((directory/f'{prefix}_local_rays.csv').open()))
    assert len(free_rows) == 4*blocks
    expected_free, free_points = [], {}
    for row in free_rows:
        c, bid = int(row['camera']), int(row['block_id'])
        u0, v0, u1, v1 = [int(row[k]) for k in ('u0', 'v0', 'u1', 'v1')]
        image = raw[c, v0:v1, u0:u1]
        values = image[np.isfinite(image) & (image > 0)]
        near = meta['near_clip_z_m'][c]
        begin_known = near is not None and np.isfinite(near) and near > 0
        begin = near if begin_known else 0.
        min_limit, max_limit = limits[c, v0:v1, u0:u1].min(), limits[c, v0:v1, u0:u1].max()
        assert int(row['raw_valid_pixels']) == values.size and int(row['total_pixels']) == image.size
        assert bool(int(row['begin_known'])) == begin_known and bool(int(row['complete_raw'])) == (values.size == image.size)
        assert np.isclose(float(row['min_z_limit']), min_limit, rtol=0, atol=1e-12)
        assert np.isclose(float(row['max_z_limit']), max_limit, rtol=0, atol=1e-12)
        end = min(float(values.min()), min_limit) if values.size else 0.
        assert float(row['z_begin']) == begin and np.isclose(float(row['z_end']), end, rtol=0, atol=1e-12)
        camera = cal['cameras'][c]
        k = np.array([[camera['fx'], 0, camera['cx']], [0, camera['fy'], camera['cy']], [0, 0, 1.]])
        center_ray = np.linalg.inv(k)@np.array([(u0+u1-1)/2, (v0+v1-1)/2, 1.])
        rotated = np.asarray(camera['robot_from_optical_rotation'])@center_ray
        t = np.asarray(camera['camera_position_robot_m'])
        center_limit = float(row['center_z_limit'])
        assert np.isclose(np.linalg.norm(center_limit*rotated+t), limit, rtol=0, atol=2e-12)
        if values.size and end > begin:
            code = c*blocks+bid
            expected_free.append(code)
            free_points[code] = end*rotated+t
    if compact:
        assert meta['on_demand_ray_intervals_exported']==len(expected_free)
        if block_mode:
            block_validation = verify_block_exclusion(directory,prefix,cal,raw,filled,index,meta,limits,points,sources)
        else:
            raw_counts = np.fromfile(directory/f'{prefix}_local_cell_raw_counts.bin',dtype='<u4')
            repaired_counts = np.fromfile(directory/f'{prefix}_local_cell_repaired_counts.bin',dtype='<u4')
            camera_mask = np.fromfile(directory/f'{prefix}_local_cell_camera_mask.bin',dtype=np.uint8)
            raw_valid = np.isfinite(raw.reshape(-1)[sources]) & (raw.reshape(-1)[sources]>0)
            expected_raw = np.bincount(obstacle_cells[raw_valid],minlength=cells)
            expected_repaired = np.bincount(obstacle_cells[~raw_valid],minlength=cells)
            expected_mask = np.zeros(cells,dtype=np.uint8)
            np.bitwise_or.at(expected_mask,obstacle_cells,(1<<(sources//n)).astype(np.uint8))
            assert np.array_equal(raw_counts,expected_raw) and np.array_equal(repaired_counts,expected_repaired) and np.array_equal(camera_mask,expected_mask)
            assert meta['constrained_cells']==int(np.count_nonzero(raw_counts+repaired_counts))
        assert meta['free_endpoint_index_generated'] is False and meta['source_boundary_generated'] is False
        assert meta['solver_domain_constructed'] is False and meta['hot_pixel_source_bytes']==meta['hot_free_index_bytes']==0
        assert meta['debug_pixel_sources_reconstructed_after_run'] is True
        for suffix in ['local_free_offsets.bin','local_free_sources.bin','local_boundary.bin','local_uncertain.bin']:
            assert not (directory/f'{prefix}_{suffix}').exists(), 'removed free/boundary index recreated'
        block_cells = np.fromfile(directory/f'{prefix}_block_cells.bin',dtype='<u4')
        local_flags = np.fromfile(directory/f'{prefix}_block_local_flags.bin',dtype=np.uint8)
        direction_counts = np.fromfile(directory/f'{prefix}_direction_counts.bin',dtype='<u4')
        direction_offsets = np.fromfile(directory/f'{prefix}_direction_offsets.bin',dtype='<u4')
        direction_sources = np.fromfile(directory/f'{prefix}_direction_sources.bin',dtype='<u4')
        direction_ids = np.repeat(np.arange(cells),np.diff(direction_offsets.astype(np.int64)))
        assert np.array_equal(block_cells[direction_sources],direction_ids) and np.array_equal(direction_counts,np.diff(direction_offsets))
        assert not index['hot_full_provenance']
        block_rows = list(csv.DictReader((directory/f'{prefix}_block_observations.csv').open()))
        for code,row in enumerate(block_rows):
            if not int(row['valid_pixels']):
                assert block_cells[code]==np.iinfo(np.uint32).max and not local_flags[code]
                continue
            c=int(row['camera']);camera=cal['cameras'][c]
            k=np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]])
            center_ray=np.linalg.inv(k)@np.array([float(row['center_u']),float(row['center_v']),1.])
            point=float(row['mean_z'])*(np.asarray(camera['robot_from_optical_rotation'])@center_ray)+camera['camera_position_robot_m']
            assert bool(local_flags[code])==bool(np.linalg.norm(point)<=limit)
    else:
        assert np.array_equal(np.sort(free_sources), expected_free), 'free ray bundle missing or fabricated'
        free_cells = np.repeat(np.arange(cells), np.diff(free_offsets))
        verify_angles(free_cells, np.array([free_points[code] for code in free_sources]).reshape(-1, 3))
        for cell, code in zip(free_cells, free_sources):
            row = free_rows[code]
            if not int(row['begin_known']) or not int(row['complete_raw']) or int(row['u0']) == 0 or int(row['v0']) == 0 or int(row['u1']) == width or int(row['v1']) == height:
                expected_uncertain[cell] = 1
        assert np.array_equal(uncertain, expected_uncertain)
        covered = ((np.diff(offsets)+np.diff(free_offsets)) > 0).reshape(nb, na)
        expected_boundary = expected_uncertain.reshape(nb, na).astype(bool)
        neighbors_unknown = ~np.roll(covered, 1, axis=1) | ~np.roll(covered, -1, axis=1)
        neighbors_unknown[0] = True; neighbors_unknown[-1] = True
        neighbors_unknown[1:] |= ~covered[:-1]; neighbors_unknown[:-1] |= ~covered[1:]
        expected_boundary |= covered & neighbors_unknown
        assert np.array_equal(boundary.reshape(nb, na), expected_boundary)
        assert meta['no_evidence_cells'] == int((~covered).sum()) and meta['boundary_cells'] == int(boundary.sum())
    tiles = list(csv.DictReader((directory/f'{prefix}_raw_region_tiles.csv').open()))
    for row in tiles:
        c, u, v = [int(row[k]) for k in ('camera', 'tile_u', 'tile_v')];edge = index['tile_edge']
        image = raw[c, v*edge:min((v+1)*edge, height), u*edge:min((u+1)*edge, width)]
        values = image[np.isfinite(image) & (image > 0)]
        assert int(row['valid_pixels']) == values.size
        assert np.float32(row['min_z']) == values.min() if values.size else row['min_z'] == ''
    examples = list(csv.DictReader((directory/f'{prefix}_envelope_queries.csv').open()))
    statuses = {}
    for row in examples:
        center = tuple(float(row[k]) for k in ('x_B','y_B','z_B'));c = int(row['camera'])
        status = int(row['status']);statuses.setdefault(center, []).append(row)
        if np.linalg.norm(center) <= 1e-9:
            assert status == 4 and int(row['projection_status']) == 2 and int(row['total_pixels']) == 0
            continue
        camera = cal['cameras'][c];rotation = np.asarray(camera['robot_from_optical_rotation'])
        pc = rotation.T@(np.asarray(center)-camera['camera_position_robot_m'])
        if pc[2] <= .1:
            assert int(row['projection_status']) == 1 and not int(row['observed_clear'])
            continue
        assert int(row['projection_status']) == 0
        # Alternative tangent-angle calculation for the archived zero-skew K.
        def bound(x, focal, principal):
            theta = np.arctan2(x, pc[2]);half = np.arcsin(.1/np.hypot(x, pc[2]))
            return focal*np.tan(theta-half)+principal, focal*np.tan(theta+half)+principal
        u0, u1 = bound(pc[0], camera['fx'], camera['cx']);v0, v1 = bound(pc[1], camera['fy'], camera['cy'])
        bounds = [int(np.floor(u0)), int(np.floor(v0)), int(np.ceil(u1))+1, int(np.ceil(v1))+1]
        clipped = [max(0,min(width,bounds[0])), max(0,min(height,bounds[1])), max(0,min(width,bounds[2])), max(0,min(height,bounds[3]))]
        assert [int(row[k]) for k in ('u0','v0','u1','v1')] == clipped
        assert bool(int(row['fully_inside'])) == (bounds[0]>=0 and bounds[1]>=0 and bounds[2]<=width and bounds[3]<=height)
        assert np.isclose(float(row['z_near']), pc[2]-.1, rtol=0, atol=1e-12) and np.isclose(float(row['z_far']), pc[2]+.1, rtol=0, atol=1e-12)
        if clipped[0] == clipped[2] or clipped[1] == clipped[3]:
            assert int(row['raw_valid']) == 0 and not int(row['observed_clear']);continue
        x0, y0, x1, y1 = clipped
        raw_roi = raw[c, y0:y1, x0:x1];raw_values = raw_roi[np.isfinite(raw_roi) & (raw_roi>0)]
        filled_roi = filled[c, y0:y1, x0:x1];filled_values = filled_roi[np.isfinite(filled_roi) & (filled_roi>0)]
        assert int(row['raw_valid']) == raw_values.size and int(row['total_pixels']) == raw_roi.size
        assert np.float32(row['raw_min_z']) == raw_values.min() if raw_values.size else row['raw_min_z'] == ''
        assert np.float32(row['filled_min_z']) == filled_values.min() if filled_values.size else row['filled_min_z'] == ''
        potential = bool(filled_values.size and filled_values.min() <= pc[2]+.1)
        near = meta['near_clip_z_m'][c]
        clear = bool(int(row['fully_inside']) and near is not None and near>0 and pc[2]-.1>=near
                     and raw_values.size==raw_roi.size and raw_values.min()>pc[2]+.1)
        assert bool(int(row['potential_obstacle'])) == potential and bool(int(row['observed_clear'])) == clear
    for center, rows in statuses.items():
        expected_status = 4 if np.linalg.norm(center)<=1e-9 else 3 if any(int(r['potential_obstacle']) for r in rows) else 1 if any(int(r['observed_clear']) for r in rows) else 0
        assert all(int(r['status']) == expected_status for r in rows)
    assert meta['radius_m'] == .1 and meta['local_range_m'] > 0 and meta['sequence'] == index['sequence'] and meta['stamp_ns'] == index['stamp_ns']
    assert not meta['current_position_checked'] and not meta['initial_basis_supplied'] and not meta['straight_distance_image_generated'] and not meta['point_cloud_generated']
    result = dict(passed=True,optical_limit_rays_verified=4*n,local_obstacle_pixels=len(sources),
                  on_demand_ray_intervals=len(expected_free),raw_tiles_verified=len(tiles),envelope_queries_verified=len(statuses),
                  all_constraints_and_intervals_verified=True,near_clip_readback=meta['near_clip_z_m'],compact=compact)
    if compact:
        result.update(source_boundary_generated=False,free_endpoint_index_generated=False,
                      hot_local_storage_bytes=meta['hot_local_storage_bytes'],full_pixel_provenance_reconstructed_only_for_debug=True)
        if block_mode:
            result.update(block_validation)
        else:
            result['constrained_cells']=meta['constrained_cells']
    else:
        result.update(ray_bundles=len(free_sources),no_evidence_cells=meta['no_evidence_cells'],boundary_cells=meta['boundary_cells'],all_sources_and_boundaries_verified=True)
    return result



def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('directory', type=Path);args = parser.parse_args()
    out = args.directory.resolve();cal = json.loads((out/'simulation.sensors.json').read_text());w, h = cal['render_width'], cal['render_height']
    repair, filled = verify_repair(out/'raw_depth.bin', out/'filled_depth.bin', w, h)
    rays = verify_rays(out, cal, w, h); index = verify_index(out, 'sample', cal, filled, w, h)
    result = dict(passed=True, scope=('synthetic numeric depth with archived camera calibration, no simulation/FIFO' if cal.get('version')=='synthetic_geometry_fixture_v1' else 'offline archived images, no new simulation/FIFO timing claim'), repair=repair, rays=rays,
                  observations=index, local=verify_local(out, 'sample', cal, filled, w, h))
    (out/'local_validation.json').write_text(json.dumps(result, indent=2)+'\n');print(json.dumps(result['local'], indent=2))


if __name__ == '__main__':
    main()
