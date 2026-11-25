#!/usr/bin/env python3
"""Offline independent verification; never used for runtime index construction."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from verify_depth_preprocessing import verify_repair,verify_rays


def verify_blocks(directory,prefix,meta,cal,filled,width,height):
    edge=meta['block_edge'];columns,rows=meta['block_columns'],meta['block_rows'];units=columns*rows
    assert edge in (1,3) and columns==(width+edge-1)//edge and rows==(height+edge-1)//edge
    assert meta['blocks_per_camera']==units and meta['block_mean_input']=='full_resolution_filled_finite_positive'
    # Accumulate the same original row/column order in float64, independently.
    sums=np.zeros((4,rows,columns),dtype=np.float64);counts=np.zeros((4,rows,columns),dtype=np.uint32)
    for dy in range(edge):
        for dx in range(edge):
            values=filled[:,dy::edge,dx::edge];valid=np.isfinite(values)&(values>0);hh,ww=values.shape[1:]
            sums[:,:hh,:ww]+=np.where(valid,values.astype(np.float64),0.)
            counts[:,:hh,:ww]+=valid
    means=np.full(sums.shape,np.nan,dtype=np.float32)
    np.divide(sums,counts,out=means,where=counts>0,casting='unsafe')
    with (directory/f'{prefix}_block_observations.csv').open() as stream:blocks=list(csv.DictReader(stream))
    assert len(blocks)==4*units
    seen=set()
    for row in blocks:
        c,idx=int(row['camera']),int(row['block_id']);assert (c,idx) not in seen;seen.add((c,idx));assert 0<=c<4 and 0<=idx<units
        by,bx=divmod(idx,columns);u0,v0=bx*edge,by*edge;u1,v1=min(u0+edge,width),min(v0+edge,height)
        assert tuple(int(row[k]) for k in ('u0','v0','u1','v1'))==(u0,v0,u1,v1)
        assert float(row['center_u'])==(u0+u1-1)*.5 and float(row['center_v'])==(v0+v1-1)*.5
        assert int(row['valid_pixels'])==counts[c,by,bx] and int(row['total_pixels'])==(u1-u0)*(v1-v0)
        assert int(row['stamp_ns'])==meta['stamp_ns'] and int(row['sequence'])==meta['sequence']
        if counts[c,by,bx]:assert np.float32(row['mean_z'])==means[c,by,bx]
        else:assert row['mean_z']==''
    u0=np.arange(columns)*edge;v0=np.arange(rows)*edge
    u,v=np.meshgrid((u0+np.minimum(u0+edge,width)-1)*.5,(v0+np.minimum(v0+edge,height)-1)*.5)
    expected_sources=[];expected_alpha=np.full(4*units,np.nan);expected_beta=expected_alpha.copy();rejected=[]
    for camera,c in enumerate(cal['cameras']):
        inverse=np.linalg.inv(np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1]],dtype=float))
        b=np.stack((u,v,np.ones_like(u)),axis=-1).reshape(-1,3)@inverse.T
        q=b@np.asarray(c['robot_from_optical_rotation'],dtype=float).T
        ids=np.flatnonzero(counts[camera].ravel()>0);z=means[camera].ravel()[ids].astype(np.float64)
        pb=q[ids] if meta.get('virtual_center_model') else z[:,None]*q[ids]+np.asarray(c['camera_position_robot_m'],dtype=float)
        keep=np.isfinite(pb).all(axis=1)&np.any(pb!=0,axis=1);rejected.append(int((~keep).sum()));pb=pb[keep];ids=ids[keep]
        codes=camera*units+ids;alpha=np.arctan2(pb[:,1],pb[:,0]);alpha[(pb[:,0]==0)&(pb[:,1]==0)]=0;alpha[alpha==np.pi]=-np.pi
        expected_sources.extend(codes.tolist());expected_alpha[codes]=alpha;expected_beta[codes]=np.arctan2(pb[:,2],np.hypot(pb[:,0],pb[:,1]))
    assert meta['usable_blocks']==[int(np.count_nonzero(c)) for c in counts]
    assert all(meta['exact_angle_observations'][c]+meta['fast_angle_observations'][c]==meta['camera_observations'][c] for c in range(4))
    return units,expected_sources,expected_alpha,expected_beta,rejected,[int(np.sum(c)) for c in counts]


def verify_index(directory,prefix,cal,filled,width,height,expected_stamp=None,expected_sequence=None):
    meta=json.loads((directory/f'{prefix}_observation_index.json').read_text())
    assert (meta['width'],meta['height'])==(width,height)
    if expected_stamp is not None:assert meta['stamp_ns']==expected_stamp
    if expected_sequence is not None:assert meta['sequence']==expected_sequence
    na,nb=meta['azimuth_bins'],meta['elevation_bins'];cells=na*nb;pixels=width*height
    offsets=np.fromfile(directory/f'{prefix}_direction_offsets.bin',dtype='<u4').astype(np.int64)
    sources=np.fromfile(directory/f'{prefix}_direction_sources.bin',dtype='<u4').astype(np.int64)
    coverage=np.fromfile(directory/f'{prefix}_direction_coverage.bin',dtype=np.uint8)
    assert offsets.shape==(cells+1,) and coverage.shape==(cells,)
    assert offsets[0]==0 and offsets[-1]==len(sources)==meta['source_count'] and np.all(np.diff(offsets)>=0)
    block_mode=meta.get('version') in ('ego4p0_direction_queries_v2_blocks','ego4p0_direction_queries_v3_virtual_blocks')
    units=meta['blocks_per_camera'] if block_mode else pixels
    assert len(np.unique(sources))==len(sources) and np.all((sources>=0)&(sources<4*units))
    if block_mode:
        units,expected_sources,expected_alpha,expected_beta,geometry_rejected,valid_counts=verify_blocks(directory,prefix,meta,cal,filled,width,height)
    else:
        b=np.fromfile(directory/'pixel_rays_b.bin',dtype='<f8').reshape(4,height,width,3)
        expected_sources=[];expected_alpha=np.full(4*pixels,np.nan);expected_beta=expected_alpha.copy();geometry_rejected=[]
        valid_counts=[]
        for camera,c in enumerate(cal['cameras']):
            z=filled[camera].reshape(-1);valid=np.isfinite(z)&(z>0);ids=np.flatnonzero(valid)
            pc=b[camera].reshape(-1,3)[ids]*z[ids,None].astype(np.float64)
            r=np.array(c['robot_from_optical_rotation'],dtype=np.float64);t=np.array(c['camera_position_robot_m'],dtype=np.float64)
            pb=pc@r.T+t
            keep=np.isfinite(pb).all(axis=1)&np.any(pb!=0,axis=1)
            geometry_rejected.append(int((~keep).sum()));valid_counts.append(len(ids))
            ids=ids[keep];pb=pb[keep];codes=camera*pixels+ids
            alpha=np.arctan2(pb[:,1],pb[:,0]);alpha[(pb[:,0]==0)&(pb[:,1]==0)]=0
            alpha[alpha==np.pi]=-np.pi
            beta=np.arctan2(pb[:,2],np.hypot(pb[:,0],pb[:,1]))
            expected_sources.extend(codes.tolist());expected_alpha[codes]=alpha;expected_beta[codes]=beta
    assert np.array_equal(np.sort(sources),np.sort(expected_sources)), 'source missing, duplicated, invalid, or fabricated'
    assert meta['valid_depth_pixels']==valid_counts and meta['geometry_rejected']==geometry_rejected
    actual_cells=np.repeat(np.arange(cells),np.diff(offsets))
    alpha_step=np.deg2rad(meta['azimuth_resolution_deg']);beta_step=np.deg2rad(meta['elevation_resolution_deg'])
    a=actual_cells%na;brow=actual_cells//na
    alo=-np.pi+a*alpha_step;ahi=np.minimum(alo+alpha_step,np.pi)
    blo=-np.pi/2+brow*beta_step;bhi=np.minimum(blo+beta_step,np.pi/2)
    alpha=expected_alpha[sources];beta=expected_beta[sources]
    # Matrix evaluation order can differ at exact cell boundaries. Check the
    # actual assigned cell against angles, with a 1e-11 rad numerical tolerance.
    center=(alo+ahi)/2;delta=np.arctan2(np.sin(alpha-center),np.cos(alpha-center))
    assert np.all(np.abs(delta)<=(ahi-alo)/2+1e-11), 'incorrect body azimuth bin'
    assert np.all((beta>=blo-1e-11)&(beta<=bhi+1e-11)), 'incorrect body elevation bin'
    predicted_a=np.minimum(((alpha+np.pi)/alpha_step).astype(np.int64),na-1)
    predicted_b=np.minimum(((beta+np.pi/2)/beta_step).astype(np.int64),nb-1)
    boundary_assignments=int(np.sum(predicted_b*na+predicted_a!=actual_cells))
    expected_coverage=np.zeros(cells,dtype=np.uint8)
    np.bitwise_or.at(expected_coverage,actual_cells,(1<<(sources//units)).astype(np.uint8))
    assert np.array_equal(coverage,expected_coverage)
    assert np.array_equal(np.diff(offsets)>0,coverage!=0)
    assert meta['observed_cells']==int(np.count_nonzero(coverage)) and meta['unobserved_cells']==int(np.count_nonzero(coverage==0))
    for camera in range(4):
        assert meta['camera_observations'][camera]==int(np.sum(sources//units==camera))
        assert meta['camera_covered_cells'][camera]==int(np.count_nonzero(coverage&(1<<camera)))
    with (directory/f'{prefix}_region_tiles.csv').open() as stream:tiles=list(csv.DictReader(stream))
    assert len(tiles)==4*meta['tile_columns']*meta['tile_rows']
    seen=set()
    for row in tiles:
        camera,x,y=map(int,(row['camera'],row['tile_u'],row['tile_v']));edge=meta['tile_edge']
        assert (camera,x,y) not in seen;seen.add((camera,x,y))
        image=filled[camera,y*edge:min((y+1)*edge,height),x*edge:min((x+1)*edge,width)]
        available=image[np.isfinite(image)&(image>0)]
        assert int(row['valid_pixels'])==available.size
        if available.size:assert np.float32(row['min_z'])==available.min()
        else:assert row['min_z']==''
    with (directory/f'{prefix}_query_examples.csv').open() as stream:queries=list(csv.DictReader(stream))
    for row in queries:
        camera=int(row['camera']);u0,v0,u1,v1=[int(row[k]) for k in ('u0','v0','u1','v1')]
        expected=None;valid=total=0
        if camera<0 or camera>=4:status='invalid_camera'
        elif row['kind']=='pixel':
            if u0<0 or v0<0 or u0>=width or v0>=height:status='outside_image'
            else:
                total=1;z=filled[camera,v0,u0]
                if np.isfinite(z) and z>0:expected=z;valid=1;status='ok'
                else:status='no_usable_depth'
        elif u0>u1 or v0>v1:status='invalid_region'
        elif u0<0 or v0<0 or u1>width or v1>height:status='outside_image'
        elif u0==u1 or v0==v1:status='empty_region'
        else:
            region=filled[camera,v0:v1,u0:u1];available=region[np.isfinite(region)&(region>0)]
            valid=available.size;total=region.size;expected=available.min() if valid else None
            status='ok' if valid else 'no_usable_depth'
        assert row['status']==status and int(row['valid_pixels'])==valid and int(row['total_pixels'])==total
        assert int(row['stamp_ns'])==meta['stamp_ns'] and int(row['sequence'])==meta['sequence']
        if expected is None:assert row['z']=='', 'no-result query fabricated a finite depth'
        else:assert np.float32(row['z'])==expected
        if row['kind']=='region' and (u0,v0,u1,v1)==(0,0,width,height):
            assert int(row['pixels_scanned'])==0 and int(row['tiles_used'])==meta['tile_rows']*meta['tile_columns']
    assert meta['per_pixel_validity_mask_created'] is False and meta['point_cloud_created'] is False and meta['environment_map_created'] is False
    return dict(passed=True,source_count=len(sources),observed_cells=meta['observed_cells'],unobserved_cells=meta['unobserved_cells'],
                source_unit='block_mean_center' if block_mode else 'pixel',block_capacity=4*units if block_mode else None,
                camera_observations=meta['camera_observations'],camera_covered_cells=meta['camera_covered_cells'],geometry_rejected=geometry_rejected,
                float_boundary_assignments_within_1e_11_rad=boundary_assignments,region_tiles_verified=len(tiles),query_examples_verified=len(queries),
                source_uniqueness_and_complete_provenance=True,no_empty_cell_interpolation=True,no_cross_camera_minimum=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args();out=args.directory.resolve()
    cal=json.loads((out/'simulation.sensors.json').read_text());width,height=cal['render_width'],cal['render_height']
    repair,filled=verify_repair(out/'raw_depth.bin',out/'filled_depth.bin',width,height);rays=verify_rays(out,cal,width,height)
    provenance=json.loads((out/'provenance.json').read_text()) if (out/'provenance.json').exists() else {}
    index=verify_index(out,'sample',cal,filled,width,height,provenance.get('stamp_ns'),provenance.get('sequence'))
    result=dict(passed=True,scope='offline archived Isaac pixels only; no online/FIFO claim',repair=repair,rays=rays,observations_and_queries=index)
    (out/'direction_query_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(index,indent=2))


if __name__=='__main__':main()
