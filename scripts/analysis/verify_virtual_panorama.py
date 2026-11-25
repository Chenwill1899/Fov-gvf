#!/usr/bin/env python3
"""Independent stopped-run checks for the explicitly approximate virtual model."""
import argparse,csv,json,math
from pathlib import Path
import numpy as np
from verify_depth_preprocessing import verify_repair,verify_rays
from verify_direction_queries import verify_index


def verify_virtual(directory,prefix,cal,filled,width,height):
    directory=Path(directory);meta=json.loads((directory/f'{prefix}_local_field.json').read_text());index=json.loads((directory/f'{prefix}_observation_index.json').read_text())
    assert meta['version']=='ego4p0_local_field_v4_virtual_center' and meta['translation_ignored'] and not meta['physical_near_clip_checked'] and meta['rotations_retained']
    limit=meta['local_range_m'];radius=meta['radius_m'];bin_m=meta['distance_bin_m'];n_bins=meta['distance_bins']
    assert n_bins==round(limit/bin_m)+1
    na,nb=index['azimuth_bins'],index['elevation_bins'];cells=na*nb;n=width*height;edge=index['block_edge'];columns=index['block_columns'];rows=index['block_rows'];blocks=columns*rows
    raw=np.fromfile(directory/'raw_depth.bin' if (directory/'raw_depth.bin').exists() else directory/f'{prefix}_depth.bin',dtype='<f4').reshape(4,height,width)
    scales=np.fromfile(directory/f'{prefix}_virtual_coefficients.bin',dtype='<f8').reshape(4,height,width)
    pixel_cells=np.fromfile(directory/f'{prefix}_virtual_pixel_cells.bin',dtype='<u4').reshape(4,height,width)
    block_cells=np.fromfile(directory/f'{prefix}_virtual_block_cells.bin',dtype='<u4').reshape(4,blocks)
    sources=np.fromfile(directory/f'{prefix}_virtual_node_blocks.bin',dtype='<u4').reshape(cells,4)
    rho=np.fromfile(directory/f'{prefix}_panorama_rho.bin',dtype='<f8');obs=np.fromfile(directory/f'{prefix}_panorama_camera_mask.bin',dtype=np.uint8);complete=np.fromfile(directory/f'{prefix}_panorama_complete_mask.bin',dtype=np.uint8)
    excluded=np.fromfile(directory/f'{prefix}_local_excluded_camera_mask.bin',dtype=np.uint8);states=np.fromfile(directory/f'{prefix}_panorama_state.bin',dtype=np.uint8)
    missing_prefix=np.fromfile(directory/f'{prefix}_panorama_missing_prefix.bin',dtype='<u4').reshape(nb+1,na+1)
    assert all(a.shape==(cells,) for a in [rho,obs,complete,excluded,states])
    alpha_step=np.deg2rad(index['azimuth_resolution_deg']);beta_step=np.deg2rad(index['elevation_resolution_deg'])
    ac=(-np.pi+np.arange(na)*alpha_step+np.minimum(-np.pi+(np.arange(na)+1)*alpha_step,np.pi))/2
    bc=(-np.pi/2+np.arange(nb)*beta_step+np.minimum(-np.pi/2+(np.arange(nb)+1)*beta_step,np.pi/2))/2
    alpha,beta=np.meshgrid(ac,bc);directions=np.stack((np.cos(beta)*np.cos(alpha),np.cos(beta)*np.sin(alpha),np.sin(beta)),axis=-1).reshape(cells,3)
    uu,vv=np.meshgrid(np.arange(columns)*edge,np.arange(rows)*edge);u0=uu.ravel();v0=vv.ravel();u1=np.minimum(u0+edge,width);v1=np.minimum(v0+edge,height);total=(u1-u0)*(v1-v0)
    def grouped(image,fill):
        padded=np.full((rows*edge,columns*edge),fill,dtype=float);padded[:height,:width]=image
        return padded.reshape(rows,edge,columns,edge).transpose(0,2,1,3).reshape(blocks,edge*edge)
    def cell_ids(points):
        a=np.arctan2(points[...,1],points[...,0]);a=np.where(a==np.pi,-np.pi,a);a=np.where((points[...,0]==0)&(points[...,1]==0),0,a)
        b=np.arctan2(points[...,2],np.hypot(points[...,0],points[...,1]));return np.minimum(((b+np.pi/2)/beta_step).astype(int),nb-1)*na+np.minimum(((a+np.pi)/alpha_step).astype(int),na-1)
    def check_bins(actual,points):
        a=np.arctan2(points[...,1],points[...,0]);b=np.arctan2(points[...,2],np.hypot(points[...,0],points[...,1]));aa=actual%na;bb=actual//na
        lower=-np.pi+aa*alpha_step;upper=np.minimum(lower+alpha_step,np.pi);delta=np.arctan2(np.sin(a-(lower+upper)/2),np.cos(a-(lower+upper)/2))
        assert np.all(np.abs(delta)<=(upper-lower)/2+1e-11)
        lower=-np.pi/2+bb*beta_step;upper=np.minimum(lower+beta_step,np.pi/2);assert np.all((b>=lower-1e-11)&(b<=upper+1e-11))
    all_min=[];all_raw_min=[];all_count=[];all_raw_count=[];all_q=[];maximum_half=0.
    for c,camera in enumerate(cal['cameras']):
        K=np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]]);inverse=np.linalg.inv(K);R=np.asarray(camera['robot_from_optical_rotation'])
        x,y=np.meshgrid(np.arange(width),np.arange(height));b=np.stack((x,y,np.ones_like(x)),axis=-1)@inverse.T;q=b@R.T
        expected_scale=np.linalg.norm(b,axis=-1);assert np.allclose(scales[c],expected_scale,rtol=1e-13,atol=1e-13);check_bins(pixel_cells[c],q)
        center_pixels=np.stack(((u0+u1-1)*.5,(v0+v1-1)*.5,np.ones(blocks)),axis=-1);center=center_pixels@inverse.T@R.T;check_bins(block_cells[c],center)
        norm=np.linalg.norm(center,axis=-1);deviation=np.zeros(blocks)
        for u in [u0-.5,u1-.5]:
            for v in [v0-.5,v1-.5]:
                corner=np.stack((u,v,np.ones(blocks)),axis=-1)@inverse.T@R.T;deviation=np.maximum(deviation,np.linalg.norm(corner-center,axis=-1))
        half=np.where(deviation>=norm,np.pi,np.arcsin(np.minimum(1,deviation/norm)));maximum_half=max(maximum_half,float(half.max()))
        image=grouped(filled[c],np.nan);valid=np.isfinite(image)&(image>0);coefficient=grouped(expected_scale,np.nan)
        minimum=np.min(np.where(valid,image*coefficient,np.inf),axis=-1);raw_image=grouped(raw[c],np.nan);raw_valid=np.isfinite(raw_image)&(raw_image>0)
        raw_minimum=np.min(np.where(raw_valid,raw_image*coefficient,np.inf),axis=-1)
        all_min.append(minimum);all_raw_min.append(raw_minimum);all_count.append(valid.sum(-1));all_raw_count.append(raw_valid.sum(-1));all_q.append(q)
        # The lookup must come from grid-center projection using R only.
        optical=directions@R;z=optical[:,2];safe=np.where(z>0,z,1);u=(K[0,0]*optical[:,0]+K[0,1]*optical[:,1])/safe+K[0,2];v=(K[1,0]*optical[:,0]+K[1,1]*optical[:,1])/safe+K[1,2]
        inside=(z>0)&(u>=-.5)&(u<width-.5)&(v>=-.5)&(v<height-.5);expected=np.full(cells,np.iinfo(np.uint32).max,dtype=np.uint32)
        expected[inside]=((v[inside]+.5)/edge).astype(int)*columns+((u[inside]+.5)/edge).astype(int)
        assert np.array_equal(sources[:,c],expected), 'virtual lookup uses wrong orientation or physical offset'
    all_min=np.array(all_min);all_raw_min=np.array(all_raw_min);all_count=np.array(all_count);all_raw_count=np.array(all_raw_count)
    assert abs(maximum_half-meta['maximum_block_half_angle_rad'])<1e-12
    records=list(csv.DictReader((directory/f'{prefix}_virtual_blocks.csv').open()));assert len(records)==4*blocks
    for code,row in enumerate(records):
        c,bid=divmod(code,blocks);assert int(row['camera'])==c and int(row['block_id'])==bid
        assert [int(row[k]) for k in ['valid_pixels','raw_valid_pixels','total_pixels']]==[all_count[c,bid],all_raw_count[c,bid],total[bid]]
        for key,expected in [('min_rho',all_min[c,bid]),('raw_min_rho',all_raw_min[c,bid])]:
            assert np.isclose(float(row[key]),expected,rtol=1e-13,atol=1e-13) if np.isfinite(expected) else row[key]==''
    # Independently derive the conservative row/bin templates.
    templates=np.zeros((2,nb,n_bins,4),dtype=int);quantization=(alpha_step+beta_step)/2
    for obstacle in range(2):
        for b in range(nb):
            for k in range(n_bins):
                lower=max(0,k*bin_m-1e-9);inflation=np.pi if lower<=radius else math.asin(radius/lower);delta=inflation+quantization+(maximum_half if obstacle else 0)
                if delta>=np.pi/2:padding=na;lo=0;hi=nb
                else:
                    low=max(-np.pi/2,bc[b]-delta-1e-10);high=min(np.pi/2,bc[b]+delta+1e-10)
                    lo=min(max(0,math.floor((low+np.pi/2)/beta_step)),nb-1);hi=min(math.floor((high+np.pi/2)/beta_step)+1,nb)
                    padding=na if abs(bc[b])+delta+1e-10>=np.pi/2 else math.ceil((math.asin(min(1,math.sin(delta)/math.cos(bc[b])))+1e-10)/alpha_step)+1
                a0,a1=(0,na) if padding*2+1>=na else ((-padding)%na,(padding+1)%na)
                templates[obstacle,b,k]=[a0,a1,lo,hi]
    rows_csv=list(csv.DictReader((directory/f'{prefix}_virtual_templates.csv').open()));assert len(rows_csv)==2*nb*n_bins
    for row in rows_csv:
        obstacle,b,k=[int(row[name]) for name in ['obstacle','row','bin']];assert np.array_equal(templates[obstacle,b,k],[int(row[name]) for name in ['a0','a1','b0','b1']])
    def windows(ids,bins,obstacle=False):
        ids=np.asarray(ids);base=templates[int(obstacle),ids//na,bins].copy();full=(base[...,0]==0)&(base[...,1]==na);a=ids%na
        base[...,0]=np.where(full,0,(base[...,0]+a)%na);base[...,1]=np.where(full,na,(base[...,1]+a)%na);return base
    expected_excluded=np.zeros((nb,na),dtype=np.uint8);near_total=0
    for c in range(4):
        near=np.flatnonzero(all_min[c]<=limit);near_total+=len(near);bins=np.clip(np.floor(all_min[c,near]/bin_m).astype(int),0,n_bins-1);boxes=windows(block_cells[c,near],bins,True)
        difference=np.zeros((nb+1,na+1),dtype=np.int64)
        def add(a0,a1,b0,b1):
            np.add.at(difference,(b0,a0),1);np.add.at(difference,(b0,a1),-1);np.add.at(difference,(b1,a0),-1);np.add.at(difference,(b1,a1),1)
        a0,a1,b0,b1=boxes.T;wrap=a0>a1;add(a0[~wrap],a1[~wrap],b0[~wrap],b1[~wrap]);add(a0[wrap],np.full(wrap.sum(),na),b0[wrap],b1[wrap]);add(np.zeros(wrap.sum(),dtype=int),a1[wrap],b0[wrap],b1[wrap])
        expected_excluded[difference.cumsum(0).cumsum(1)[:nb,:na]>0]|=np.uint8(1<<c)
    assert np.array_equal(excluded.reshape(nb,na),expected_excluded) and meta['excluded_blocks']==near_total and meta['excluded_cells']==int((excluded!=0).sum())
    expected_rho=np.full(cells,np.inf);expected_obs=np.zeros(cells,dtype=np.uint8);expected_complete=expected_obs.copy()
    for c in range(4):
        ids=np.flatnonzero(sources[:,c]!=np.iinfo(np.uint32).max);bid=sources[ids,c];values=all_min[c,bid];valid=np.isfinite(values)
        expected_rho[ids]=np.minimum(expected_rho[ids],values);expected_obs[ids[valid]]|=np.uint8(1<<c)
        full=(all_raw_count[c,bid]==total[bid])&np.isfinite(all_raw_min[c,bid]);expected_complete[ids[full]]|=np.uint8(1<<c)
    assert np.array_equal(obs,expected_obs) and np.array_equal(complete,expected_complete)
    assert np.allclose(rho, np.where(np.isfinite(expected_rho),expected_rho,np.nan),equal_nan=True,rtol=1e-13,atol=1e-13)
    def integral(bad):return np.pad(bad.reshape(nb,na).cumsum(0).cumsum(1),((1,0),(1,0)))
    expected_missing=integral(complete==0);assert np.array_equal(missing_prefix,expected_missing)
    far_bad=integral((complete==0)|~(expected_rho>limit+radius))
    def sums(prefix,boxes):
        a0,a1,b0,b1=np.asarray(boxes).T
        def rectangle(x0,x1):return prefix[b1,x1]-prefix[b0,x1]-prefix[b1,x0]+prefix[b0,x0]
        return np.where(a0<=a1,rectangle(a0,a1),rectangle(a0,np.full(a0.shape,na))+rectangle(np.zeros(a0.shape,dtype=int),a1))
    ids=np.arange(cells);far_boxes=windows(ids,np.full(cells,n_bins-1));expected_states=np.zeros(cells,dtype=np.uint8);expected_states[(obs!=0)&(excluded!=0)]=1
    expected_states[(obs!=0)&(excluded==0)&(complete!=0)&(sums(far_bad,far_boxes)==0)]=2
    assert np.array_equal(states,expected_states) and meta['available_cells']==int((states==2).sum()) and meta['unknown_cells']==int((states==0).sum())
    assert np.all(states[obs==0]==0), 'uncovered directions fabricated free'
    tiles=list(csv.DictReader((directory/f'{prefix}_panorama_tiles.csv').open()));tile_columns=(na+7)//8
    for row in tiles:
        by,ax=divmod(int(row['tile']),tile_columns);values=rho.reshape(nb,na)[by*8:min(by*8+8,nb),ax*8:min(ax*8+8,na)];valid=values[np.isfinite(values)&(values>0)]
        assert int(row['valid'])==len(valid);assert np.isclose(float(row['min_rho']),valid.min(),rtol=1e-13,atol=1e-13) if len(valid) else row['min_rho']==''
    query_rows=list(csv.DictReader((directory/f'{prefix}_virtual_queries.csv').open()))
    for row in query_rows:
        center=np.array([float(row[k]) for k in ['x','y','z']]);distance=np.linalg.norm(center);cell=int(cell_ids(center));assert int(row['cell'])==cell
        box=windows(np.array([cell]),np.array([min(n_bins-1,int(math.floor(distance/bin_m))) ]))[0];assert np.array_equal(box,[int(row[k]) for k in ['a0','a1','b0','b1']])
        a0,a1,b0,b1=box;region=rho.reshape(nb,na)[b0:b1];values=region[:,a0:a1] if a0<=a1 else np.concatenate((region[:,a0:],region[:,:a1]),axis=1);valid=values[np.isfinite(values)&(values>0)];minimum=float(valid.min()) if valid.size else None;missing=int(sums(expected_missing,box[None,:])[0])
        assert int(row['missing'])==missing and int(row['total'])==values.size
        assert np.isclose(float(row['min_rho']),minimum,rtol=1e-13,atol=1e-13) if minimum is not None else row['min_rho']==''
        expected=0 if obs[cell]==0 else 1 if excluded[cell]!=0 else 1 if minimum is not None and minimum<=distance+radius else 0 if missing else 2
        assert int(row['state'])==expected
    assert meta['hot_dynamic_angle_binning'] is False and meta['hot_physical_projection_queries'] is False and meta['hot_pixel_coordinate_transforms']==0
    return dict(passed=True,virtual_center=True,translation_ignored=True,physical_near_clip_checked=False,coefficients_verified=4*n,fixed_pixel_cells_verified=4*n,source_node_lookups_verified=4*cells,block_ranges_verified=4*blocks,inflation_templates_verified=len(rows_csv),panorama_cells_verified=cells,excluded_blocks=near_total,excluded_cells=int((excluded!=0).sum()),available_cells=int((states==2).sum()),unknown_cells=int((states==0).sum()),unavailable_cells=int((states==1).sum()),query_examples_verified=len(query_rows),range_tiles_verified=len(tiles),rho_merge='minimum common radial distance, never minimum optical Z',raw_complete_required=True,missing_directions_unknown=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);args=p.parse_args();out=args.directory.resolve();cal=json.loads((out/'simulation.sensors.json').read_text());w,h=cal['render_width'],cal['render_height']
    repair,filled=verify_repair(out/'raw_depth.bin',out/'filled_depth.bin',w,h);rays=verify_rays(out,cal,w,h);index=verify_index(out,'sample',cal,filled,w,h);local=verify_virtual(out,'sample',cal,filled,w,h)
    result=dict(passed=True,scope='offline numerical validation of accepted approximate body-origin model; no real-camera geometric guarantee or online timing',repair=repair,rays=rays,observations=index,local=local);(out/'local_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(local,indent=2))

if __name__=='__main__':main()
