#!/usr/bin/env python3
"""Independent offline NumPy verification of C++ depth repair and ray artifacts.

The running algorithm is entirely C++; this reference is used after a run or
on explicitly archived-pixel benchmarks, never in reception/control threads.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def verify_repair(raw_path, filled_path, width, height):
    raw=np.fromfile(raw_path,dtype='<f4').reshape(4,height,width)
    filled=np.fromfile(filled_path,dtype='<f4').reshape(raw.shape)
    valid=np.isfinite(raw)&(raw>0)
    total=np.zeros(raw.shape,dtype=np.float64)
    count=np.zeros(raw.shape,dtype=np.uint8)
    # Shift whole original arrays instead of calling the C++ pixel routine.
    for dy in (-1,0,1):
        for dx in (-1,0,1):
            y0,y1=max(0,-dy),min(height,height-dy)
            x0,x1=max(0,-dx),min(width,width-dx)
            neighbor=raw[:,y0+dy:y1+dy,x0+dx:x1+dx]
            mask=valid[:,y0+dy:y1+dy,x0+dx:x1+dx]
            total[:,y0:y1,x0:x1]+=np.where(mask,neighbor,0).astype(np.float64)
            count[:,y0:y1,x0:x1]+=mask
    repaired=(~valid)&(count>0)
    unchanged=~repaired
    expected=raw.copy()
    expected[repaired]=(total[repaired]/count[repaired]).astype(np.float32)
    # Checks all bits, including untouched valid values and unresolved NaNs.
    assert np.array_equal(filled.view('<u4'),expected.view('<u4')), 'filled output differs from original-only 3x3 reference'
    assert np.array_equal(filled.view('<u4')[unchanged],raw.view('<u4')[unchanged])
    result=[]
    for camera in range(4):
        result.append(dict(invalid=int((~valid[camera]).sum()),filled=int(repaired[camera].sum()),
                           unresolved=int(((~valid[camera])&(count[camera]==0)).sum()),
                           valid_unchanged_bitwise=True,unresolved_unchanged_bitwise=True))
    return result,filled


def verify_rays(directory,cal,width,height):
    b=np.fromfile(directory/'pixel_rays_b.bin',dtype='<f8').reshape(4,height,width,3)
    d=np.fromfile(directory/'pixel_rays_d.bin',dtype='<f8').reshape(b.shape)
    u,v=np.meshgrid(np.arange(width),np.arange(height))
    pixel=np.stack((u,v,np.ones_like(u)),axis=-1)
    errors=[]
    for camera,c in enumerate(cal['cameras']):
        k=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1]],dtype=np.float64)
        expected=pixel@np.linalg.inv(k).T
        unit=expected/np.linalg.norm(expected,axis=-1,keepdims=True)
        assert np.isfinite(b[camera]).all() and np.isfinite(d[camera]).all()
        assert np.allclose(b[camera],expected,rtol=1e-12,atol=1e-12)
        assert np.allclose(d[camera],unit,rtol=1e-12,atol=1e-12)
        assert np.allclose(np.linalg.norm(d[camera],axis=-1),1.,rtol=0,atol=1e-12)
        errors.append(dict(b_max_abs_error=float(np.max(np.abs(b[camera]-expected))),
                           d_max_abs_error=float(np.max(np.abs(d[camera]-unit)))))
    return dict(shape=[4,height,width,3],dtype='float64_le',frame='camera_optical',cameras=errors)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('benchmark_directory',type=Path)
    args=parser.parse_args();out=args.benchmark_directory.resolve()
    cal=json.loads((out/'simulation.sensors.json').read_text());meta=json.loads((out/'benchmark.json').read_text())
    width,height=meta['width'],meta['height']
    repair,_=verify_repair(out/'raw_depth.bin',out/'filled_depth.bin',width,height)
    rays=verify_rays(out,cal,width,height)
    assert meta['ray_builds']==1
    assert all({k:row[k] for k in ('invalid','filled','unresolved')}=={k:expected[k] for k in ('invalid','filled','unresolved')}
               for row,expected in zip(meta['cameras'],repair))
    result=dict(passed=True,scope='offline real archived Isaac depth; no new simulation/FIFO scheduling claim',
                repair=repair,rays=rays,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'preprocess_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
