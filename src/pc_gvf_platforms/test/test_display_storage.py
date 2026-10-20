import numpy as np
from pc_gvf_platforms.observed_map_visualizer import ObservedVoxelMap
from pc_gvf_platforms.command_bridge import TimingSeries


def test_display_lru_and_incremental_coarse_counts_remain_bounded():
    m=ObservedVoxelMap(.1,2,maximum_voxels=32,maximum_candidates=8)
    for i in range(500):
        point=np.asarray([[i*.1+.001,0,1.]])
        m.integrate(point);m.integrate(point)
        if i%7==0:m.points(.3,12)
        assert len(m.occupied_voxels)<=32 and len(m.voxel_hits)<=8
    actual=m.points(.3,12)
    centers=(np.asarray(tuple(m.occupied_voxels))+.5)*.1
    expected=np.unique(np.floor(centers/m._publication_resolution).astype(np.int64),axis=0)
    assert set(map(tuple,expected))==set(m._coarse_counts)
    assert actual.shape[0]<=12 and m.evicted_voxels==468


def test_evicted_pending_hit_does_not_become_confirmation():
    m=ObservedVoxelMap(.1,2,maximum_voxels=32,maximum_candidates=4)
    for i in range(100):m.integrate(np.asarray([[i,0,0.]]))
    m.integrate(np.asarray([[0.,0.,0.]]))
    assert len(m.voxel_hits)==4 and not m.occupied_voxels


def test_bridge_timing_has_lifetime_mean_and_max_with_bounded_percentiles():
    s=TimingSeries()
    for i in range(100000):s.add(i)
    assert s.count==100000 and len(s.values)==8192
    assert s.mean()==49999.5 and s.maximum()==99999
    assert s.percentile(.5)==95904
