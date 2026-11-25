#!/usr/bin/env python3
"""Offline evidence analysis only; never runs inside the control/receiver loops."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from verify_depth_preprocessing import verify_repair,verify_rays
from verify_direction_queries import verify_index
from verify_local_field import verify_local


def stats(values):
    a=np.asarray(values,dtype=np.float64)
    if not a.size:return None
    return dict(count=int(a.size),mean=float(a.mean()),p50=float(np.percentile(a,50)),
                p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),p999=float(np.percentile(a,99.9)),max=float(a.max()))


def main():
    p=Path(sys.argv[1]).resolve()
    meta=json.loads((p/'reader_meta.json').read_text())
    manifest=json.loads((p/'manifest.json').read_text())
    cal=json.loads((p/'simulation.sensors.json').read_text())
    sim=json.loads((p/'simulation.json').read_text())
    rows=list(csv.DictReader((p/'reads.csv').open()))
    frames=list(csv.DictReader((p/'frames.csv').open()))
    received=list(csv.DictReader((p/'received.csv').open()))
    assert meta['ok'] and meta['reads']==len(rows)>0
    assert manifest['run_id']==cal['run_id']==meta['run_id']==sim['run_id']
    period_ns=int(meta.get('requested_period_ns',1_000_000))
    period_ms=period_ns/1e6
    assert period_ns>0
    assert np.isclose(sim['actual_physics_dt'],1./manifest['camera_hz'])
    assert np.isclose(sim['actual_render_dt'],1./manifest['camera_hz'])
    if manifest.get('sensor_mode')=='rate':
        assert cal['actual_sensor_tick_rate']==[float(manifest['camera_hz'])]*4
    names=('front','left','back','right')
    ts=[int(r['wall_monotonic_ns']) for r in rows]
    start,end=ts[0],ts[-1]
    wall=(end-start)/1e9
    col=lambda key:[float(r[key]) for r in rows if r[key]!='']
    intervals=np.diff(ts)/1e6
    assert np.allclose(intervals,col('interval_ms'),rtol=1e-9,atol=1e-9)
    assert all(len({int(r[f't{i}_ns']) for i in range(1,5)})==1 for r in rows)
    recv_sets=[{int(r['image_stamp_ns']) for r in received if int(r['camera'])==i} for i in range(4)]
    assert all(all(int(f['image_stamp_ns']) in stamps for stamps in recv_sets) for f in frames)
    sequences=[int(r['sequence']) for r in rows]
    assert all(a<=b for a,b in zip(sequences,sequences[1:]))
    fresh=len(set(sequences))
    assert sum(int(r['new_frame']) for r in rows)==fresh
    # Every scheduled gap equals an explicitly recorded skipped slot count.
    scheduled=[int(r['scheduled_ns']) for r in rows]
    assert all(b-a==period_ns*(1+int(rows[i]['skipped_slots'])) for i,(a,b) in enumerate(zip(scheduled,scheduled[1:])))
    if meta['duration_clock']=='sim':assert meta['measured_sim_duration_s']>=meta['requested_duration_s']
    else:assert wall>=meta['requested_duration_s']
    camera_rates={}
    for i,n in enumerate(names):
        dedup={}
        for r in received:
            t=int(r['wall_monotonic_ns'])
            if int(r['camera'])==i and start<=t<=end:dedup.setdefault(int(r['image_stamp_ns']),t)
        pairs=sorted(dedup.items())
        sim_stamps=[x[0] for x in pairs];wall_stamps=[x[1] for x in pairs]
        duration=(max(wall_stamps)-min(wall_stamps))/1e9 if len(pairs)>1 else 0
        camera_rates[n]=dict(unique_images=len(pairs),
            hz_wall=(len(pairs)-1)/duration if duration else None,
            hz_sim=(len(pairs)-1)*1e9/(sim_stamps[-1]-sim_stamps[0]) if len(pairs)>1 else None,
            interval_wall_ms=stats(np.diff(wall_stamps)/1e6),
            interval_sim_ms=stats(np.diff(sim_stamps)/1e6))
    frame_stamps=[int(f['image_stamp_ns']) for f in frames]
    frame_walls=[int(f['published_ns']) for f in frames]
    copy_ms=[(int(f['published_ns'])-int(f['received_ns']))/1e6 for f in frames]
    pixel_stats={}
    preprocessing=meta.get('depth_preprocessing',{})
    preprocessing_check={}
    direction_query_check={}
    if preprocessing.get('enabled'):
        assert preprocessing['ray_table_builds']==1
        preprocessing_check['rays']=verify_rays(p,cal,meta['width'],meta['height'])
    for prefix in ('first','last'):
        data=np.fromfile(p/f'{prefix}_depth.bin',dtype='<f4')
        assert data.size==4*meta['width']*meta['height']
        data=data.reshape(4,meta['height'],meta['width'])
        filled=None
        if preprocessing.get('enabled'):
            preprocessing_check[prefix],filled=verify_repair(p/f'{prefix}_depth.bin',p/f'{prefix}_filled_depth.bin',meta['width'],meta['height'])
            if meta.get('direction_depth_queries',{}).get('enabled'):
                stamp=meta[prefix+'_image_stamp_ns'];sequence=next(int(f['sequence']) for f in frames if int(f['image_stamp_ns'])==stamp)
                direction_query_check[prefix]=verify_index(p,prefix,cal,filled,meta['width'],meta['height'],stamp,sequence)
        state=meta[prefix+'_state'];outputs={'v_B':np.array(state['v_B']),
            'R_WB':np.array(state['R_WB']).reshape(3,3),'state_t_ns':np.int64(state['t_ns'])}
        assert np.isfinite(outputs['v_B']).all() and np.allclose(outputs['R_WB'].T@outputs['R_WB'],np.eye(3),atol=1e-6)
        assert np.linalg.det(outputs['R_WB'])>0
        if 'p_W' in state:
            outputs['p_W']=np.asarray(state['p_W']);assert outputs['p_W'].shape==(3,) and np.isfinite(outputs['p_W']).all()
        for i,(d,c) in enumerate(zip(data,cal['cameras']),1):
            k=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
            t=np.eye(4);t[:3,:3]=c['robot_from_optical_rotation'];t[:3,3]=c['camera_position_robot_m']
            outputs.update({f'D{i}':d,f'K{i}':k,f'TBC{i}':t,
                f't{i}_ns':np.int64(meta[prefix+'_image_stamp_ns']),f'depth_type{i}':'optical_axis_z_m'})
            if filled is not None:outputs[f'D{i}_filled']=filled[i-1]
            valid=np.isfinite(d)&(d>0)
            pixel_stats[prefix+'_'+names[i-1]]=dict(valid=int(valid.sum()),inf=int(np.isinf(d).sum()),
                nan=int(np.isnan(d).sum()),nonpositive=int((np.isfinite(d)&(d<=0)).sum()),
                min_m=float(d[valid].min()) if valid.any() else None,max_m=float(d[valid].max()) if valid.any() else None)
        np.savez(p/f'{prefix}_input.npz',**outputs)
    local_check={}
    if meta.get('local_field_preparation',{}).get('enabled'):
        for prefix in ('first','last'):
            _,filled=verify_repair(p/f'{prefix}_depth.bin',p/f'{prefix}_filled_depth.bin',meta['width'],meta['height'])
            local_check[prefix]=verify_local(p,prefix,cal,filled,meta['width'],meta['height'])
        for r in rows:
            direction=np.array([float(r['desired_'+k]) for k in ('x','y','z')]);speed=float(r['desired_speed_mps'])
            assert np.isfinite(direction).all() and 0<=speed<=meta["desired_motion"]["max_speed_mps"]
            assert np.isclose(np.linalg.norm(direction),1.,atol=1e-9) or (np.all(direction==0) and speed==0)
    root=Path(__file__).resolve().parents[2]
    source_ok=all(hashlib.sha256((root/k).read_bytes()).hexdigest()==v for k,v in manifest['source_sha256'].items())
    executable_ok=all(hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in manifest['installed_module_sha256'].items())
    assert source_ok and executable_ok
    assert sim['initial_position']==sim['final_position'] and sim['command_publishers_created']==0
    assert sim['scene_sha256_after']==cal['scene_sha256']==hashlib.sha256(Path(manifest['scene']).read_bytes()).hexdigest()
    assert json.loads((p/'process_result.json').read_text())=={'reader':0,'simulator':0}
    s=dict(meta,calibration=cal,requested_camera_hz=manifest['camera_hz'],
        requested_period_ms=period_ms,requested_read_hz=1e9/period_ns,read_only=True,unique_frames_consumed=fresh,repeated_reads=len(rows)-fresh,
        read_ms=stats(col('read_ms')),work_ms=stats(col('work_ms')),interval_ms=stats(intervals),
        wake_lateness_ms=stats(col('wake_lateness_ms')),read_over_period=sum(v>period_ms for v in col('read_ms')),
        interval_over_period=int((intervals>period_ms).sum()),interval_over_2_2ms=int((intervals>2.2).sum()),
        interval_below_2_2ms=int((intervals<2.2).sum()),interval_over_period_plus_5pct=int((intervals>period_ms*1.05).sum()),
        interval_over_period_plus_50pct=int((intervals>period_ms*1.5).sum()),skipped_periods=sum(int(r['skipped_slots']) for r in rows),
        deadline_misses=sum(float(r['wake_lateness_ms'])+float(r['work_ms'])>period_ms for r in rows),
        depth_age_ms=stats(col('depth_age_ms')),buffer_age_ms=stats(col('buffer_age_ms')),
        state_age_ms=stats(col('state_age_ms')),observed_reads_per_wall_s=(len(rows)-1)/wall,
        producer_sets_per_wall_s=(len(frames)-1)*1e9/(frame_walls[-1]-frame_walls[0]) if len(frames)>1 else None,
        producer_sets_per_sim_s=(len(frames)-1)*1e9/(frame_stamps[-1]-frame_stamps[0]) if len(frames)>1 else None,
        receiver_copy_publish_ms=stats(copy_ms),camera_rates=camera_rates,pixel_stats=pixel_stats,
        max_sync_spread_ns=0,strict_every_poll_under_period=bool(np.all(intervals<=period_ms)),
        timing_scope='C++ read_ms: protected latest frame/state acquisition, filled-depth/ray probes and cached pixel/ROI queries when enabled; copy mode also copies raw+filled images. New-group repair/scan/projection/merge and all file IO/statistics excluded. Parallel repair timings overlap camera_work_ms.' if preprocessing.get('enabled') else 'C++ read_ms: protected latest frame/state acquisition plus four raw data probes; receiver and file IO excluded.',
        data_age_scope='depth_age_ms in ROS simulation time; buffer_age_ms wall time since complete-buffer publication, not exposure-to-read wall age',
        bounded_image_storage='3 ownership-protected raw+filled frame slots; max4 partial DDS groups; fixed first snapshot and optional reader copy; immutable pixel-ray lookup tables; no frame history queue' if preprocessing.get('enabled') else '3 ownership-protected raw frame slots; no frame history queue')
    if preprocessing.get('enabled'):
        s['preprocessing_validation']=preprocessing_check
        s['receiver_raw_copy_ms']=stats([float(f['raw_copy_ms']) for f in frames])
        if preprocessing.get('location')=='camera_workers_before_scan':
            s['receiver_repair_max_elapsed_ms']=stats([float(f['repair_max_ms']) for f in frames])
            s['repair_timing_scope']=preprocessing['repair_timing_scope']
        else:
            s['receiver_depth_repair_ms']=stats([float(f['preprocess_ms']) for f in frames])
        s['repair_cameras']=[]
        for i in range(1,5):
            assert all(int(f[f'invalid{i}'])==int(f[f'filled{i}'])+int(f[f'unresolved{i}']) for f in frames)
            s['repair_cameras'].append(dict(invalid_total=sum(int(f[f'invalid{i}']) for f in frames),
                filled_total=sum(int(f[f'filled{i}']) for f in frames),unresolved_total=sum(int(f[f'unresolved{i}']) for f in frames),
                repair_ms=stats([float(f[f'repair{i}_ms']) for f in frames])))
    if meta.get('direction_depth_queries',{}).get('enabled'):
        assert set(direction_query_check)=={'first','last'}
        s['direction_query_validation']=direction_query_check
        if meta['direction_depth_queries'].get('block_edge'):
            s['receiver_camera_work_ms']=stats([float(f['camera_work_ms']) for f in frames])
            s['receiver_index_merge_ms']=stats([float(f['index_merge_ms']) for f in frames])
        else:
            s['receiver_direction_index_ms']=stats([float(f['direction_index_ms']) for f in frames])
            s['receiver_region_index_ms']=stats([float(f['region_index_ms']) for f in frames])
        assert all(int(r['region_total_pixels'])==meta['width']*meta['height'] and int(r['region_pixels_scanned'])==0 for r in rows)
        s['query_scope']='read_ms includes one pixel and one tiled full-front-image minimum query; only numeric coverage, no U(Q)/free-domain decision'
        s['timing_scope']+=' Direction/region index construction is in the receiver before publication, outside read_ms and reported separately.'
    if local_check:
        s['local_field_validation']=local_check
        shared='shared_projection_local_max_ms' if 'shared_projection_local_max_ms' in frames[0] else 'local_camera_max_ms'
        s['receiver_'+shared]=stats([float(f[shared]) for f in frames])
        s['receiver_local_merge_ms']=stats([float(f['local_merge_ms']) for f in frames])
        s['receiver_input_pipeline_ms']=stats([float(f['camera_work_ms'])+float(f['index_merge_ms'])+float(f['local_merge_ms']) for f in frames])
        s['timing_scope']+=' Local-camera work overlaps camera_work_ms; local_merge_ms follows index_merge_ms. read_ms also includes desired direction and cached local counts.'
    (p/'reader_summary.json').write_text(json.dumps(s,indent=2,allow_nan=False)+'\n')
    (p/'validation.json').write_text(json.dumps(dict(passed=True,csv_schedule_and_interval_recomputed=True,
        exact_stamps_confirmed_in_all_four_receive_streams=True,source_and_elf_sha_match=True,
        first_last_array_and_state_valid=True,scene_unchanged=True,processes_exit_zero=True,
        first_last_depth_preprocessing_verified=bool(preprocessing.get('enabled')),
        pixel_ray_tables_verified=bool(preprocessing.get('enabled')),
        first_last_direction_sources_and_region_queries_verified=bool(meta.get('direction_depth_queries',{}).get('enabled')),
        first_last_local_field_verified=bool(local_check),
        requested_period_ms=period_ms,strict_period_passed=s['strict_every_poll_under_period']),indent=2)+'\n')
    print(json.dumps({k:s[k] for k in ('read_mode','reads','requested_period_ms','read_ms','interval_ms','skipped_periods','producer_sets_per_wall_s','producer_sets_per_sim_s')},indent=2))


if __name__=='__main__':main()
