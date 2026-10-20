"""Negative controls for arrival evidence: geometry, limits and process failure."""
import csv
import json
import numpy as np
import pytest
from speed_benchmark import audit


def test_current_map_and_protocol_use_the_same_full_geometry():
    from speed_benchmark import MAPS
    from sim4_benchmark import parser, protocol_for
    assert list(MAPS) == ['original']
    scene, shape, goal = MAPS['original']
    manifest = json.loads(scene.with_name('manifest.json').read_text())
    assert list(shape) == manifest['shape']
    assert scene.with_name('occupancy.bin').stat().st_size == int(np.prod(shape))
    assert goal == manifest['goal']
    cli = parser()
    protocol = protocol_for(cli.parse_args(['unified_map_check']), 'original')
    assert protocol['goal'] == goal
    assert protocol['timeout_s'] == 600.
    with pytest.raises(SystemExit):
        cli.parse_args(['unified_map_check', '--maps', 'long'])


@pytest.mark.parametrize('failure', ['none', 'voxel', 'ground', 'speed', 'process', 'guard', 'config'])
def test_independent_audit_refuses_false_arrivals(tmp_path, failure):
    scene = tmp_path/'scene.usda'
    scene.write_text('# fixture geometry')
    shape = (20, 20, 20)
    grid = np.zeros(shape, np.uint8)
    if failure == 'voxel':
        grid[5, 5, 4] = 1
    grid.tofile(tmp_path/'occupancy.bin')
    result = tmp_path/'run.json'
    protocol = dict(body_radius_m=.48, max_speed_mps=2., max_vertical_speed_mps=1.,
                    plant_accel_mps2=1.2, warmup_s=0.)
    result.write_text(json.dumps(dict(protocol=protocol, status='ARRIVED', collision_blocks=int(failure=='guard'),
        process_exit_code=int(failure=='process'), simulation_elapsed_s=2., wall_elapsed_s=2.)))
    effective = dict(max_speed=3. if failure=='config' else 2., command_accel_limit=1.2, body_radius=.48, max_depth=10.)
    result.with_suffix('.parameters.json').write_text(json.dumps(dict(effective=effective)))
    result.with_suffix('.log').write_text('')
    keys = ['t','x','y','z','vx','vy','vz','qx','qy','qz','applied_target_x','applied_target_y','applied_target_z','status']
    with result.with_suffix('.csv').open('w') as stream:
        w=csv.DictWriter(stream, fieldnames=keys);w.writeheader()
        for t in range(3):
            row=dict.fromkeys(keys, 0.);row.update(t=t,x=-78.,y=-58.,z=.1 if failure=='ground' else 1.6,
                vx=3. if failure=='speed' else 0.,qx=1.,status='FIXTURE');w.writerow(row)
    report = audit(result, scene, shape)
    assert report['safe_arrival'] == (failure == 'none')
