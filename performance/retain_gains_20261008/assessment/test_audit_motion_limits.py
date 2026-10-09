"""Synthetic, bounded fail-closed tests. Never reads a recorded Isaac trajectory."""
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest

import audit_motion_limits as audit


def sample(t, velocity=(0., 0., 0.), target=(0., 0., 0.)):
    return dict(zip(audit.COLUMNS, (t, 0., 0., 0., *velocity, *target)))


class MotionAuditTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.csv = self.root / 'trace.csv'

    def write_csv(self, rows, columns=audit.COLUMNS):
        with self.csv.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
            writer.writeheader(); writer.writerows(rows)
        return self.csv

    def report(self, rows):
        result = audit.audit_csv(self.write_csv(rows))
        json.dumps(result, allow_nan=False)
        return result

    def test_steady_valid_and_hash(self):
        result = self.report([sample(0., (1., 0., 0.)), sample(1., (1., 0., 0.))])
        self.assertEqual(result['status'], 'PASS')
        self.assertEqual(result['sha256'], audit.digest(self.csv.read_bytes()))
        self.assertEqual(result['metrics']['actual_acceleration_mps2']['finite_values_checked'], 1)

    def test_vector_speed_exceeds_even_if_components_do_not(self):
        result = self.report([sample(0., (1.5, 1.5, 0.)), sample(1., (1.5, 1.5, 0.))])
        metric = result['metrics']['actual_speed_mps']
        self.assertEqual(result['status'], 'FAIL'); self.assertEqual(metric['violation_count'], 2)
        self.assertGreater(metric['raw_max'], 2.)
        self.assertEqual(metric['first_violation']['t_s'], 0.)

    def test_actual_vertical_excess(self):
        result = self.report([sample(0., (0., 0., 1.01)), sample(1., (0., 0., 1.01))])
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['metrics']['actual_vertical_speed_mps']['violation_count'], 2)

    def test_actual_acceleration_excess_in_early_interval(self):
        result = self.report([sample(0.), sample(1., (1.2001, 0., 0.)), sample(2., (1.2001, 0., 0.))])
        metric = result['metrics']['actual_acceleration_mps2']
        self.assertEqual(result['status'], 'FAIL'); self.assertEqual(metric['violation_count'], 1)
        self.assertEqual(metric['first_violation']['interval_start_s'], 0.)
        self.assertEqual(metric['first_violation']['t_s'], 1.)
        self.assertEqual(result['metrics']['actual_speed_mps']['violation_count'], 0)

    def test_applied_target_limits_independent_of_actual_motion(self):
        for target, metric in [((1.5, 1.5, 0.), 'applied_target_speed_mps'),
                               ((0., 0., 1.01), 'applied_target_vertical_speed_mps')]:
            with self.subTest(target=target):
                result = self.report([sample(0., target=target), sample(1., target=target)])
                self.assertEqual(result['status'], 'FAIL')
                self.assertEqual(result['metrics'][metric]['violation_count'], 2)
                self.assertEqual(result['metrics']['actual_acceleration_mps2']['violation_count'], 0)

    def test_immediate_release_target_is_legal(self):
        result = self.report([sample(0., (1., 0., 0.), (2., 0., 0.)),
                              sample(1./60., (.98, 0., 0.), (0., 0., 0.))])
        self.assertEqual(result['status'], 'PASS')
        self.assertAlmostEqual(result['metrics']['actual_acceleration_mps2']['raw_max'], 1.2)

    def test_all_required_fields_reject_nan_inf(self):
        for column in audit.COLUMNS:
            for value in ('nan', 'inf', '-inf'):
                with self.subTest(column=column, value=value):
                    rows = [sample(0.), sample(1.)]; rows[1][column] = value
                    result = self.report(rows)
                    self.assertEqual(result['status'], 'FAIL')
                    self.assertEqual(result['invalid_sample_count'], 1)

    def test_missing_columns_and_bad_fields(self):
        for column in audit.COLUMNS:
            with self.subTest(missing=column):
                self.write_csv([sample(0.), sample(1.)], [c for c in audit.COLUMNS if c != column])
                self.assertEqual(audit.audit_csv(self.csv)['status'], 'FAIL')
        for value in ('', None, 'not-a-number'):
            with self.subTest(value=value):
                rows = [sample(0.), sample(1.)]; rows[1]['vy'] = value
                self.assertEqual(self.report(rows)['status'], 'FAIL')

    def test_duplicate_column_and_extra_value_fail(self):
        self.write_csv([sample(0.), sample(1.)], (*audit.COLUMNS, 'vx'))
        self.assertEqual(audit.audit_csv(self.csv)['status'], 'FAIL')
        self.write_csv([sample(0.), sample(1.)])
        with self.csv.open('a') as stream:
            stream.write(','.join(['2'] + ['0'] * len(audit.COLUMNS)) + '\n')
        self.assertEqual(audit.audit_csv(self.csv)['status'], 'FAIL')

    def test_time_duplicate_reverse_and_nonfinite_interval_fail(self):
        for times in ((0., 0.), (1., 0.), (-1e308, 1e308)):
            with self.subTest(times=times):
                result = self.report([sample(t) for t in times])
                self.assertEqual(result['status'], 'FAIL')
                self.assertTrue(result['integrity_errors'])

    def test_empty_header_only_single_frame_and_missing_file_fail(self):
        self.csv.write_text('')
        self.assertEqual(audit.audit_csv(self.csv)['status'], 'FAIL')
        for rows in ([], [sample(0.)]):
            self.assertEqual(self.report(rows)['status'], 'FAIL')
        self.assertEqual(audit.audit_csv(self.root / 'missing.csv')['status'], 'FAIL')

    def test_tolerance_is_explicit_and_bounded(self):
        for extra, expected in ((.5e-6, 'PASS'), (2e-6, 'FAIL')):
            with self.subTest(extra=extra):
                result = self.report([sample(0., (2.+extra, 0., 0.)), sample(1., (2.+extra, 0., 0.))])
                self.assertEqual(result['status'], expected)
                self.assertEqual(result['metrics']['actual_speed_mps']['floating_tolerance'], 1e-6)

    def test_derived_overflow_fails_and_serializes_strictly(self):
        result = self.report([sample(0., (-1e308, 0., 0.)), sample(1., (1e308, 0., 0.))])
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['metrics']['actual_acceleration_mps2']['nonfinite_derived_count'], 1)

    def fixture_manifest(self, case='jitter'):
        frozen = {}
        protocol = dict(max_speed_mps=2., max_vertical_speed_mps=1., plant_accel_mps2=1.2, plant_tau_s=.22)
        contents = {'protocol': json.dumps(protocol), 'motion_helper': '# inert synthetic source\n',
                    'simulator': '\n'.join(f'v{i} = scalar_from_env({key!r}, {value!r})' for i, (key, value) in enumerate(audit.DEFAULTS.items()))}
        for name, suffix in audit.SOURCE_SUFFIXES.items():
            path = self.root / suffix; path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents[name]); frozen[str(path)] = audit.digest(path.read_bytes())
        self.write_csv([sample(0.), sample(1.)])
        runtime_path = self.root / 'run.parameters.json'
        runtime = dict(executable='/frozen/controller', executable_sha256='controller-sha', launch_sha256='launch-sha',
                       common_source_sha256={suffix: frozen[str(self.root / suffix)] for suffix in audit.SOURCE_SUFFIXES.values()})
        runtime_path.write_text(json.dumps(runtime))
        env = dict(FOV_GVF_RUNTIME_MANIFEST=str(runtime_path), FOV_GVF_RUN_ID='synthetic-run',
                   ISAAC_GOAL_PROTOCOL=str(self.root / audit.SOURCE_SUFFIXES['protocol']))
        meta = dict(case=case, variant='baseline', repeat=1, csv=str(self.csv), experiment_environment=env,
                    runtime_binary_verified=True, runtime_launch_verified=True, runtime_common_sources_verified=True,
                    controller_executable='/frozen/controller', controller_sha256='controller-sha', launch_sha256='launch-sha',
                    shared_simulator_sha256=frozen[str(self.root / audit.SOURCE_SUFFIXES['simulator'])])
        if case == 'goal':
            meta['result_path'] = str(self.root / 'run.json')
        manifest = dict(complete=True, file_sha256=frozen, binary_sha256={'baseline': 'controller-sha'},
                        order_by_repeat=[['baseline']], runs=[meta])
        path = self.root / 'manifest.json'; path.write_text(json.dumps(manifest))
        return path, manifest, runtime_path, runtime

    def audit_fixture(self, path, manifest):
        path.write_text(json.dumps(manifest))
        result = audit.audit_manifests([path]); json.dumps(result, allow_nan=False)
        return result

    def test_manual_and_goal_provenance_pass(self):
        for case in ('jitter', 'goal'):
            with self.subTest(case=case):
                path, manifest, _, _ = self.fixture_manifest(case)
                result = self.audit_fixture(path, manifest)
                self.assertEqual(result['status'], 'PASS', result)
                self.assertEqual(result['expected_run_count'], 1)
                self.assertEqual(result['audited_run_count'], 1)
                self.assertEqual(result['manifests'][0]['sha256'], audit.digest(path.read_bytes()))

    def test_incomplete_inventory_and_duplicate_manifest_fail(self):
        path, manifest, _, _ = self.fixture_manifest()
        manifest['complete'] = False
        self.assertEqual(self.audit_fixture(path, manifest)['status'], 'FAIL')
        manifest['complete'] = True; manifest['order_by_repeat'] = [['baseline', 'v2']]
        self.assertEqual(self.audit_fixture(path, manifest)['status'], 'FAIL')
        path, manifest, _, _ = self.fixture_manifest()
        self.assertEqual(audit.audit_manifests([path, path])['status'], 'FAIL')
        self.assertEqual(audit.audit_manifests([self.root / 'missing.json'])['status'], 'FAIL')

    def test_missing_or_changed_source_fails(self):
        for mutation in ('missing', 'changed', 'unfrozen', 'invalid-protocol', 'changed-default'):
            with self.subTest(mutation=mutation):
                path, manifest, _, _ = self.fixture_manifest()
                source = self.root / audit.SOURCE_SUFFIXES['simulator']
                if mutation == 'missing': source.unlink()
                elif mutation == 'changed': source.write_text(source.read_text() + '\n# changed')
                elif mutation == 'unfrozen': manifest['file_sha256'].pop(str(source))
                elif mutation == 'changed-default':
                    source.write_text(source.read_text().replace("'ISAAC_PLANT_ACCEL', 1.2", "'ISAAC_PLANT_ACCEL', 99."))
                    manifest['file_sha256'][str(source)] = audit.digest(source.read_bytes())
                else:
                    source = self.root / audit.SOURCE_SUFFIXES['protocol']; source.write_text('[]')
                    manifest['file_sha256'][str(source)] = audit.digest(source.read_bytes())
                self.assertEqual(self.audit_fixture(path, manifest)['status'], 'FAIL')

    def test_runtime_and_physics_evidence_fail_closed(self):
        for mutation in ('missing-runtime', 'controller-sha', 'controller-path', 'launch-sha', 'false-driver',
                         'physics-override', 'frozen-controller', 'shared-simulator', 'missing-common', 'protocol-path'):
            with self.subTest(mutation=mutation):
                path, manifest, runtime_path, runtime = self.fixture_manifest('goal')
                meta = manifest['runs'][0]
                if mutation == 'missing-runtime': runtime_path.unlink()
                elif mutation == 'controller-sha': runtime['executable_sha256'] = 'wrong'
                elif mutation == 'controller-path': runtime['executable'] = '/wrong'
                elif mutation == 'launch-sha': runtime['launch_sha256'] = 'wrong'
                elif mutation == 'false-driver': meta['runtime_common_sources_verified'] = False
                elif mutation == 'physics-override': meta['experiment_environment']['ISAAC_PLANT_ACCEL'] = '2'
                elif mutation == 'frozen-controller': manifest['binary_sha256']['baseline'] = 'wrong'
                elif mutation == 'shared-simulator': meta['shared_simulator_sha256'] = 'wrong'
                elif mutation == 'missing-common': runtime['common_source_sha256'] = {}
                elif mutation == 'protocol-path': meta['experiment_environment']['ISAAC_GOAL_PROTOCOL'] = '/wrong'
                if mutation != 'missing-runtime': runtime_path.write_text(json.dumps(runtime))
                self.assertEqual(self.audit_fixture(path, manifest)['status'], 'FAIL')

    def test_cli_status_and_refuses_overwrite(self):
        path, _, _, _ = self.fixture_manifest()
        output = self.root / 'out.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(audit.main(['--manifest', str(path), '--output', str(output)]), 0)
        original = output.read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            audit.main(['--manifest', str(path), '--output', str(output)])
        self.assertEqual(raised.exception.code, 2); self.assertEqual(output.read_bytes(), original)
        self.write_csv([sample(0.), sample(1., (1.5, 0., 0.))])
        output2 = self.root / 'failed.json'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(audit.main(['--manifest', str(path), '--output', str(output2)]), 1)
        self.assertEqual(json.loads(output2.read_text())['status'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
