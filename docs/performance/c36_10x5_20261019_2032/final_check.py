#!/usr/bin/env python3
from pathlib import Path
import json,csv,hashlib,collections,datetime
R=Path('/home/ubuntu5090/dhm/EGO2P6');T=R/'performance/c36_10x5_20261019_2032'
s=json.loads((T/'completion_summary.json').read_text());rows=json.loads((T/'records.json').read_text());ex=json.loads((T/'exclusions.json').read_text()) if (T/'exclusions.json').exists() else {};identity=json.loads((T/'frozen_identity.json').read_text())['files'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert s['complete'] and s['formal_flights']==50
assert all(g['formal']==5 for g in s['groups'].values())
assert len({x['label'] for x in rows})==len(rows)
assert set(ex)<={x['label'] for x in rows}
assert all(sha(R/n)==h for n,h in identity.items()),'Active source/install/map changed'
routes={r['id']:r for r in json.loads((R/'performance/sync_candidate24_20261019_1240/source_5090_comparison/routes.json').read_text())['routes']};proof=[]
for row in rows:
 if not row['has_trajectory']:continue
 p=R/row['result'];a=json.loads(p.with_suffix('.audit.json').read_text());m=json.loads(p.with_suffix('.manifest.json').read_text());protocol=json.loads(p.with_suffix('.protocol.json').read_text())
 assert a==row['audit'],row['label']
 assert protocol['start']==routes[row['route']]['start'] and protocol['goal']==routes[row['route']]['goal']
 assert protocol['max_speed_mps']==10 and protocol['max_vertical_speed_mps']==1 and protocol['pipeline_delay_s']==.20 and protocol['plant_accel_mps2']==16 and protocol['plant_tau_s']==.10
 assert m['complete'] and m['execution_matches_frozen'] and not m['changed_files_during_run'],row['label']
 checks=a.get('additional_checks',{})
 for k in ['protocol_matches_prepared','starts_at_protocol','frame_cadence','integration_matches_velocity','result_matches_trace','frozen_execution_matches','sensor_record_valid','sensor_request_matches','sensor_frozen_matches','camera_extrinsics_valid']:
  assert checks[k],(row['label'],k)
 proof.append(dict(label=row['label'],route=row['route'],status=row['status'],safe_arrival=row['safe_arrival'],excluded=row['label'] in ex,audit_sha256=sha(p.with_suffix('.audit.json')),trace_sha256=sha(p.with_suffix('.csv')),protocol_sha256=sha(p.with_suffix('.protocol.json'))))
assert s['all_samples_complete'] and s['all_frozen_unchanged']
raw=[r for r in rows if r['has_trajectory']];assert len(raw)==50+len(ex)
result=dict(passed=True,verified_at=datetime.datetime.now().astimezone().isoformat(),formal_flights=50,formal_safe_arrivals=s['formal_safe_arrivals'],formal_failed=s['formal_failed'],raw_flights=len(raw),raw_attempts=len(rows),startup_attempts=s['startup_attempts'],excluded=len(ex),source_install_map_files_unchanged=len(identity),all_original_audits_preserved=True,all_samples_complete=s['all_samples_complete'],proof=proof)
(T/'final_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='proof'},ensure_ascii=False))
