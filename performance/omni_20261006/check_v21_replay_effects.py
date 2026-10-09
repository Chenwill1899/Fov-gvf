#!/usr/bin/env python3
"""Classify intentional decision changes after more complete union proofs."""
from pathlib import Path
import json,subprocess,math
out=Path(__file__).resolve().parent;root=out.parents[1]
exe=out/'v21_frozen/paper_replay'
old=json.loads((out/'inherited_replay_check.json').read_text())
files=sorted((root/'performance/stall_20261006_205620/replays').glob('frame_*.bin'))
assert len(files)==old['count']
files+=sorted((root/'performance/navigation_benchmark_20260929/omni_final_v17_ego1p3_3_replays').glob('frame_*.bin'))
rows=[]
for file in files:
 expected=json.loads(file.with_suffix('.json').read_text())
 r=subprocess.run([str(exe),str(file)],text=True,capture_output=True,timeout=60,check=True)
 actual=json.loads(r.stdout)
 keys=['status','reason','accepted','free_directions','refined']
 same=all(actual[k]==expected[k] for k in keys)
 same &= all(math.isclose(actual[k],expected[k],rel_tol=1e-10,abs_tol=1e-10) for k in ['best_prefix','required_prefix'])
 same &= all(math.isclose(x,y,rel_tol=1e-10,abs_tol=1e-10) for x,y in zip(actual['command'],expected['command']))
 finite=all(math.isfinite(v) for v in actual['command'])
 rows.append({'file':str(file),'same_decision':same,'newly_certified':not expected['accepted'] and actual['accepted'],'finite_command':finite,'old':expected,'new':actual})
report={'count':len(rows),'same_decisions':sum(r['same_decision'] for r in rows),'newly_certified':sum(r['newly_certified'] for r in rows),'all_finite':all(r['finite_command'] for r in rows),'scope':'Saved conservative depth and proof state only; native hits/frontend timing are not reproduced. Changed decisions are expected for deeper certified unions, not counted as exact replay matches.','frames':rows}
(out/'v21_replay_effects.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2))
