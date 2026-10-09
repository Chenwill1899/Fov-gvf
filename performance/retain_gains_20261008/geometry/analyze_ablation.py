#!/usr/bin/env python3
"""Paired complete-batch uncertainty estimates; all metrics and groups retained."""
from pathlib import Path
import hashlib,json,math,random,statistics
stage=Path(__file__).resolve().parent
raw=stage/'replay_ablation.json';d=json.loads(raw.read_text());assert d['complete']
plan=json.loads((stage/'plan.json').read_text());n=len(plan['orders']);batch=d['batch_summary']
assert n==8
comparisons=plan['required_incremental_comparisons']+[plan['additional_descriptive_comparison']]
def percentile(a,q):
    a=sorted(a);u=(len(a)-1)*q;i=int(u);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(u-i)
rng=random.Random(20261008);resamples=[[rng.randrange(n) for _ in range(n)] for _ in range(20000)]
def pair_stats(before,after):
    reduction=[1-b/a if a else 0. for a,b in zip(before,after)]
    improve=sum(x>0 for x in reduction);worsen=sum(x<0 for x in reduction);non_ties=improve+worsen
    effect=1-statistics.mean(after)/statistics.mean(before) if statistics.mean(before) else 0.
    draws=[1-sum(after[i] for i in index)/sum(before[i] for i in index) if sum(before[i] for i in index) else 0. for index in resamples]
    p=lambda k:sum(math.comb(non_ties,i) for i in range(k,non_ties+1))/2**non_ties if non_ties else 1.
    return dict(before=before,after=after,before_mean=statistics.mean(before),after_mean=statistics.mean(after),relative_reduction=effect,bootstrap_95pct_ci=[percentile(draws,.025),percentile(draws,.975)],improved_batches=improve,worsened_batches=worsen,tied_batches=n-non_ties,sign_p_benefit=p(improve),sign_p_harm=p(worsen))
out=dict(raw_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),plan_sha256=d['plan_sha256'],independent_units=n,ci_method=plan['confidence'],sign_method=plan['direction'],comparisons={})
for before,after in comparisons:
    key=before+'_to_'+after;out['comparisons'][key]={}
    for group in ['all','stall16','inherited21']:
        out['comparisons'][key][group]={metric:{stat:pair_stats([batch[str(i)][group][before][metric][stat] for i in range(n)],[batch[str(i)][group][after][metric][stat] for i in range(n)]) for stat in ['mean','p50','p95','max']} for metric in ['wall_ms','cpu_ms']}
path=stage/'paired_analysis.json'
if path.exists():raise RuntimeError('refusing overwrite')
path.write_text(json.dumps(out,indent=2)+'\n')
for comparison,groups in out['comparisons'].items():
    print(comparison)
    for group,metrics in groups.items():
        for stat in ['mean','p95','max']:
            r=metrics['wall_ms'][stat];print(group,stat,'reduction',round(100*r['relative_reduction'],3),'CI',*[round(100*x,3) for x in r['bootstrap_95pct_ci']],'direction',r['improved_batches'],r['worsened_batches'])
