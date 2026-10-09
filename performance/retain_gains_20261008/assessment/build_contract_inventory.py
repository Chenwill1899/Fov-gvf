#!/usr/bin/env python3
"""Freeze reviewed selected cases from existing per-library and helper test evidence.

No tests are executed. Counts below come from individually reviewed unconditional
assertions, not the size of a passing suite. The helper is shared Python and its
source is checked against each controller's full frozen runtime source manifest.
"""
from pathlib import Path
import shutil
from common import load, sha, evidence, write_new
from collect_tasks import frozen_sources, contracts_check


def main():
    here=Path(__file__).resolve().parent;stage=here.parent
    destinations={"baseline":stage/"baseline",**{v:stage/"controllers"/v for v in ("v2","v4","combined")}}
    helper_source=stage/'combined_ctest_evidence/test_manual_control_math.xunit.xml'
    dest=here/'contract_evidence/manual_control_math_39.xunit.xml'
    if dest.exists():raise RuntimeError('refusing overwrite of evidence')
    dest.parent.mkdir(exist_ok=True);shutil.copy2(helper_source,dest)
    capability=stage/'capability_checks/summary.json'
    cases={
      'unknown_rejection':(2,{'src/pc_gvf/test/cpp/paper_guidance_check.cpp':['"unobserved near field accepted"','"unknown inside body footprint accepted"']},['paper_guidance_check']),
      'expired_depth_rejection':(2,{'src/pc_gvf/test/cpp/paper_guidance_check.cpp':['"stale observation accepted"'],'src/pc_gvf/test/cpp/observed_space_check.cpp':['"expired history authorized motion"']},['paper_guidance_check','observed_space_check']),
      'stale_command_rejection':(1,{'src/pc_gvf/test/test_manual_control_math.py':['def test_timed_sample_repeats_without_refreshing_receipt_or_sequence():','sample.select(10.101, True)[:2] == ((0., 0., 0.), 0)']},[])}
    out={'scope':'Selected source-reviewed negative cases, not exhaustive real-world fault coverage. Unknown/expired checks ran against each frozen core library. Shared Python receipt-expiry assertion passed once in the 39-case helper suite, and its runtime/test source is identical across the four frozen variants.','variants':{}}
    audits={}
    for variant,directory in destinations.items():
        freeze,contents=frozen_sources(directory)
        entry=dict(source_manifest_sha256=freeze['source_manifest_sha256'],controller_sha256=freeze['binary_sha256']['depth_angular_controller'],capability_summary=evidence(capability),checks={})
        for name,(count,sources,tests) in cases.items():
            passes=[]
            for test in tests:
                p=stage/'capability_checks'/variant/(test+'.json')
                passes.append({**evidence(p),'pattern':r'"test_returncode": 0'})
            if name=='stale_command_rejection':passes=[{**evidence(dest),'junit_case':'test_timed_sample_repeats_without_refreshing_receipt_or_sequence'}]
            assertions=[dict(file=file,sha256=freeze['source_sha256'][file],contains=tokens) for file,tokens in sources.items()]
            if name=='stale_command_rejection':
                for file in ('scripts/isaac/manual_control_math.py','scripts/isaac/run_fov_gvf_navigation.py'):
                    base,_=frozen_sources(destinations['baseline'])
                    if freeze['source_sha256'][file]!=base['source_sha256'][file]:raise RuntimeError('shared helper runtime changed')
            entry['checks'][name]=dict(cases=count,unsafe=0,passed=True,source_assertions=assertions,test_passes=passes)
        out['variants'][variant]=entry
        audits[variant]=contracts_check(out,variant,freeze,contents,here)
    out['verification']=audits
    if any(a['status']!='VERIFIED_SELECTED_CASES' for a in audits.values()):
        write_new(here/'contracts_review_failed.json',out);raise RuntimeError('contract verification failed; failure preserved')
    write_new(here/'contracts_reviewed.json',out)
    print('Four variants: unknown 2, expired depth/history 2, shared stale receipt 1 selected case each; all evidence verified.')


if __name__=='__main__':main()
