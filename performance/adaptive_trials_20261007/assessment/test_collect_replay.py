"""Replay adapter tests use tiny text fixtures; they execute no replay binary."""
import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parent))
import collect_replay as c
import assess_replay


def put(p,x):p.write_text(json.dumps(x))


def output():
    return dict(status='SAFE',reason='READY',accepted=True,free_directions=8,refined=False,command_changed=False,reset_reason='',continued=False,
        command=[.1,0.,0.],required_prefix=.1,best_prefix=2.,intent_change_angle=0.)


@pytest.fixture
def fixture(tmp_path):
    project=tmp_path/'project';project.mkdir()
    frames=[]
    for i in range(2):
        name=f'frame{i}.bin';(project/name).write_bytes(bytes([i]));frames.append(dict(group='same',file=name))
    source=tmp_path/'candidate.cpp';source.write_text('candidate source')
    baseline=tmp_path/'source_sha256.json';baseline.write_text('{}')
    binaries={}
    for side in ['baseline','candidate']:
        p=tmp_path/side;p.write_bytes(side.encode());binaries[side]=str(p)
    phases=[]
    for phase in ['explore','confirm']:
        rows=[]
        for repeat in range(5):
            for frame in frames:
                order=['baseline','candidate'] if (repeat+(phase=='confirm'))%2==0 else ['candidate','baseline']
                for side in order:
                    rows.append(dict(repeat=repeat,variant=side,**frame,wall_ms=10 if side=='baseline' else 8,cpu_ms=5,returncode=0,stdout=json.dumps(output()),stderr=''))
                rows[-1]['pair_exact_output_equal']=True
        data=dict(complete=True,frames=frames,frame_sha256={f['file']:c.sha(project/f['file']) for f in frames},
            executables=binaries,sha256={s:c.sha(p) for s,p in binaries.items()},environment={'OMP_WAIT_POLICY':'PASSIVE'},rows=rows,
            started_unix=1 if phase=='explore' else 3,ended_unix=2 if phase=='explore' else 4)
        p=tmp_path/(phase+'.json');put(p,data);phases.append(p)
    return dict(project=project,source=source,baseline=baseline,explore=phases[0],confirm=phases[1])


def adapt(f,confirm=False):
    return c.collect(f['explore'],f['confirm'] if confirm else None,'test',f['project'],f['baseline'],f['source'],expected_frames=2)


def test_batch_sample_count_not_number_of_frames(fixture):
    rows,plan,audit=adapt(fixture)
    assert len(rows['before'])==len(rows['after'])==5
    assert all(r['decision_count']==2 and r['numeric_checks_passed'] for r in rows['after'])
    assert audit['phases'][0]['pooled']['all']['baseline']['wall_ms']['n']==10
    assert rows['before'][0]['first_execution_index']==1
    assert rows['after'][0]['first_execution_index']==2
    assert rows['before'][0]['last_execution_index']==3
    assert rows['after'][0]['last_execution_index']==4


def test_retrospective_mapping_never_passes_even_with_gain(fixture):
    rows,plan,_=adapt(fixture)
    r=assess_replay.evaluate({v['run_id']:v for v in rows['before']},{v['run_id']:v for v in rows['after']},plan,stage='explore')
    assert r['decision']=='INSUFFICIENT'
    assert plan['retrospective_mapping']


def test_real_exact_stdout_mismatch_not_overridden_by_attestation(fixture):
    x=c.load(fixture['explore']);modified=output();modified['command'][0]=.1000000000001;x['rows'][1]['stdout']=json.dumps(modified);put(fixture['explore'],x)
    rows,_,_=adapt(fixture)
    assert not rows['after'][0]['numeric_checks_passed']
    assert any('stdout mismatch' in e for e in rows['after'][0]['errors'])


def test_nonfinite_equal_outputs_fail_numeric_check(fixture):
    x=c.load(fixture['explore']);modified=output();modified['command'][0]=float('nan')
    for r in x['rows'][:2]:r['stdout']=json.dumps(modified)
    put(fixture['explore'],x);rows,_,_=adapt(fixture)
    assert not rows['after'][0]['numeric_checks_passed']


def test_missing_fixture_cannot_reduce_population_to_speed_up(fixture):
    x=c.load(fixture['explore']);x['rows']=x['rows'][:-2];put(fixture['explore'],x)
    rows,_,_=adapt(fixture)
    assert len(rows['after'])==5
    assert not rows['after'][-1]['numeric_checks_passed']


def test_changed_binary_or_fixture_marks_not_valid(fixture):
    (fixture['project']/'frame1.bin').write_bytes(b'changed')
    rows,_,audit=adapt(fixture)
    assert audit['phases'][0]['problems']
    assert all(not r['numeric_checks_passed'] for r in rows['after'])


def test_confirmation_sequence_and_inverted_order_preserved(fixture):
    rows,plan,_=adapt(fixture,True)
    assert len(rows['before'])==10
    pair=plan['pairs'][5];assert pair['order']==['after','before']
    assert rows['after'][5]['sequence']<rows['before'][5]['sequence']
    assert min(r['sequence'] for r in rows['after'][5:])>max(r['sequence'] for r in rows['before'][:5])


def test_overlapping_confirmation_not_independent(fixture):
    x=c.load(fixture['confirm']);x['started_unix']=1.5;put(fixture['confirm'],x)
    rows,_,_=adapt(fixture,True)
    assert all(not r['numeric_checks_passed'] for r in rows['after'])


def test_changed_discrete_decision_changes_digest(fixture):
    x=c.load(fixture['explore']);modified=output();modified['accepted']=False;x['rows'][1]['stdout']=json.dumps(modified);put(fixture['explore'],x)
    rows,_,_=adapt(fixture)
    assert rows['before'][0]['decisions_sha256']!=rows['after'][0]['decisions_sha256']
