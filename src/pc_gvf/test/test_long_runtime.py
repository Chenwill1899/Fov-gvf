from pathlib import Path
import sys
import pytest
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts/isaac'))
from manual_control_math import BoundedTimingSamples
from navigation_benchmark import SequenceGoalTrial


def protocol():
    return dict(goal=[0,0,10],start=[0,0,10],waypoints=[[300,0,10],[0,0,10]],warmup_s=2.,timeout_s=600.,max_speed_mps=10.,max_vertical_speed_mps=1.,input_deadband_m=.5,arrival_radius_m=1.,arrival_speed_mps=.1,arrival_hold_s=.5,leg_timeout_s=180.,between_leg_release_s=1.)


def test_million_timing_samples_do_not_grow_resident_window():
    s=BoundedTimingSamples()
    for i in range(1000000):s.append(i)
    assert s.count==1000000 and len(s.values)==8192
    assert s.summary()[0]==499999.5 and s.summary()[3]==999999


def test_continuous_legs_keep_hold_release_and_reverse_intent():
    t=SequenceGoalTrial(protocol(),60.)
    assert t.intent([0,0,10],2.)==(10.,0.,0.)
    assert t.update([300,0,10],[0,0,0],40.,100.,False)=='RUNNING'
    assert not t.completed_legs
    assert t.update([300,0,10],[0,0,0],40.5,100.5,False)=='RUNNING'
    assert len(t.completed_legs)==1 and t.intent([300,0,10],41.)==(0.,0.,0.)
    assert t.intent([300,0,10],41.5)==(-10.,0.,0.)
    assert t.update([0,0,10],[0,0,0],80.,140.,False)=='RUNNING'
    assert t.update([0,0,10],[0,0,0],80.5,140.5,False)=='ARRIVED'
    assert len(t.completed_legs)==2


def test_continuous_failure_cannot_count_as_completion_or_skip_guard_in_pause():
    t=SequenceGoalTrial(protocol(),60.)
    assert t.update([0,0,10],[0,0,0],1.,100.,True)=='COLLISION_GUARD'
    assert t.update([0,0,10],[0,0,0],182.,281.,False)=='LEG_TIMEOUT'
    assert not t.completed_legs


def test_continuous_final_goal_must_match_protocol():
    p=protocol();p['goal']=[1,0,10]
    with pytest.raises(ValueError):SequenceGoalTrial(p)


def test_manual_launcher_uses_validated_child_affinity_only(monkeypatch):
    import ast, os
    source=ROOT/'src/pc_gvf/launch/isaac_cloud_navigation.launch.py'
    tree=ast.parse(source.read_text())
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='controller_cpu_prefix')
    scope={'os':os};exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),scope)
    monkeypatch.setattr(os,'sched_getaffinity',lambda pid:set(range(16)))
    monkeypatch.setenv('EGO5090_CONTROLLER_CPU_AFFINITY','0,1,7,15')
    assert scope['controller_cpu_prefix']()==['taskset -c 0,1,7,15']
    for invalid in ['0,0','99999','0; echo bad']:
        monkeypatch.setenv('EGO5090_CONTROLLER_CPU_AFFINITY',invalid)
        with pytest.raises(ValueError):scope['controller_cpu_prefix']()
