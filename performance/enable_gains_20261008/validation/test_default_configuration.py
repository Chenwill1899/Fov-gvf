from pathlib import Path
import pytest
from validate_default_configuration import FEATURES,cases,enabled_features,feature_checks,read_ros_parameters,write_new


def test_defaults_follow_requested_four_on_two_off():
    defaults=[c for c in cases() if c['label'].endswith('_defaults')]
    assert len(defaults)==2
    for case in defaults:
        assert case['expected']=={
            'depth_uncertainty_enabled':True,'paper_dynamic_obstacles':True,
            'paper_incremental_field':False,'paper_spherical_memory':True,
            'operator_assistance':False,'paper_shared_obstacles':True}
        assert case['environment']=={}


def test_all_routes_and_each_disable_are_covered_without_duplicates():
    matrix=cases();assert len(matrix)==16
    assert len({c['label'] for c in matrix})==16
    for fixed in (False,True):
        selected=[c for c in matrix if c['fixed_goal'] is fixed]
        assert len(selected)==8
        for name,(env,param,_) in FEATURES.items():
            case=next(c for c in selected if c['label'].endswith('_disable_'+name))
            assert case['environment']=={env:'0'}
            assert case['expected'][param] is False


def test_one_disable_does_not_disable_other_expected_on_features():
    case=next(c for c in cases() if c['label']=='manual_disable_memory')
    assert case['expected']['paper_spherical_memory'] is False
    assert case['expected']['depth_uncertainty_enabled'] is True
    assert case['expected']['paper_dynamic_obstacles'] is True
    assert case['expected']['paper_shared_obstacles'] is True


def test_expected_defaults_are_parameterized_for_later_decision():
    defaults=next(c for c in cases({'dynamic','shared'}) if c['label']=='goal_defaults')
    assert sum(defaults['expected'].values())==2
    assert defaults['expected']['depth_uncertainty_enabled'] is False
    assert defaults['expected']['paper_dynamic_obstacles'] is True


def test_empty_expected_on_list_can_check_old_configuration():
    assert not any(cases(enabled_features(''))[0]['expected'].values())


@pytest.mark.parametrize('value',['unknown','dynamic,dynamic','dynamic,','dynamic, shared'])
def test_invalid_cli_feature_list_rejected(value):
    with pytest.raises(ValueError):enabled_features(value)


@pytest.mark.parametrize('value',[1,0,'true','false',None])
def test_nonboolean_parameter_does_not_pass(value):
    assert not feature_checks({'flag':value},{'flag':True})['flag']
    assert not feature_checks({'flag':value},{'flag':False})['flag']


def test_missing_false_parameter_must_not_pass():
    assert not feature_checks({}, {'missing':False})['missing']


def test_changed_on_parameter_is_detected():
    expected=cases()[0]['expected'];actual=dict(expected);actual['paper_shared_obstacles']=False
    assert not all(feature_checks(actual,expected).values())


def test_humble_tuple_tag_is_read_as_inert_sequence(tmp_path):
    p=tmp_path/'parameters.yaml';p.write_text('/**:\n  ros__parameters:\n    enabled: true\n    camera_offset: !!python/tuple [0.22, 0, 0.02]\n')
    d=read_ros_parameters(p)
    assert d=={'enabled':True,'camera_offset':[.22,0,.02]}


def test_object_constructor_is_not_accepted(tmp_path):
    import yaml
    p=tmp_path/'bad.yaml';p.write_text('/**:\n  ros__parameters: !!python/object:builtins.object {}\n')
    with pytest.raises(yaml.constructor.ConstructorError):read_ros_parameters(p)


def test_result_refuses_to_overwrite_initial_failure(tmp_path):
    p=tmp_path/'result.json';write_new(p,{'passed':False})
    with pytest.raises(FileExistsError):write_new(p,{'passed':True})
    assert 'false' in p.read_text()


def source_fixture(tmp_path):
    import hashlib,json,tarfile
    project=tmp_path/'project';project.mkdir();freeze=tmp_path/'freeze';freeze.mkdir()
    (project/'node.cpp').write_text('code');(project/'WORK_LOG.md').write_text('record')
    sources={n:hashlib.sha256((project/n).read_bytes()).hexdigest() for n in ('node.cpp','WORK_LOG.md')}
    (freeze/'source_sha256.json').write_text(json.dumps(sources))
    with tarfile.open(freeze/'sources.tar.gz','w:gz') as archive:
        for name in sources:archive.add(project/name,arcname=name)
    return project,freeze


def test_frozen_sources_bind_runtime_but_allow_later_records(tmp_path):
    from validate_default_configuration import source_freeze_check
    project,freeze=source_fixture(tmp_path);(project/'WORK_LOG.md').write_text('new record')
    assert source_freeze_check(freeze,project)['verified']


def test_changed_runtime_source_cannot_pass_identity(tmp_path):
    from validate_default_configuration import source_freeze_check
    project,freeze=source_fixture(tmp_path);(project/'node.cpp').write_text('changed')
    d=source_freeze_check(freeze,project)
    assert not d['verified'] and d['current_runtime_mismatches']==['node.cpp']


def test_archive_manifest_disagreement_cannot_pass(tmp_path):
    import json
    from validate_default_configuration import source_freeze_check
    project,freeze=source_fixture(tmp_path)
    p=freeze/'source_sha256.json';d=json.loads(p.read_text());d['node.cpp']='0'*64;p.write_text(json.dumps(d))
    assert not source_freeze_check(freeze,project)['verified']
