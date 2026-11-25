import csv,hashlib,importlib.util,json,shutil,tempfile
from pathlib import Path
root=Path(__file__).resolve().parents[3];experiment=Path(__file__).resolve().parent
source=root/'scripts/audit_isaac_closed_loop.py';spec=importlib.util.spec_from_file_location('flight_audit',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
baseline=experiment/'run_20261117_224531_657435'
inputs=['scenario.json','result.json','trajectory.csv','base_step1.json','controller.csv','simulation/sensor_frames.csv','simulation/sensor_samples.csv','simulation/simulation.sensors.json','simulation/overlay_depth_check.json']
original={name:hashlib.sha256((baseline/name).read_bytes()).hexdigest() for name in inputs};results=[]
with tempfile.TemporaryDirectory(prefix='p3_audit_guards_',dir='/tmp') as directory:
 folder=Path(directory)
 for name in inputs:
  (folder/name).parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(baseline/name,folder/name)
 result=module.audit(folder);assert result['passed'];results.append({'case':'valid_actual_straight','passed':True})
 def edit_csv(name,key,value):
  path=folder/name;rows=list(csv.DictReader(path.open()));fields=list(rows[0]);rows[0][key]=value
  with path.open('w',newline='') as stream:
   writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
 edit_csv('controller.csv','ux','nan');result=module.audit(folder);assert not result['passed'] and 'controller_finite_records' in result['failed'];results.append({'case':'NaN_controller_command','rejected':True,'failed':result['failed']});shutil.copyfile(baseline/'controller.csv',folder/'controller.csv')
 edit_csv('simulation/sensor_frames.csv','camera','UnrecognizedCamera');result=module.audit(folder);assert not result['passed'] and 'four_rendered_camera_frames_each_step' in result['failed'];results.append({'case':'wrong_camera_identity','rejected':True,'failed':result['failed']});shutil.copyfile(baseline/'simulation/sensor_frames.csv',folder/'simulation/sensor_frames.csv')
 path=folder/'simulation/overlay_depth_check.json';data=json.loads(path.read_text());data['checks'][0]['comparisons']=[];path.write_text(json.dumps(data));result=module.audit(folder);assert not result['passed'] and 'overlay_depth_isolation' in result['failed'];results.append({'case':'empty_overlay_comparisons','rejected':True,'failed':result['failed']})
assert original=={name:hashlib.sha256((baseline/name).read_bytes()).hexdigest() for name in inputs}
report={'passed':True,'auditor_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'cases':results,'baseline_inputs_unchanged':True,'baseline':str(baseline)}
(experiment/'audit_guard_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
for name in ['run_20261117_224531_657435','run_20261117_224648_909899','run_20261117_225039_833717','run_20261117_225236_259438']:
 folder=experiment/name;result=module.audit(folder);result['auditor_sha256']=report['auditor_sha256'];result['note']='Read-only strengthened audit; original run audits and execution inputs remain unchanged.';(folder/'independent_flight_audit_strengthened.json').write_text(json.dumps(result,indent=2)+'\n');assert result['passed'],(name,result['failed']);print(name,'strengthened PASS')
