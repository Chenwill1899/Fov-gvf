"""Freeze final optional mechanisms and verify delivery without launching control."""
from pathlib import Path
import ast,hashlib,json,re,shutil,subprocess,tarfile
root=Path(__file__).resolve().parents[3];lab=root/'performance/six_axis_20261007';out=lab/'accepted_optional'
files=sorted(p for d in ('src','scripts','tools') for p in (root/d).rglob('*') if p.is_file() and not any(v in p.parts for v in ('__pycache__','.pytest_cache')))
py=[p for p in files if p.suffix=='.py'];sh=[p for p in files if p.suffix=='.sh']
for p in py:ast.parse(p.read_text(),filename=str(p))
for p in sh:subprocess.run(['bash','-n',str(p)],check=True)
diff=subprocess.run(['git','diff','--check'],cwd=root,text=True,capture_output=True)
(out/'diff_check.log').write_text(diff.stdout+diff.stderr)
assert diff.returncode==0,diff.stderr+diff.stdout
bad=[]
for name in ['SIX_AXIS_OPTIMIZATION.md','README.md','EGO1P5_VERSION.md']:
 for link in re.findall(r'\]\(([^)]+)\)',(root/name).read_text()):
  if '://' not in link and not link.startswith('#'):
   target=(root/link.split('#',1)[0])
   if not target.exists():bad.append(dict(file=name,target=link))
assert not bad,bad
checks=dict(python_ast=len(py),bash_syntax=len(sh),git_diff_check=diff.returncode,broken_doc_links=bad)
(out/'syntax_checks.json').write_text(json.dumps(checks,indent=2))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources={str(p.relative_to(root)):sha(p) for p in files}
(out/'source_sha256.json').write_text(json.dumps(sources,indent=2))
with tarfile.open(out/'sources.tar.gz','w:gz') as tar:
 for p in files:tar.add(p,arcname=str(p.relative_to(root)),recursive=False)
binaries={}
for name in ['depth_angular_controller','paper_replay']:
 p=Path('/tmp/fov_gvf_ego1p5_isaac_install/pc_gvf/lib/pc_gvf')/name
 shutil.copy2(p,out/name);binaries[name]=sha(out/name)
assert binaries['paper_replay']==json.loads((out/'inherited_replay.json').read_text())['current_sha256']
(out/'binary_sha256.json').write_text(json.dumps(binaries,indent=2))
logmap={}
for name in ['ros_control.json','ros_all_features.json']:
 data=json.loads((lab/'frontend_followup'/name).read_text())
 for old in data['logs']:
  src=Path(old);dst=out/'ros_raw'/src.name;dst.parent.mkdir(exist_ok=True)
  shutil.copy2(src,dst);logmap[old]=dict(saved=str(dst.relative_to(root)),sha256=sha(dst))
(out/'ros_log_archive.json').write_text(json.dumps(logmap,indent=2))
print(json.dumps(dict(checks=checks,frozen_source_files=len(files),binaries=binaries,archived_ros_logs=len(logmap)),indent=2))
