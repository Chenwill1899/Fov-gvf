#!/usr/bin/python3
import pathlib,json,hashlib,difflib,os
r=pathlib.Path(__file__).resolve().parent;s=json.load(open(r/'session.json'));b=pathlib.Path(s['base']);w=r/'work/EGO4P0';root=pathlib.Path('/home/starry/isaac-data')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
protected=json.load(open(r/'active_06_protected_sha256.json'));activebad=[n for n,h in protected.items() if not (root/n).is_file() or sha(root/n)!=h]
old=json.load(open(b/'frozen/source_sha256.json'));oldbad=[n for n,h in old.items() if sha(b/'work/EGO4P0'/n)!=h]
oldbin=json.load(open(b/'frozen/binary_sha256.json'));binbad=[n for n,h in oldbin.items() if sha(b/'bin'/n)!=h]
new=json.load(open(r/'frozen/source_sha256.json'));newbad=[n for n,h in new.items() if sha(w/'src/new_controller'/n)!=h]
(r/'protection_validation.json').write_text(json.dumps({'active_entries':len(protected),'active_mismatches':activebad,'old_source_entries':len(old),'old_source_mismatches':oldbad,'old_binary_entries':len(oldbin),'old_binary_mismatches':binbad,'candidate_source_entries':len(new),'candidate_source_mismatches':newbad},indent=2)+'\n')
assert not(activebad or oldbad or binbad or newbad)
paths=set(n for n in new)|set(str(pathlib.Path(n).relative_to('src/new_controller')) for n in old if n.startswith('src/new_controller/'))
changes=[];patch=[]
for n in sorted(paths):
 a=b/'work/EGO4P0/src/new_controller'/n;c=w/'src/new_controller'/n
 aa=a.read_text() if a.exists() else '';cc=c.read_text() if c.exists() else ''
 if aa!=cc:
  changes.append({'path':'src/new_controller/'+n,'old_sha256':sha(a) if a.exists() else None,'new_sha256':sha(c) if c.exists() else None})
  patch.extend(difflib.unified_diff(aa.splitlines(True),cc.splitlines(True),fromfile=('a/src/new_controller/'+n if a.exists() else '/dev/null'),tofile=('b/src/new_controller/'+n if c.exists() else '/dev/null')))
(r/'changed_files.json').write_text(json.dumps(changes,indent=2)+'\n');(r/'experimental.patch').write_text(''.join(patch))
print('protection PASS',len(protected),len(old),len(oldbin),len(new),'changed',len(changes))
