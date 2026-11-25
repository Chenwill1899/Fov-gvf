from pathlib import Path
import hashlib,json,shlex,subprocess,zipfile
d=Path(__file__).resolve().parent
p=d.parents[2]
old=p/'docs/performance/r05_model_sync_20261117'
manifest=json.loads((p/json.loads((old/'latest_build.json').read_text())['manifest']).read_text())
def digest(x):return hashlib.sha256(x.read_bytes()).hexdigest()
assert all(digest(p/k)==v for k,v in manifest['source_identity'].items())
z=zipfile.ZipFile(d.parent/'r05_wp207_cruise_20261118/R05_wp207_evidence.zip')
def rep(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b)
s=z.read('native.cpp').decode()
s=rep(s,'    ++group;', '''    const auto t0=field_now_ns(); int64_t prepare=0, steering=0, curve=0, stop=0;
    ++group;''')
s=rep(s,'      CruiseRoute::Motion preview','      prepare=field_now_ns()-t0;\n      CruiseRoute::Motion preview')
s=rep(s,'        const auto next=trial_route.next(preview);','        const auto ta=field_now_ns();\n        const auto next=trial_route.next(preview);')
s=rep(s,'        if(!ob.input_ready','        const auto tb=field_now_ns();steering+=tb-ta;\n        if(!ob.input_ready')
s=rep(s,'        preview=next;','        curve+=field_now_ns()-tb;\n        preview=next;')
s=rep(s,'      if(!proof(first.v)){','      const auto ts=field_now_ns();\n      if(!proof(first.v)){')
s=rep(s,'      old_omega=first.w;','      stop=field_now_ns()-ts;\n      old_omega=first.w;')
s=rep(s,'command_z\\n','command_z,step_compute_ns,prepare7_ns,steer7_ns,curve7_ns,stop7_ns,pre_ns,pool_ns,cam_wall_ns,cam_cpu_ns,cam_project_ns,cam_compress_ns,cam_repair_ns,field_ns,spatial_ns,occupancy_ns\\n')
s=rep(s,'    log<<group<<','''    const auto elapsed=field_now_ns()-t0;
    const auto &depth=incoming->depth;const auto &oi=depth.observations;
    const int camera=std::max_element(trial_cam_wall.begin(),trial_cam_wall.end())-trial_cam_wall.begin();
    log<<group<<''')
s=rep(s,"<<old_command[2]<<'\\n';","""<<old_command[2]<<','<<elapsed<<','<<prepare<<','<<steering<<','<<curve<<','<<stop
       <<','<<plan.preprocessing_ns<<','<<oi.camera_work_ns<<','<<trial_cam_wall[camera]<<','<<trial_cam_cpu[camera]
       <<','<<oi.projection_ns[camera]<<','<<oi.compression_ns[camera]<<','<<depth.repair_ns[camera]
       <<','<<plan.field_ns<<','<<plan.spatial_ns<<','<<plan.occupancy_ns<<'\\n';""")
(d/'profile.cpp').write_text(s)
h=p/'src/new_controller/ldlt/include/new_controller/depth_observations.hpp'
s=h.read_text();s=rep(s,'class FrameObservationBuilder {','''inline std::array<int64_t,4> trial_cam_wall{},trial_cam_cpu{};
inline int64_t trial_cpu_now(){timespec t;clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return t.tv_sec*1000000000LL+t.tv_nsec;}
class FrameObservationBuilder {''')
s=rep(s,'bool repair) {\n    if(repair) {','bool repair) {\n    const auto tw=now(),tc=trial_cpu_now();\n    if(repair) {')
s=rep(s,'if(!fused_scans_) region_camera(camera,frame);','if(!fused_scans_) region_camera(camera,frame);\n    trial_cam_cpu[camera]=trial_cpu_now()-tc;trial_cam_wall[camera]=now()-tw;')
h2=d/'include/new_controller'/h.name;h2.parent.mkdir(parents=True,exist_ok=True);h2.write_text(s)
b=(p/manifest['binary']['path']).parent;cm=b/'CMakeFiles/r05_controller.dir'
flags=[shlex.split(l.split(' = ',1)[1]) for l in (cm/'flags.make').read_text().splitlines() if l.startswith('CXX_')]
subprocess.run(['/usr/bin/c++','-I'+str(d/'include'),*sum(flags,[]),'-c',str(d/'profile.cpp'),'-o',str(d/'profile.o')],check=True)
cmd=shlex.split((cm/'link.txt').read_text());cmd[cmd.index('CMakeFiles/r05_controller.dir/controller.cpp.o')]=str(d/'profile.o');cmd[cmd.index('-o')+1]=str(d/'trial-controller');subprocess.run(cmd,cwd=b,check=True)
# Standard simulator, same two local-path adaptations as the archived simulation.
s=(p/'src/r05_model/run_simulation.py').read_text()
s=rep(s,'project = Path(__file__).resolve().parents[2]',f'project = Path({str(p)!r})\nsys.path.insert(0,str(project/"src/r05_model"))')
(d/'trial_sim.py').write_text(s)
s=(p/'scripts/run_r05_model.py').read_text()
s=rep(s,'ROOT = Path(__file__).resolve().parents[1]',f'ROOT = Path({str(p)!r})')
s=rep(s,"ROOT / 'docs/performance/r05_model_sync_20261117'","Path(__file__).resolve().parent")
s=rep(s,"APP/'run_simulation.py'","EXPERIMENT/'trial_sim.py'")
s=rep(s,"if fifo and not root:", "if root:raise RuntimeError('experiment supervisor must be unprivileged')\n    if fifo and resource.getrlimit(resource.RLIMIT_RTPRIO)!=(0,5):")
s=rep(s,"else (0,0))\n        resource.setrlimit(resource.RLIMIT_CORE","else (0,0))\n        resource.setrlimit(resource.RLIMIT_CORE")
(d/'supervise.py').write_text(s)
manifest['source_identity'].update({str(f.relative_to(p)):digest(f) for f in (d/'prepare.py',d/'supervise.py',d/'trial_sim.py',d/'profile.cpp',h2)})
manifest['binary']={'path':str((d/'trial-controller').relative_to(p)),'sha256':digest(d/'trial-controller')}
(d/'manifest.json').write_text(json.dumps(manifest,indent=2));(d/'latest_build.json').write_text(json.dumps({'manifest':str((d/'manifest.json').relative_to(p))}))
