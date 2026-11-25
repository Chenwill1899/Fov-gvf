import socket,struct,subprocess,time,os,json,fcntl,shutil,math,hashlib
from pathlib import Path
from datetime import datetime
root=Path(__file__).resolve().parents[3];exp=root/'docs/performance/isaac_closed_loop_20261117'
headless=os.environ.get('P3_SMOKE_HEADLESS','1')=='1'
out=exp/('adapter_smoke_'+('headless_' if headless else 'gui_')+datetime.now().strftime('%Y%m%d_%H%M%S_%f'));out.mkdir()
socket_dir=Path('/tmp')/('p3_adapter_smoke_'+str(os.getpid()));socket_dir.mkdir(mode=0o700);sockpath=socket_dir/'ipc.sock'
lock=open('/tmp/fov_gvf_user_navigation.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(sockpath));server.listen(1);server.settimeout(1)
cmd=[str(root.parent/'isaacsim/python.sh'),str(root/'scripts/isaac/run_closed_loop_simulation.py'),'--scenario',str(root/'src/new_controller/config/isaac_closed_loop/straight.json'),'--output',str(out/'simulation'),'--socket',str(sockpath),'--steps','3','--time-scale','1']
if headless:cmd.append('--headless')
cmd=['/usr/bin/setpriv','--no-new-privs','--']+cmd
cache=exp/'shared_cache';environment=os.environ.copy()
for key,name in [('XDG_CACHE_HOME','cache'),('CUDA_CACHE_PATH','cuda'),('__GL_SHADER_DISK_CACHE_PATH','shader'),('WARP_CACHE_PATH','warp')]:
 (cache/name).mkdir(parents=True,exist_ok=True);environment[key]=str(cache/name)
environment['EGO4P3_SIM_SHARED_CACHE']=str(cache)
log=(out/'simulation.log').open('w');child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=environment)
receipt={'purpose':'Adapter/GPU/IPC smoke only. All-zero stub commands, no EGO4P3 algorithm execution or flight claim.','command':cmd,'headless':headless,'requests':[], 'source_sha256':{name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in ['scripts/isaac/run_closed_loop_simulation.py','scripts/isaac/closed_loop_scene.py']}}
print('SMOKE_DIR='+str(out),flush=True)
connection=None
try:
 deadline=time.monotonic()+900
 while connection is None and time.monotonic()<deadline:
  if child.poll() is not None:raise RuntimeError('simulator exited before socket: '+str(child.returncode))
  try:connection,_=server.accept()
  except socket.timeout:pass
 if connection is None:raise TimeoutError('adapter startup timeout')
 ready=json.loads((out/'simulation/sensors_ready.json').read_text());pid=int(ready['pid']);threads=[]
 for task in (Path('/proc')/str(pid)/'task').iterdir():
  try:
   tid=int(task.name);status=dict(line.split(':',1) for line in (task/'status').read_text().splitlines() if ':' in line)
   threads.append(dict(tid=tid,name=(task/'comm').read_text().strip(),policy=os.sched_getscheduler(tid),priority=os.sched_getparam(tid).sched_priority,uids=list(map(int,status['Uid'].split())),cap_eff=int(status['CapEff'].strip(),16),cap_prm=int(status['CapPrm'].strip(),16),no_new_privs=int(status['NoNewPrivs'].strip())))
  except (FileNotFoundError,ProcessLookupError):pass
 receipt['simulator_threads']=threads
 connection.settimeout(30)
 def read(n):
  data=bytearray()
  while len(data)<n:
   part=connection.recv(n-len(data))
   if not part:raise EOFError('early EOF')
   data.extend(part)
  return data
 for _ in range(3):
  vals=struct.unpack('<6Q15d',read(168));magic,version,seq,ns,w,h=vals[:6];assert magic==0x45474F3450330001 and version==1
  read(4*w*h*4)
  distance=math.sqrt((vals[6]-16)**2+vals[7]**2+(vals[8]-4)**2)
  reply=struct.pack('<6Q12d',magic,1,seq,ns,0,0,*([0.]*7+[distance]+[0.]*4))
  connection.sendall(reply);receipt['requests'].append({'seq':seq,'simulation_ns':ns,'width':w,'height':h})
 receipt['exit_code']=child.wait(timeout=60)
 receipt['passed']=receipt['exit_code']==0 and len(receipt['requests'])==3
except Exception as exc:
 receipt['passed']=False;receipt['error']=repr(exc)
 try:child.wait(timeout=30)
 except subprocess.TimeoutExpired:pass
finally:
 if child.poll() is None:
  import signal
  os.killpg(child.pid,signal.SIGTERM)
  try:child.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 receipt['exit_code']=child.returncode
 if connection:connection.close()
 server.close();sockpath.unlink(missing_ok=True);socket_dir.rmdir();log.close()
 (out/'smoke_execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({key:value for key,value in receipt.items() if key!='simulator_threads'}),flush=True)
raise SystemExit(0 if receipt['passed'] else 1)
