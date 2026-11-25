#!/usr/bin/env python3
"""Replace robot appearance in current maps without regenerating any obstacles."""
import hashlib
import json
from pathlib import Path
from pxr import Sdf,Usd
from robot_visual import ROOT,attach,LEGACY


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def signature(stage,rig=False):
    layer=Sdf.Layer.CreateAnonymous();layer.TransferContent(stage.GetRootLayer())
    edits=Sdf.BatchNamespaceEdit()
    if rig:
        world=layer.GetPrimAtPath('/World')
        for child in world.nameChildren.values():
            if child.name!='Robot':edits.Add(child.path,Sdf.Path.emptyPath)
        for name in LEGACY+['Visuals']:
            path=Sdf.Path('/World/Robot/'+name)
            if layer.GetPrimAtPath(path):edits.Add(path,Sdf.Path.emptyPath)
    else:edits.Add(Sdf.Path('/World/Robot'),Sdf.Path.emptyPath)
    if not layer.Apply(edits):raise ValueError('Could not isolate scene signature')
    return hashlib.sha256(layer.ExportToString().encode()).hexdigest()


def apply():
    records=[]
    for folder in ['ego_swarm_cloud','reference_lab','reference_hangar','reference_forest','reference_geo_forest','reference_geo_clutter','reference_geo_garage']:
        if not (ROOT/'scenes'/folder/'manifest.json').exists():continue
        directory=ROOT/'scenes'/folder;m=json.loads((directory/'manifest.json').read_text())
        path=directory/m['scene'];assert sha(path)==m['scene_sha256']
        grid=sha(directory/'occupancy.bin')
        s=Usd.Stage.Open(str(path));before=signature(s);rig=signature(s,True)
        old_hash=m['scene_sha256'];m['robot_visual']=attach(s)
        if 'height_stretch' in m:
            m['height_stretch'].update(ground_robot_and_sensors_unchanged=False,ground_and_sensor_rig_unchanged=True,robot_visual_replaced=True)
        assert signature(s)==before,'Non-robot scene changed'
        assert signature(s,True)==rig,'Camera/control rig changed'
        tmp=path.with_name(path.stem+'.robot-tmp.usdc');s.GetRootLayer().Export(str(tmp));tmp.replace(path)
        m['scene_sha256']=sha(path)
        if folder!='ego_swarm_cloud':m['robot_template_sha256']=sha(ROOT/'scenes/ego_swarm_cloud/ego_swarm_cloud_navigation.usd')
        assert sha(directory/'occupancy.bin')==grid==m['occupancy_sha256']
        temp=directory/'manifest.robot-tmp.json';temp.write_text(json.dumps(m,indent=None if folder=='reference_forest' else 2)+'\n');temp.replace(directory/'manifest.json')
        records.append(dict(map=folder,previous_scene_sha256=old_hash,scene_sha256=m['scene_sha256'],
            environment_signature=before,rig_signature=rig,occupancy_sha256=grid,
            environment_unchanged=True,camera_rig_unchanged=True,occupancy_unchanged=True))
        print(folder,'visual replaced; environment, camera rig and occupancy unchanged',flush=True)
    cloud=records[0]
    for filename in ['routes.json','route300.json']:
        p=ROOT/'src/pc_gvf/config'/filename;d=json.loads(p.read_text());d['scene_sha256']=cloud['scene_sha256']
        t=p.with_suffix('.tmp');t.write_text(json.dumps(d,indent=2)+'\n');t.replace(p)
    header='// Generated robot-appearance update. Exact 40m Cloud bytes.\n'
    header+='static const char* kHeight40CloudScene = "'+cloud['scene_sha256']+'";\n'
    header+='static const char* kHeight40CloudOccupancy = "'+cloud['occupancy_sha256']+'";\n'
    p=ROOT/'src/pc_gvf/config/generated_scene_identity/cloud_height_registry.inc';t=p.with_suffix('.tmp');t.write_text(header);t.replace(p)
    entries=[]
    for name in ['lab','hangar','forest','geo_forest','geo_clutter','geo_garage']:
        if not (ROOT/f'scenes/reference_{name}/manifest.json').exists():continue
        m=json.loads((ROOT/f'scenes/reference_{name}/manifest.json').read_text())
        entries.append('    {"%s", "%s", {%s}, {%s}, %.17g},' % (m['scene_sha256'],m['occupancy_sha256'],','.join(map(str,m['shape'])),','.join(map(str,m['origin'])),m['resolution']))
    header='''// Generated robot-appearance update. Exact-byte startup allowlist.
struct ReferenceSceneIdentity {
    const char* scene; const char* occupancy; int shape[3]; double origin[3]; double resolution;
};
static const ReferenceSceneIdentity kReferenceScenes[] = {
'''+ '\n'.join(entries)+'\n};\n'
    p=ROOT/'src/pc_gvf/config/generated_scene_identity/reference_scene_registry.inc';t=p.with_suffix('.tmp');t.write_text(header);t.replace(p)
    return records


if __name__=='__main__':
    import argparse
    cli=argparse.ArgumentParser();cli.add_argument('--report',type=Path,required=True);args=cli.parse_args()
    args.report.write_text(json.dumps(apply(),indent=2)+'\n')
