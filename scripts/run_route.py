#!/usr/bin/env python3
"""Use the shared evidence/audit driver with a frozen 300m mid-height route."""
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/pc_gvf/tools'))
import sim4_benchmark as driver

route_id = os.environ.get('EGO2P6_GOAL_ROUTE', 'route300').lower()
if route_id == 'route300':
    route_path = ROOT / 'src/pc_gvf/config/route300.json'
    route = json.loads(route_path.read_text())
else:
    route_path = (ROOT / 'src/pc_gvf/config/routes.json')
    routes = json.loads(route_path.read_text())['routes']
    matches = [item for item in routes if item['id'].lower() == route_id]
    if len(matches) != 1:
        raise ValueError('EGO2P6_GOAL_ROUTE must be route300 or one of: ' +
                         ', '.join(item['id'] for item in routes))
    route = matches[0]
original_protocol = driver.protocol_for
def protocol_for(args, map_name):
    protocol = original_protocol(args, map_name)
    protocol.update(start=route['start'], goal=route['goal'])
    # Match candidate24's declared body bounds in the independent audit.
    protocol['working_bounds'] = dict(
        enabled=os.environ.get('FOV_GVF_WORKING_BOUNDS_ENABLED', '1') == '1',
        min_x=float(os.environ.get('FOV_GVF_WORKING_MIN_X', '-80')),
        max_x=float(os.environ.get('FOV_GVF_WORKING_MAX_X', '399.8')),
        min_y=float(os.environ.get('FOV_GVF_WORKING_MIN_Y', '-60')),
        max_y=float(os.environ.get('FOV_GVF_WORKING_MAX_Y', '59.8')),
        min_z=float(os.environ.get('FOV_GVF_FREE_SPACE_FLOOR_Z', '0')),
        max_z=float(os.environ.get('FOV_GVF_FREE_SPACE_CEILING_Z', '39.8')))
    return protocol
driver.protocol_for = protocol_for

if __name__ == '__main__':
    raise SystemExit('EGO4P0路线控制尚待重写。离线协议/审计函数保留，未运行旧导航。')
