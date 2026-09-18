#!/usr/bin/env python3
"""Export the currently opened Blender cloud scene to a Z-up, metre-scale USD."""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import bpy

raw = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args(raw)
output = args.output.expanduser().resolve()
output.parent.mkdir(parents=True, exist_ok=True)
result = bpy.ops.wm.usd_export(
    filepath=str(output), selected_objects_only=False, export_animation=False,
    export_materials=True, export_normals=True, export_cameras=True,
    export_lights=True, export_custom_properties=True, root_prim_path="/World",
    # Blender and Isaac Sim both use a Z-up, right-handed metre world here.
    # Do not apply Blender's optional forward-axis conversion: it would mirror
    # the authored X start/goal direction on USD export.
    convert_orientation=False, export_global_forward_selection="NEGATIVE_Y",
    export_global_up_selection="Z", convert_scene_units="METERS", meters_per_unit=1.0,
)
if "FINISHED" not in result:
    raise RuntimeError(f"USD export failed: {result}")
print(f"[BLENDER USD] Exported {output}", flush=True)
