"""Session-only depth projection/mount overrides and actual USD readback.

No Isaac/ROS imports at module load. The renderer and CameraInfo use square
pixels, so the two aperture dimensions must match the render aspect ratio.
"""
from __future__ import annotations
import json
import math
import os
import sys
from pathlib import Path

ENV_NAME = 'ISAAC_DEPTH_HFOV_DEG'
OFFSET_ENV_NAME = 'ISAAC_DEPTH_CAMERA_OFFSET_M'
SENSOR_VERSION = 'ego2p1_runtime_sensors_v2'


def requested_hfov(environment=None):
    environment = os.environ if environment is None else environment
    if ENV_NAME not in environment:
        return None
    try:
        value = float(environment[ENV_NAME])
    except (ValueError, TypeError) as exc:
        raise ValueError(f'{ENV_NAME} must be a finite angle strictly between 0 and 180 degrees') from exc
    if not math.isfinite(value) or not 0. < value < 180.:
        raise ValueError(f'{ENV_NAME} must be a finite angle strictly between 0 and 180 degrees')
    return value


def requested_camera_offset(environment=None):
    environment = os.environ if environment is None else environment
    if OFFSET_ENV_NAME not in environment:
        return None
    try:
        value = float(environment[OFFSET_ENV_NAME])
    except (ValueError, TypeError) as exc:
        raise ValueError(f'{OFFSET_ENV_NAME} must be a finite positive distance in meters') from exc
    if not math.isfinite(value) or value <= 0.:
        raise ValueError(f'{OFFSET_ENV_NAME} must be a finite positive distance in meters')
    return value


def camera_extrinsic_record(stage, Usd, UsdGeom, camera_prim):
    """Read actual camera pose, converting USD camera axes to ROS optical axes."""
    robot = stage.GetPrimAtPath('/World/Robot')
    mount = camera_prim.GetParent()
    if not robot or not mount or mount.GetParent() != robot:
        raise ValueError('depth camera mount must be a direct child of /World/Robot')
    translation = mount.GetAttribute('xformOp:translate').Get()
    if translation is None or len(translation) != 3:
        raise ValueError('depth camera mount requires an explicit translation')
    time = Usd.TimeCode.Default()
    relative = (UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(time) *
                UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(time).GetInverse())
    local = UsdGeom.Xformable(camera_prim).GetLocalTransformation(time)
    # Gf transforms use row vectors. Transpose into a column-vector rotation,
    # then flip USD +Y(up), +Z(back) to ROS optical +Y(down), +Z(forward).
    rotation = [[float(relative[j][i]) * (1. if j == 0 else -1.)
                 for j in range(3)] for i in range(3)]
    record = dict(mount_path=str(mount.GetPath()),
                  mount_translation_robot_m=[float(v) for v in translation],
                  camera_position_robot_m=[float(v) for v in relative.ExtractTranslation()],
                  robot_from_optical_rotation=rotation,
                  camera_local_matrix4x4=[[float(local[i][j]) for j in range(4)] for i in range(4)])
    values = (record['mount_translation_robot_m'] + record['camera_position_robot_m'] +
              [v for row in rotation for v in row] + [float(local[i][j]) for i in range(4) for j in range(4)])
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite actual depth camera transform')
    return record


def camera_projection_record(camera, name, prim_path, depth_topic, info_topic,
                             frame_id, width, height):
    if type(width) is not int or type(height) is not int or width < 8 or height < 8:
        raise ValueError('depth render dimensions must be integers >= 8')
    focal = float(camera.GetFocalLengthAttr().Get())
    horizontal = float(camera.GetHorizontalApertureAttr().Get())
    vertical = float(camera.GetVerticalApertureAttr().Get())
    if not all(math.isfinite(value) and value > 0. for value in (focal, horizontal, vertical)):
        raise ValueError(f'invalid actual USD camera projection: {prim_path}')
    fx, fy = width * focal / horizontal, height * focal / vertical
    return dict(name=name, prim_path=prim_path, depth_topic=depth_topic,
                camera_info_topic=info_topic, frame_id=frame_id, width=width, height=height,
                focal_length=focal, horizontal_aperture=horizontal, vertical_aperture=vertical,
                hfov_deg=math.degrees(2. * math.atan(horizontal / (2. * focal))),
                vfov_deg=math.degrees(2. * math.atan(vertical / (2. * focal))),
                fx=fx, fy=fy, cx=width * .5, cy=height * .5,
                projection=str(camera.GetProjectionAttr().Get()),
                lens_distortion_model=camera.GetPrim().GetAttribute('omni:lensdistortion:model').Get(),
                horizontal_aperture_offset=float(camera.GetHorizontalApertureOffsetAttr().Get()),
                vertical_aperture_offset=float(camera.GetVerticalApertureOffsetAttr().Get()))


def configure_depth_projection(stage, Usd, UsdGeom, views, width, height,
                               environment=None, run_id='', scene_path=''):
    hfov = requested_hfov(environment)
    offset = requested_camera_offset(environment)
    cameras = []
    for name, prim_path, depth_topic, info_topic, frame_id in views:
        camera = UsdGeom.Camera(stage.GetPrimAtPath(prim_path))
        if not camera:
            raise ValueError(f'invalid depth camera: {prim_path}')
        before = camera_projection_record(camera, name, prim_path, depth_topic,
                                          info_topic, frame_id, width, height)
        if offset is not None:
            mount = camera.GetPrim().GetParent()
            if mount.GetParent().GetPath() != '/World/Robot':
                raise ValueError('runtime camera offset requires a direct Robot child mount')
            attribute = mount.GetAttribute('xformOp:translate')
            translation = attribute.Get()
            if translation is None or len(translation) != 3:
                raise ValueError('runtime camera offset requires an authored mount translation')
            radial = math.hypot(translation[0], translation[1])
            if not math.isfinite(radial) or radial <= 0. or not math.isfinite(translation[2]):
                raise ValueError('invalid authored radial camera mount')
            # Preserve the authored azimuth, height and every rotation op.
            # This moves a virtual sensor; it never edits collision geometry,
            # suppresses returns, or pretends the displaced camera is body size.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                attribute.Set(type(translation)(offset * translation[0] / radial,
                                                offset * translation[1] / radial, translation[2]))
        if hfov is not None:
            if (before['projection'] != 'perspective' or before['lens_distortion_model'] is not None or
                    before['horizontal_aperture_offset'] != 0. or before['vertical_aperture_offset'] != 0.):
                raise ValueError('runtime HFOV override requires a centered, undistorted pinhole camera')
            # Keep authored horizontal aperture; use actual stored aperture after
            # Set/readback (USD attributes are float), never pretend requested
            # values are measured. No USD layer is modified when env is unset.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                camera.GetFocalLengthAttr().Set(before['horizontal_aperture'] /
                                               (2. * math.tan(math.radians(hfov) / 2.)))
                camera.GetVerticalApertureAttr().Set(before['horizontal_aperture'] * height / width)
        record = camera_projection_record(camera, name, prim_path, depth_topic,
                                          info_topic, frame_id, width, height)
        record.update(camera_extrinsic_record(stage, Usd, UsdGeom, camera.GetPrim()))
        if hfov is not None and not math.isclose(record['fx'], record['fy'], rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError('actual camera apertures do not match square-pixel render aspect')
        cameras.append(record)
    return dict(version=SENSOR_VERSION, run_id=run_id, scene_path=scene_path,
                requested_hfov_deg=hfov, override_applied=hfov is not None,
                requested_camera_offset_m=offset, camera_offset_override_applied=offset is not None,
                render_width=width, render_height=height, cameras=cameras,
                evidence_source='USD projection and Robot-relative transform readback before render-product creation; ROS readback is separate')


def save_projection_evidence(record, result_path):
    output = Path(result_path).with_suffix('.sensors.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
    return output


if __name__ == '__main__':
    if sys.argv[1:] != ['--check-env']:
        raise SystemExit('usage: depth_camera_projection.py --check-env')
    try:
        requested_hfov()
        requested_camera_offset()
    except ValueError as exc:
        raise SystemExit(str(exc))
