"""Finite simulation fixtures and independent continuous sphere-sweep checks.

Geometry checks do not depend on the navigation library or on rendered depth.
USD imports are deferred so these checks can be exercised without Isaac.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


VIEWS = (
    ("Front", "/World/Robot/CameraMount/DepthCamera", "/sim/depth/image_raw", "/sim/depth/camera_info", "camera_front_optical"),
    ("Left", "/World/Robot/CameraMountLeft/DepthCamera", "/sim/depth_left/image_raw", "/sim/depth_left/camera_info", "camera_left_optical"),
    ("Back", "/World/Robot/CameraMountBack/DepthCamera", "/sim/depth_back/image_raw", "/sim/depth_back/camera_info", "camera_back_optical"),
    ("Right", "/World/Robot/CameraMountRight/DepthCamera", "/sim/depth_right/image_raw", "/sim/depth_right/camera_info", "camera_right_optical"),
)


def read_scenario(path: Path) -> tuple[dict, str]:
    raw = path.read_bytes()
    scenario = json.loads(raw)
    if scenario.get("schema") != 1 or scenario.get("static_scene") is not True:
        raise ValueError("expected schema=1 and an explicitly static scene")
    for key in ("bounds_min", "bounds_max", "start", "goal"):
        value = scenario[key]
        if len(value) != 3 or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in value):
            raise ValueError("invalid three-vector: " + key)
    if not all(a < b for a, b in zip(scenario["bounds_min"], scenario["bounds_max"])):
        raise ValueError("invalid room bounds")
    for key in ("physics_dt_s", "body_radius_m", "camera_near_m", "camera_far_m", "seed_radius_m",
                "cruise_speed_mps", "arrival_radius_m", "arrival_speed_mps", "arrival_hold_s",
                "simulation_timeout_s", "wall_timeout_s", "camera_hfov_deg"):
        if not isinstance(scenario[key], (float, int)) or not math.isfinite(scenario[key]) or scenario[key] <= 0:
            raise ValueError("invalid positive scalar: " + key)
    if scenario.get("camera_offset_m") != 0 or not 0 < scenario["camera_hfov_deg"] < 180:
        raise ValueError("this fixture requires co-located centered pinhole cameras")
    if scenario["camera_near_m"] >= scenario["camera_far_m"]:
        raise ValueError("invalid clipping planes")
    for key in ("image_width", "image_height"):
        if type(scenario[key]) is not int or not 8 <= scenario[key] <= 4096:
            raise ValueError("invalid image dimension: " + key)
    for box in scenario["boxes"]:
        if any(len(box[k]) != 3 or not all(math.isfinite(v) for v in box[k]) for k in ("min", "max")):
            raise ValueError("invalid box bounds")
        if not all(a < b for a, b in zip(box["min"], box["max"])):
            raise ValueError("degenerate box")
    for name in ("start", "goal"):
        clearance, _ = swept_clearance(scenario[name], scenario[name], scenario["body_radius_m"], scenario)
        if clearance <= 0:
            raise ValueError(name + " body intersects scene geometry")
    seed_clearance, _ = swept_clearance(scenario["start"], scenario["start"], scenario["seed_radius_m"], scenario)
    if seed_clearance <= 0:
        raise ValueError("initial seed sphere is not strictly inside known empty scene space")
    return scenario, hashlib.sha256(raw).hexdigest()


def segment_aabb_distance(begin, end, lower, upper):
    """Exact segment-to-box distance, minimizing each piecewise quadratic."""
    delta = [end[i] - begin[i] for i in range(3)]
    cuts = {0.0, 1.0}
    for i in range(3):
        if delta[i] != 0:
            for boundary in (lower[i], upper[i]):
                t = (boundary - begin[i]) / delta[i]
                if 0 < t < 1:
                    cuts.add(t)
    cuts = sorted(cuts)

    def squared(t):
        return sum(max(lower[i] - (begin[i] + t * delta[i]),
                       begin[i] + t * delta[i] - upper[i], 0.0) ** 2 for i in range(3))

    best = min(squared(t) for t in cuts)
    for first, last in zip(cuts[:-1], cuts[1:]):
        mid = (first + last) * .5
        quadratic = linear = 0.0
        for i in range(3):
            value = begin[i] + mid * delta[i]
            bound = lower[i] if value < lower[i] else upper[i] if value > upper[i] else None
            if bound is not None:
                quadratic += delta[i] ** 2
                linear += delta[i] * (begin[i] - bound)
        if quadratic:
            t = min(last, max(first, -linear / quadratic))
            best = min(best, squared(t))
    return math.sqrt(max(0.0, best))


def swept_clearance(begin, end, radius, scenario):
    if len(begin) != 3 or len(end) != 3 or not all(math.isfinite(v) for v in (*begin, *end, radius)) or radius <= 0:
        raise ValueError("nonfinite/invalid sweep")
    lower, upper = scenario["bounds_min"], scenario["bounds_max"]
    clearance = min(min(begin[i], end[i]) - lower[i] - radius for i in range(3))
    clearance = min(clearance, min(upper[i] - max(begin[i], end[i]) - radius for i in range(3)))
    nearest = "room_boundary"
    for index, box in enumerate(scenario["boxes"]):
        value = segment_aabb_distance(begin, end, box["min"], box["max"]) - radius
        if value < clearance:
            clearance, nearest = value, box.get("name", "box_" + str(index))
    return clearance, nearest


def build_scene(stage, scenario):
    """Build only the declared room, boxes, and a tiny co-located camera rig."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    physics = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics.CreateGravityMagnitudeAttr(0.0)

    def surface(prim, color):
        # Explicit opaque visualization material. A low emissive component
        # keeps closed-room geometry readable without adding sensor geometry.
        material = UsdShade.Material.Define(stage, "/World/Materials/" + prim.GetName())
        shader = UsdShade.Shader.Define(stage, str(material.GetPath()) + "/Surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[v*.12 for v in color]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.7)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

    def cube(path, lower, upper, color):
        value = UsdGeom.Cube.Define(stage, path)
        value.CreateSizeAttr(1.0)
        value.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        value.CreateDoubleSidedAttr(True)
        transform = UsdGeom.XformCommonAPI(value)
        transform.SetTranslate(Gf.Vec3d(*[(a + b) * .5 for a, b in zip(lower, upper)]))
        transform.SetScale(Gf.Vec3f(*[b - a for a, b in zip(lower, upper)]))
        surface(value.GetPrim(), color)
        return value

    lower, upper = scenario["bounds_min"], scenario["bounds_max"]
    for axis in range(3):
        for side in (0, 1):
            lo = [v - .1 for v in lower]
            hi = [v + .1 for v in upper]
            if side == 0:
                lo[axis], hi[axis] = lower[axis] - .1, lower[axis]
            else:
                lo[axis], hi[axis] = upper[axis], upper[axis] + .1
            color = (.24, .29, .35) if axis == 2 and side == 0 else (.52, .57, .62)
            cube(f"/World/Room/Wall_{axis}_{side}", lo, hi, color)
    for index, box in enumerate(scenario["boxes"]):
        cube(f"/World/Obstacles/Box_{index}", box["min"], box["max"], (.85, .38, .12))
    robot = UsdGeom.Xform.Define(stage, "/World/Robot")
    transform = UsdGeom.XformCommonAPI(robot)
    transform.SetTranslate(Gf.Vec3d(*scenario["start"]))
    transform.SetRotate(Gf.Vec3f(0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    # All visible body geometry is inside every .10m near plane. The certified
    # body is the .10m sphere, not the larger ARL appearance used in old scenes.
    body = UsdGeom.Sphere.Define(stage, "/World/Robot/Body")
    body.CreateRadiusAttr(min(.085, scenario["body_radius_m"] * .85, scenario["camera_near_m"] * .85))
    body.CreateDisplayColorAttr([Gf.Vec3f(.05, .75, .95)])
    surface(body.GetPrim(), (.05, .75, .95))
    body.GetPrim().CreateAttribute("certified_radius_m", Sdf.ValueTypeNames.Double).Set(scenario["body_radius_m"])
    # USD camera -Z points forward, +Y up. Row-vector local basis maps the
    # front camera to body +X, with optical +X=-body Y and +Y=-body Z.
    camera_matrix = Gf.Matrix4d(0, -1, 0, 0, 0, 0, 1, 0, -1, 0, 0, 0, 0, 0, 0, 1)
    for view, yaw in zip(VIEWS, (0, 90, 180, -90)):
        mount = UsdGeom.Xform.Define(stage, str(Path(view[1]).parent))
        mount.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        mount.AddRotateZOp().Set(float(yaw))
        camera = UsdGeom.Camera.Define(stage, view[1])
        camera.AddTransformOp().Set(camera_matrix)
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        camera.CreateHorizontalApertureAttr(24.0)
        camera.CreateVerticalApertureAttr(24.0 * scenario["image_height"] / scenario["image_width"])
        camera.CreateFocalLengthAttr(12.0 / math.tan(math.radians(scenario["camera_hfov_deg"] / 2)))
        camera.CreateClippingRangeAttr(Gf.Vec2f(scenario["camera_near_m"], scenario["camera_far_m"]))
        camera.GetPrim().ApplyAPI("OmniSensorAPI")
        camera.GetPrim().CreateAttribute("omni:sensor:tickRate", Sdf.ValueTypeNames.Float).Set(1.0 / scenario["physics_dt_s"])
        camera.GetPrim().CreateAttribute("omni:sensor:frameRate", Sdf.ValueTypeNames.Double2).Set(Gf.Vec2d(1.0 / scenario["physics_dt_s"], 1.0))
    chase = UsdGeom.Camera.Define(stage, "/World/ChaseCamera")
    chase.CreateClippingRangeAttr(Gf.Vec2f(.01, 1000.0))
    chase.CreateFocalLengthAttr(18.0)
    chase.CreateHorizontalApertureAttr(36.0)
    chase.CreateVerticalApertureAttr(20.25)
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(600.0)
    # The room is closed: exterior dome illumination alone can leave the
    # camera viewport black. Point lights have no visible surface/depth return.
    for index, fraction in enumerate((.2, .5, .8)):
        light = UsdLux.SphereLight.Define(stage, f"/World/RoomLight_{index}")
        light.CreateTreatAsPointAttr(True)
        light.CreateRadiusAttr(0.0)
        light.CreateIntensityAttr(5000.0)
        light.AddTranslateOp().Set(Gf.Vec3d(lower[0] + fraction * (upper[0]-lower[0]),
                                          (lower[1]+upper[1])*.5, upper[2]-.3))
    stage.GetRootLayer().customLayerData = {"closedLoopScenario": json.dumps(scenario, sort_keys=True)}
    return robot
