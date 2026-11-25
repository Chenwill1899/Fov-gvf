#!/usr/bin/python3
"""Build the spherical cruise controller from local sources and publish its verified inputs."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "src/auto_cruise"
MODEL = ROOT / "src/r05_model"
CORE = ROOT / "src/new_controller/ldlt"
DEPS = ROOT / "src/third_party"
LIB = DEPS / "suitesparse/usr/lib/x86_64-linux-gnu"
RELEASES = ROOT / "src/auto_cruise/releases"

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def inputs():
    files = [Path(__file__), APP/"controller.cpp", APP/"spherical_guide.py",
             APP/"depth_timing/include/new_controller/depth_observations.hpp",
             APP/"include/new_controller/depth_camera_workers.hpp",
             APP/"simulation.py", APP/"supervise.py", APP/"audit.py",
             APP/"hangar_guide.csv", APP/"scenarios.json", MODEL/"presentation.py",
             CORE/"src/sparse_trial_backend.cpp"]
    files.extend(APP.glob("*.hpp"))
    files.extend(ROOT/s for s in ("src/new_controller/config/base_step1.json","src/new_controller/config/navigation_step2.json",
        "scripts/run_auto_cruise.py","scripts/run_isaac_closed_loop.py","scripts/base_config.py",
        "scripts/isaac/depth_camera_projection.py","scripts/isaac/manual_control_math.py"))
    for folder in (CORE/"include",MODEL/"src",MODEL/"include",DEPS/"amgcl-1.4.9/amgcl",
                   DEPS/"suitesparse/usr/include",ROOT/"scenes/r05_cloud",ROOT/"scenes/reference_hangar",
                   ROOT/"scenes/robots/arl_robot_1"):
        files.extend(p for p in folder.rglob("*") if p.is_file())
    return {str(p.relative_to(ROOT)):digest(p) for p in sorted(files)}

def build(tmp):
    exe = tmp/"auto-controller"
    # Use the cruise camera handoff and observation probes before the generic core.
    flags = ["-O3","-DNDEBUG","-fno-fast-math","-pthread","-std=gnu++17",
             "-DAMGCL_NO_BOOST","-DEGO4P0_COARSE_PC_MODE=2","-DEGO4P0_CORNER_SMALL_STEP=1",
             "-DEGO4P0_CURVE_GUARD=1","-DEGO4P2_STEP2=1"]
    include = [APP/"depth_timing/include", APP/"include", CORE/"include", MODEL/"include",
               DEPS/"amgcl-1.4.9", DEPS/"suitesparse/usr/include/suitesparse"]
    rpath = "$ORIGIN/"+os.path.relpath(LIB,tmp)
    subprocess.run(["/usr/bin/c++",*flags,*["-I"+str(p) for p in include],"-isystem","/usr/include/eigen3",
                    APP/"controller.cpp",MODEL/"src/verified_start.cpp",CORE/"src/sparse_trial_backend.cpp",
                    "-Wl,--disable-new-dtags","-Wl,-rpath,"+rpath,"-Wl,-rpath-link,"+str(LIB),"-lyaml-cpp","-lcrypto",
                    LIB/"libcholmod.so","-o",exe],check=True)
    linked = subprocess.check_output(["/usr/bin/ldd",exe],text=True)
    if "not found" in linked:
        raise RuntimeError(linked)
    deps = {}
    for word in linked.split():
        if word.startswith("/") and Path(word).is_file():
            path = Path(word).resolve()
            key = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
            deps[str(key)] = digest(path)
    if str((LIB/"libcholmod.so").resolve().relative_to(ROOT)) not in deps:
        raise RuntimeError("Expected the project-local CHOLMOD library")
    return exe,deps

def main():
    if os.geteuid()==0:
        raise SystemExit("Build as the ordinary project user")
    RELEASES.mkdir(parents=True,exist_ok=True)
    with (RELEASES/".build.lock").open("a") as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        with tempfile.TemporaryDirectory(dir=RELEASES) as working:
            tmp = Path(working)
            before = inputs()
            exe,deps = build(tmp)
            if inputs()!=before:
                raise RuntimeError("Source files changed during compilation")
            identity = dict(profile="spherical-cruise",source_identity=before,dependencies=deps)
            binary_sha = digest(exe)
            key = hashlib.sha256((binary_sha+json.dumps(identity,sort_keys=True)).encode()).hexdigest()[:24]
            output = RELEASES/key
            identity["binary"] = dict(path=str((output/exe.name).relative_to(ROOT)),sha256=binary_sha)
            manifest = tmp/"manifest.json"
            manifest.write_text(json.dumps(identity,indent=2)+"\n")
            if output.exists():
                if digest(output/exe.name)!=binary_sha or (output/manifest.name).read_bytes()!=manifest.read_bytes():
                    raise RuntimeError("Existing release differs from the rebuilt files")
            else:
                staged = tmp/"release"
                staged.mkdir()
                exe.rename(staged/exe.name)
                manifest.rename(staged/manifest.name)
                staged.rename(output)
            pointer = tmp/"latest_build.json"
            pointer.write_text(json.dumps({"manifest":str((output/"manifest.json").relative_to(ROOT))})+"\n")
            pointer.replace(RELEASES/pointer.name)
            print(output/"auto-controller")

if __name__=="__main__":
    main()
