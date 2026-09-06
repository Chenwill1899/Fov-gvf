from glob import glob
from setuptools import find_packages, setup

name = "pc_gvf_platforms"
setup(
    name=name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
        ("share/" + name + "/launch", glob("launch/*.launch.py")),
        ("share/" + name + "/config", glob("config/*")),
        ("share/" + name + "/meshes", glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "position_cmd_to_twist = pc_gvf_platforms.command_bridge:main",
        "kinematic_sim = pc_gvf_platforms.kinematic_sim:main",
        "joy_to_intent = pc_gvf_platforms.joy_to_intent:main",
        "intent_trace_replay = pc_gvf_platforms.intent_trace_replay:main",
        "navigation_visualizer = pc_gvf_platforms.navigation_visualizer:main",
    ]},
)
