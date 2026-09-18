from glob import glob
from setuptools import find_packages, setup

package_name = "pc_gvf"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "depth_angular_demo_sim = pc_gvf.depth_angular_demo_sim:main",
        "depth_angular_core = pc_gvf.depth_angular_core:main",
    ]},
)
