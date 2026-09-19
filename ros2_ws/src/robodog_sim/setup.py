from glob import glob
from setuptools import find_packages, setup

package_name = "robodog_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/models", glob("models/*.xml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farouk",
    maintainer_email="farouk15160@gmail.com",
    description="MuJoCo simulation and test worlds for robodog.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "generate_models = robodog_sim.generate_models:main",
        "world_markers = robodog_sim.world_markers_node:main",
        "viewer = robodog_sim.viewer:main",
    ]},
)
