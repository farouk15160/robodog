from glob import glob
from setuptools import find_packages, setup

package_name = "robodog_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farouk",
    maintainer_email="farouk15160@gmail.com",
    description="Control stack for the robodog quadruped.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "control_node = robodog_control.control_node:main",
        "joint_test = robodog_control.cli.joint_test:main",
        "pose = robodog_control.cli.pose:main",
    ]},
)
