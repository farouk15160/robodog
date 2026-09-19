from setuptools import find_packages, setup

package_name = "robodog_hardware"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/robstride_bus.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farouk",
    maintainer_email="farouk15160@gmail.com",
    description="Hardware abstraction and ROBSTRIDE02 CAN driver for robodog.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "calibrate_joint = robodog_hardware.tools.calibrate_joint:main",
    ]},
)
