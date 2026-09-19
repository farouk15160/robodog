from glob import glob
from setuptools import find_packages, setup

package_name = "robodog_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/www", glob("www/*.html")),
        ("share/" + package_name + "/www/css", glob("www/css/*.css")),
        ("share/" + package_name + "/www/js", glob("www/js/*.js")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farouk",
    maintainer_email="farouk15160@gmail.com",
    description="Web GUI for the robodog quadruped.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "web_server = robodog_web.web_server:main",
    ]},
)
