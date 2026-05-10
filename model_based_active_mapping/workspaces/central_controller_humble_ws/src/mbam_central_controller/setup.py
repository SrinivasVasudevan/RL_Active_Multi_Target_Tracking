from glob import glob
import os
from setuptools import setup

package_name = "mbam_central_controller"


def files_in(path_pattern):
    return [f for f in glob(path_pattern, recursive=True) if os.path.isfile(f)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        f"{package_name}.core",
        f"{package_name}.nodes",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), files_in("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), files_in("config/*")),
        (os.path.join("share", package_name, "checkpoints"), files_in("checkpoints/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mbam",
    maintainer_email="maintainer@example.com",
    description="Central MBAM controller for real LIMO robots (ROS 2 Humble).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "central_runner = mbam_central_controller.nodes.central_runner_node:main",
        ],
    },
)
