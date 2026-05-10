import os
from glob import glob

from setuptools import setup

package_name = "mbam_limo_agent"


def files_in(pattern):
    return [f for f in glob(pattern, recursive=True) if os.path.isfile(f)]


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), files_in("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "cmd_vel_safety = mbam_limo_agent.cmd_vel_safety_node:main",
        ],
    },
)
