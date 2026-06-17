"""ament_python setup for the unibots_localization package."""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "unibots_localization"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament resource index marker.
        ("share/ament_index/resource_index/packages",
         ["resource/" + PACKAGE_NAME]),
        # package manifest.
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        # launch files.
        (os.path.join("share", PACKAGE_NAME, "launch"),
         glob("launch/*.launch.py")),
        # config files (ekf.yaml, tags.yaml, apriltag.yaml).
        (os.path.join("share", PACKAGE_NAME, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krishiv Agrawal",
    maintainer_email="krishivagrawal7@gmail.com",
    description=(
        "AprilTag-to-absolute-pose bridge and robot_localization EKF for the "
        "Unibots UK 2026 mecanum competition robot."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_bridge_node = unibots_localization.ekf_bridge_node:main",
        ],
    },
)
