"""ament_python setup for the unibots_ultrasonic package."""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "unibots_ultrasonic"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament_index resource marker.
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", PACKAGE_NAME)]),
        # package manifest.
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        # launch files.
        (os.path.join("share", PACKAGE_NAME, "launch"),
         glob("launch/*.launch.py")),
        # sensor configuration.
        (os.path.join("share", PACKAGE_NAME, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Faraaz Ali Khan",
    maintainer_email="itsfaraazak@gmail.com",
    description=(
        "Ultrasonic proximity sensing and collision-avoidance fusion for the "
        "Unibots UK 2026 holonomic mecanum competition robot."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ultrasonic_node = unibots_ultrasonic.ultrasonic_node:main",
        ],
    },
)
