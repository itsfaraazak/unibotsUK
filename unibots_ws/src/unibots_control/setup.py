"""ament_python setup for the unibots_control package."""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "unibots_control"

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
        # offline solver build script.
        (os.path.join("share", PACKAGE_NAME, "scripts"),
         glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krishiv Agrawal",
    maintainer_email="krishivagrawal7@gmail.com",
    description=(
        "Iterative MPC (iMPC) and APF navigation controllers for the Unibots "
        "UK 2026 holonomic mecanum competition robot."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpc_controller_node = unibots_control.mpc_controller_node:main",
            "apf_controller_node = unibots_control.apf_controller_node:main",
            'hardware_motor_node = unibots_control.hardware_motor_node:main',
            'hardware_servo_node = unibots_control.hardware_servo_node:main',
        ],
    },
)
