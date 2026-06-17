"""ament_python package setup for unibots_game (top-level match FSM)."""

import os
from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "unibots_game"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Krishiv Agrawal",
    maintainer_email="krishivagrawal7@gmail.com",
    description=(
        "Top-level match finite-state machine for the Unibots UK 2026 "
        "mecanum competition robot."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "game_state_node = unibots_game.game_state_node:main",
        ],
    },
)
