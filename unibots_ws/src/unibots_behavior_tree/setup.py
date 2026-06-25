import os
from glob import glob
from setuptools import setup

package_name = 'unibots_behavior_tree'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'py_trees', 'py_trees_ros'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@email.com',
    description='Behavior tree decision logic for Unibots 2026',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'main_tree_node = unibots_behavior_tree.main_tree_node:main',
        ],
    },
)