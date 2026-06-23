# unibotsUK

## Quickstart

Run
```
nix-shell
distrobox assemble create --replace
distrobox enter ubuntu-26.04-ros2
```
This will provide access to the ROS2 + Gazebo environment.
Requires nix installed.

Whenever you need to introduce a new dependency, make sure to add it to setup.sh.

## Common commands
python3 src/unibots_control/scripts/build_mpc_solver.py

colcon build

source install/setup.bash

pkill -9 -f 'gz sim'; pkill -9 -f ruby

ros2 launch unibots_sim full_sim.launch.py controller:=mpc #apf alternative