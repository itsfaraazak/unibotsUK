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


