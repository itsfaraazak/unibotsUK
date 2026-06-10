# Unibots UK 2025-2026 — UniDesign2 Robot XACRO Package

## Package structure
```
unibots_robot/
├── urdf/
│   └── robot.xacro          ← Main robot description (edit this)
├── meshes/
│   └── *.stl                ← All 3D-printed parts from Fusion 360
├── launch/
│   ├── display.launch.py    ← Visualise in RViz2
│   └── gazebo.launch.py     ← Simulate in Gazebo + Unibots arena
├── worlds/
│   └── unibots_arena.world  ← 2m×2m arena with nets + balls
├── config/
│   └── robot.rviz           ← RViz preset
├── CMakeLists.txt
└── package.xml
```

## Quick start

### 1. Place package in your ROS2 workspace
```bash
cp -r unibots_robot ~/ros2_ws/src/
```

### 2. Build
```bash
cd ~/ros2_ws
colcon build --packages-select unibots_robot
source install/setup.bash
```

### 3. Visualise in RViz2
```bash
ros2 launch unibots_robot display.launch.py
```

### 4. Simulate in Gazebo
```bash
ros2 launch unibots_robot gazebo.launch.py
```

### 5. Drive the robot (in a separate terminal)
```bash
# Install teleop if needed: sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/unibots/cmd_vel
```

## Topics
| Topic | Type | Description |
|---|---|---|
| `/unibots/cmd_vel` | `geometry_msgs/Twist` | Drive command input |
| `/unibots/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/unibots/imu/data` | `sensor_msgs/Imu` | IMU readings |
| `/unibots/camera/image_raw` | `sensor_msgs/Image` | Camera feed |

## Robot dimensions (from Fusion 360 STLs)
| Part | Size (mm) |
|---|---|
| Bottom plate | 150 × 175 × 5 |
| Top plate | 190 × 190 × 5 |
| Column height | ~75 |
| Total height (est.) | ~200 |
| Mecanum wheel diameter | 60 |
| Wheelbase (F↔B) | 120 |
| Track width (L↔R) | 150 |

## Customising

### Change wheel size
In `robot.xacro`, edit the properties section at the top:
```xml
<xacro:property name="w_r" value="0.030"/>  <!-- radius in metres -->
<xacro:property name="w_y" value="0.075"/>  <!-- half track width -->
```

### Add a servo joint for the claw
Change `claw_joint` type from `fixed` to `revolute` and add limits:
```xml
<joint name="claw_joint" type="revolute">
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="0.0" effort="1.0" velocity="1.0"/>
  ...
</joint>
```

### Swap STL meshes
Replace any file in `meshes/` and rebuild — names must match exactly.

## Competition notes (Unibots 2025-2026 rulebook)
- Robot must start within 200 mm cube → fits ✓ (chassis is 175×150×200 mm)
- Fully autonomous — no SSH or remote control during match
- Must have physical power switch accessible at all times
- Must display team number board (≥ 50×50 mm rigid material)
