# Unibots UK 2026 — Complete ELI5 Implementation Guide
> You have: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic installed.  
> You are building: a fully autonomous mecanum robot that collects orange ping-pong balls  
> and steel bearings in a 2 m × 2 m arena, scoring by launching balls over a 150 mm wall.  
> Competition date: 27 June 2026.

---

## How to Use This Guide

Read each section in order. Every section has three parts:

1. **🧠 What it is** — a simple analogy so you understand WHY before you touch code  
2. **🔧 How to do it** — exact commands and files to run/create  
3. **✅ How to know it worked** — what you should see on screen  

Do not skip ahead. Each layer depends on the previous one being verified.

---

# PART 0 — The Big Picture

Before touching anything, read this once so nothing feels random later.

## What the Robot Does (In Plain English)

Every 180 seconds, your robot has to:

1. **Find** orange ping-pong balls and steel bearings on a white floor
2. **Drive** to them without hitting other robots or walls
3. **Collect** ping-pong balls (they go in a hopper), let bearings stick to a magnetic skirt passively
4. **Return** to its own coloured wall and **launch** ping-pong balls over the wall into its net
5. **Touch** its coloured wall in the last few seconds (parking = 3 free points)

## The Points (Memorise This — It Drives All Design Decisions)

| What | Points | Notes |
|---|---|---|
| Ping-pong ball in YOUR net | **4** | Must clear the 150 mm wall |
| Bearing in YOUR net | **2** | Must also clear the wall |
| Bearing stuck to robot | **1** | Just touching your robot counts — magnets give you this for free |
| Parking (touching your wall at end) | **3** | Free points, never miss this |
| Ping-pong ball held but NOT in net | **0** | Holding it means nothing |

**Critical insight:** Your magnetic skirt picks up bearings as you drive. Each one touching your chassis = 1 point, zero software required. 24 bearings in the arena = up to 24 free points. This is your safety floor. Win the match on top of that with ping-pong balls.

## The Software Layers (Top to Bottom)

Think of the robot's software like a company with departments that only talk through a message board:

```
┌─────────────────────────────────────────────┐
│  BRAIN: Behaviour Tree                       │  "What should I do right now?"
│  (decides: search / collect / deposit / park)│
├─────────────────────────────────────────────┤
│  NAVIGATION: Nav2                            │  "How do I get there?"
│  (path planning, obstacle avoidance)         │
├─────────────────────────────────────────────┤
│  LOCALISATION: EKF + AprilTags               │  "Where am I in the arena?"
│  (fuses IMU + odometry + tag sightings)      │
├─────────────────────────────────────────────┤
│  PERCEPTION: YOLO + Tracker                  │  "What do I see?"
│  (detects balls, bearings, robots)           │
├─────────────────────────────────────────────┤
│  HARDWARE: ros2_control                      │  "Spin motor 3 at 47 RPM"
│  (talks directly to motors and sensors)      │
└─────────────────────────────────────────────┘
```

Every layer talks to the layers above/below it through **ROS 2 topics** — named message channels. They never call each other directly. This is the key design principle.

---

# PART 1 — ROS 2: The Foundation

## 🧠 What it is

**Analogy:** Imagine a large office where every piece of software is a different worker (called a **node**). Workers never talk to each other face to face. Instead, they post notes on named noticeboards around the office (called **topics**). Worker A (camera node) posts "I see a ball at pixel 320, 240" on the noticeboard called `/ball_detections`. Worker B (navigation node) reads that noticeboard and decides where to drive. Worker C (motor node) reads the driving commands noticeboard and spins the wheels.

This means: you can swap out any worker without touching the others, as long as they read/write the same noticeboards. That is why ROS 2 is good for a maintainable platform.

## Key Vocabulary (you will see these constantly)

| Term | Plain English |
|---|---|
| **Node** | One running program (e.g., camera_node.py) |
| **Topic** | A named channel (e.g., `/cmd_vel`, `/detections`) |
| **Message** | The data packet posted to a topic (e.g., a `Twist` message = linear + angular velocity) |
| **Publisher** | A node that writes to a topic |
| **Subscriber** | A node that reads from a topic |
| **Service** | A one-off request-response (e.g., "start the arm") unlike a topic which is continuous |
| **Launch file** | A script that starts multiple nodes at once |
| **Package** | A folder of related nodes, configs, and launch files — the unit of sharing |
| **Workspace** | Your project folder — contains all your packages |

## 🔧 Setting Up Your Workspace

A ROS 2 workspace is just a folder with a specific structure. You will work in this folder for everything.

```bash
# Create your workspace
mkdir -p ~/unibots_ws/src
cd ~/unibots_ws

# This is where ALL your code lives:
# ~/unibots_ws/src/your_package_1/
# ~/unibots_ws/src/your_package_2/
# etc.

# Install core dependencies you will need
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-core \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-robot-localization \
  ros-jazzy-apriltag-ros \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux \
  ros-jazzy-slam-toolbox

# Initialise rosdep (tool that installs package dependencies)
sudo rosdep init   # only if you haven't done this before — ignore error if already done
rosdep update

# Source ROS 2 in every terminal (add this to ~/.bashrc so it's automatic)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Creating Your First Package

```bash
cd ~/unibots_ws/src

# Create a Python package called 'unibots_robot'
ros2 pkg create --build-type ament_python unibots_robot \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs

# This creates:
# unibots_robot/
#   unibots_robot/       ← your Python files go here
#   resource/
#   test/
#   package.xml          ← declares dependencies
#   setup.py             ← registers your nodes
```

## Building the Workspace

```bash
cd ~/unibots_ws

# Build everything in src/
colcon build --symlink-install

# The --symlink-install flag means: when you edit a .py file,
# you do NOT need to rebuild — changes take effect immediately.

# After building, source the workspace (also add to ~/.bashrc)
echo "source ~/unibots_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ✅ Verify ROS 2 Works

```bash
# In terminal 1 — start a demo publisher
ros2 run demo_nodes_py talker

# In terminal 2 — listen to it
ros2 run demo_nodes_py listener
```

You should see "Hello World: 1", "Hello World: 2" etc. in terminal 2.
If you do, ROS 2 is working. Close both with Ctrl+C.

```bash
# Useful commands you will use constantly:
ros2 topic list          # show all active topics
ros2 topic echo /topic   # print messages on a topic
ros2 node list           # show all running nodes
ros2 run <pkg> <node>    # start a single node
ros2 launch <pkg> <file> # start multiple nodes from a launch file
```

---

# PART 2 — Describing Your Robot (URDF)

## 🧠 What it is

**Analogy:** URDF (Unified Robot Description Format) is your robot's birth certificate combined with an instruction manual. It tells every piece of software: "This robot has a rectangular body 200 mm wide. It has 4 wheels at these exact positions. Its camera is mounted 150 mm above the centre, pointing down at 30 degrees." Without this, Nav2 doesn't know how big the robot is, the EKF doesn't know where the IMU is, and Gazebo can't simulate it.

It is written in XML (a text format with opening and closing tags like `<robot>...</robot>`).

Your teammate is building the URDF. Your job is to understand it well enough to use it, check it in Gazebo, and hook it up to ros2_control.

## Structure of a URDF

```xml
<robot name="unibots_robot">

  <!-- A 'link' is a rigid body part -->
  <link name="base_link">
    <visual>
      <!-- What it looks like in simulation -->
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </visual>
    <collision>
      <!-- The collision shape for physics -->
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
    <inertial>
      <!-- Mass and inertia for physics simulation -->
      <mass value="2.0"/>
      ...
    </inertial>
  </link>

  <!-- A 'joint' connects two links -->
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.08 0.09 -0.04" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- wheel spins around Y axis -->
  </joint>

  <!-- Plugins tell Gazebo/ros2_control how to actuate the joints -->
  <ros2_control name="mecanum_system" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <!-- repeat for all 4 wheels -->
  </ros2_control>

</robot>
```

## Using XACRO (Makes URDF Less Repetitive)

XACRO is URDF with variables and macros. Files end in `.urdf.xacro`. Convert to URDF like this:

```bash
# Convert xacro to urdf (check it is valid)
ros2 run xacro xacro my_robot.urdf.xacro > my_robot.urdf

# Visualise in RViz to check it looks right
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## ✅ Verify URDF Works

When you launch the display, RViz should open and show your robot's shape. All links should appear in their correct positions. Move the joint sliders — wheels should turn.

---

# PART 3 — Talking to Motors (ros2_control)

## 🧠 What it is

**Analogy:** Imagine your robot's wheels are workers at a factory floor. `ros2_control` is the factory floor manager. Higher-level software (Nav2) tells the manager "I want the robot to go forward at 0.5 m/s and turn right at 0.2 rad/s." The manager converts this into: "Front-left wheel: 8.3 rad/s. Front-right wheel: 6.1 rad/s. Rear-left wheel: 6.1 rad/s. Rear-right wheel: 8.3 rad/s." It also reads the wheel encoder sensors and reports back how far each wheel has actually turned.

The key thing `ros2_control` gives you: **hardware abstraction**. Your brain (Nav2) never thinks about individual wheels. It just says "go here" and the factory manager figures it out.

## Mecanum Wheel Kinematics (Why This Matters)

A mecanum wheel has rollers on it at 45 degrees. This means:
- All 4 wheels forward → robot moves forward
- Left wheels forward, right wheels backward → robot rotates
- Front wheels in opposite directions to rear wheels → robot strafes sideways

The `mecanum_drive_controller` in ros2_control handles this maths. You never need to write the kinematics yourself.

## Controller Configuration File

Create `config/controllers.yaml` in your package:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz — how often the controller runs

    # Which controllers to load
    mecanum_drive_controller:
      type: mecanum_drive_controller/MecanumDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

mecanum_drive_controller:
  ros__parameters:
    # These MUST match your URDF joint names exactly
    front_left_wheel_name: "front_left_wheel_joint"
    front_right_wheel_name: "front_right_wheel_joint"
    rear_left_wheel_name: "rear_left_wheel_joint"
    rear_right_wheel_name: "rear_right_wheel_joint"

    # Measure these from your actual robot (or Fusion model)
    wheel_separation_x: 0.15  # distance front-to-rear axle (metres)
    wheel_separation_y: 0.17  # distance left-to-right wheel (metres)
    wheel_radius: 0.04        # wheel radius (metres) — measure from Fusion

    # Topic this controller listens to for velocity commands
    # Nav2 will publish here automatically
    publish_rate: 50.0

    # Odometry — this estimates position from wheel rotations
    # It drifts over time (fixed by the EKF later)
    open_loop: false
    enable_odom_tf: false  # EKF will handle the TF, not this controller
```

## Launch File for the Robot

Create `launch/robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterFile
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('unibots_robot')

    # Node that reads the URDF and publishes robot structure
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg, 'urdf', 'robot.urdf')).read()
        }]
    )

    # The ros2_control manager — starts all controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': open(os.path.join(pkg, 'urdf', 'robot.urdf')).read()},
            os.path.join(pkg, 'config', 'controllers.yaml')
        ]
    )

    # These two nodes activate the controllers after the manager starts
    mecanum_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller']
    )
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        mecanum_spawner,
        jsb_spawner,
    ])
```

## ✅ Testing Motor Control (Keyboard Teleop)

```bash
# Terminal 1: launch the robot
ros2 launch unibots_robot robot.launch.py

# Terminal 2: drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mecanum_drive_controller/cmd_vel

# Press i to go forward, j/l to rotate, u/o to strafe diagonally
# The robot should move in Gazebo (or on the real robot)
```

> **Heading hold problem:** When you press strafe, the robot curves instead of going straight. This is the mecanum slip issue. You will fix it in Part 5 when the IMU feeds the EKF, which feeds Nav2's controller. The controller uses the IMU heading to correct the wheel mix automatically. Do not try to fix it here.

---

# PART 4 — Making the Robot See (YOLO Detection)

## 🧠 What it is

**Analogy:** You want someone who can look at any photo and say "there's an orange ball at the top-left, and a silver bearing at the bottom-right." You could write rules ("find orange circles") — that's what your old code did, and it worked for the balls but failed completely for the bearings, because bearings have no consistent colour (they reflect everything around them). 

Instead, you hire an artist who has seen thousands of example photos with labels drawn on them. After training, this artist can recognise things by shape, texture, and context — not just colour. That artist is **YOLO** (You Only Look Once), a neural network. Training it is the process of showing it your labelled examples. After training you get a model file (`.pt`) that you load and run on camera frames.

## Why the Old Colour-Threshold Approach Failed for Bearings

A steel bearing looks different in every lighting condition. It's a mirror: it reflects the orange balls next to it, the green arena walls, your hand. No HSV range can capture "whatever a bearing looks like." A neural net learns "bearings are small, round, metallic-looking blobs that appear like this across many different contexts." That generalises.

**Important:** because your magnetic skirt picks up bearings passively (1 pt each just for touching your robot), your detector's priority is **orange balls** (4 pts each, require active navigation). Solid bearing detection is a bonus, not a blocker.

## Step 1: Set Up Ultralytics

```bash
# Install Ultralytics (the library that wraps YOLO training and inference)
pip3 install ultralytics supervision

# Test it immediately — if this works, you're ready to train
python3 -c "from ultralytics import YOLO; print('YOLO ready')"
```

## Step 2: Collect Training Data

This is the most important step. Your model is only as good as your data.

**What you need:**
- 200–400 photos of the actual arena (or a close simulation)
- Mix of: balls close up, balls far away, balls partially occluded, bearings near balls, bearings alone, different lighting
- At minimum 50–80 images of steel bearings in various positions

**How to collect:**
```bash
# On your robot's Pi, stream the camera over the network
# (or just plug a USB cam into your laptop and point it at some balls on a white sheet)

# Simple script to capture frames from your robot's camera:
python3 - << 'EOF'
import cv2, os, time
os.makedirs("dataset/images", exist_ok=True)
cap = cv2.VideoCapture(0)  # change to camera index or video path
count = 0
print("Press SPACE to capture, Q to quit")
while True:
    ret, frame = cap.read()
    cv2.imshow("Capture", frame)
    key = cv2.waitKey(1)
    if key == ord(' '):
        cv2.imwrite(f"dataset/images/frame_{count:04d}.jpg", frame)
        print(f"Saved frame {count}")
        count += 1
    elif key == ord('q'):
        break
cap.release()
EOF
```

## Step 3: Label Your Data (Roboflow)

1. Go to **https://roboflow.com** and create a free account
2. Create a new project → Object Detection
3. Upload your images
4. Draw bounding boxes around each object:
   - Class `ping_pong_ball` — draw tight box around each orange ball
   - Class `bearing` — draw tight box around each bearing
   - Class `robot` — draw box around any other robots visible
5. Apply augmentations: flip horizontal, flip vertical, rotation ±15°, brightness ±20%, blur. This multiplies your dataset.
6. Export → YOLOv8 format → Download ZIP

**Your dataset folder structure after download:**
```
dataset/
  train/
    images/   ← training photos
    labels/   ← .txt files with bounding box coordinates
  valid/
    images/
    labels/
  test/
    images/
    labels/
  data.yaml   ← tells YOLO what the classes are
```

The `data.yaml` will look like:
```yaml
path: /home/you/dataset
train: train/images
val: valid/images
test: test/images

nc: 3  # number of classes
names: ['ping_pong_ball', 'bearing', 'robot']
```

## Step 4: Train the Model

```bash
# Train YOLO11 nano model on your dataset
# This takes ~20-60 minutes on a laptop GPU, or ~2-4 hours on CPU
yolo detect train \
  model=yolo11n.pt \        # start from pretrained nano weights
  data=/path/to/data.yaml \ # your dataset
  epochs=100 \              # number of training passes
  imgsz=640 \               # image size
  batch=16 \                # images per batch (reduce to 8 if out of memory)
  name=unibots_detector     # name for this training run

# Results saved to: runs/detect/unibots_detector/
# Your best model is at: runs/detect/unibots_detector/weights/best.pt
```

**If you have no GPU:** Use Google Colab (free GPU). Upload your dataset to Google Drive, mount it in Colab, run the same command there. Download `best.pt` when done.

```python
# Quick Colab snippet (paste into a Colab cell):
!pip install ultralytics
from google.colab import drive
drive.mount('/content/drive')
!yolo detect train model=yolo11n.pt data='/content/drive/MyDrive/dataset/data.yaml' epochs=100 imgsz=640 batch=16 name=unibots
```

## Step 5: Test Your Model

```bash
# Run inference on test images
yolo detect predict \
  model=runs/detect/unibots_detector/weights/best.pt \
  source=dataset/test/images \
  conf=0.5 \
  save=True

# Results saved to runs/detect/predict/
# Open the images and check: are boxes drawn correctly?
```

**What good looks like:** Tight boxes around every ball and bearing in the test images. Confidence scores above 0.7. Few false positives (boxes where there is nothing).

**What bad looks like:** Missing detections (recall problem → need more training data, especially close-up shots), boxes on the floor pattern (precision problem → add hard-negative images of just the floor with no balls).

## Step 6: Write the ROS 2 Detection Node

Create `unibots_robot/detection_node.py`:

```python
#!/usr/bin/env python3
"""
Detection Node
- Reads camera frames
- Runs YOLO to detect balls, bearings, robots
- Tracks detections across frames (so balls don't blink in/out)
- Publishes detections with 3D floor positions

Topics published:
  /detections  — list of detected objects with arena-frame positions
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import supervision as sv
import numpy as np
import cv2

# Class IDs (must match your data.yaml order)
CLASS_PING_PONG = 0
CLASS_BEARING   = 1
CLASS_ROBOT     = 2

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # Load your trained model
        self.model = YOLO('/path/to/best.pt')  # ← UPDATE THIS PATH

        # Tracker — keeps IDs stable across frames so balls don't blink
        self.tracker = sv.ByteTrack()

        # Bridge converts between ROS Image messages and OpenCV arrays
        self.bridge = CvBridge()

        # Subscribe to camera
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publish detections
        self.pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

        # Camera-to-floor homography (computed once from calibration)
        # This converts pixel (x,y) → floor position (x,y) in robot frame (metres)
        # YOU MUST CALIBRATE THIS — see Part 4b below
        self.homography = None  # set after calibration

        self.get_logger().info('Detection node started')

    def image_callback(self, msg):
        # Convert ROS image to OpenCV BGR array
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model(frame, conf=0.45, verbose=False)[0]

        # Convert to supervision format for tracking
        detections = sv.Detections.from_ultralytics(results)

        # Update tracker — assigns stable IDs to each detection
        detections = self.tracker.update_with_detections(detections)

        # Build ROS message
        out = Detection2DArray()
        out.header = msg.header

        for i in range(len(detections)):
            d = Detection2D()
            box = detections.xyxy[i]         # [x1, y1, x2, y2] pixels
            class_id = int(detections.class_id[i])
            score = float(detections.confidence[i])
            track_id = int(detections.tracker_id[i]) if detections.tracker_id is not None else -1

            # Centre of bounding box in pixels
            cx = (box[0] + box[2]) / 2
            cy = (box[1] + box[3]) / 2

            # Bounding box size in the Detection2D message
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.size_x = float(box[2] - box[0])
            d.bbox.size_y = float(box[3] - box[1])

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = score
            d.results.append(hyp)

            # Store track_id in id field
            d.id = str(track_id)

            out.detections.append(d)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = DetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Register this node in your `setup.py`:
```python
entry_points={
    'console_scripts': [
        'detection_node = unibots_robot.detection_node:main',
    ],
},
```

## Part 4b — Camera Calibration and Floor Projection

**The problem:** YOLO tells you a ball is at pixel (320, 400). But Nav2 needs to know "the ball is 0.8 m ahead and 0.1 m to the right." You need to convert pixels → metres.

**The solution:** A **homography matrix** — a 3×3 matrix that maps image pixels to floor coordinates. You compute it once from known reference points.

```python
# CALIBRATION SCRIPT — run this once to compute homography
# Place 4 markers on the arena floor at KNOWN positions (e.g., tape crosses)
# Take a photo, click on each marker in the image, record its pixel coords

import cv2
import numpy as np

# Known floor positions of your 4 reference markers (metres, in robot frame)
# Example: markers at 0.5m ahead, ±0.3m left/right; 1.0m ahead, ±0.3m left/right
floor_points = np.float32([
    [0.5, -0.3],   # front-left marker
    [0.5,  0.3],   # front-right marker
    [1.0, -0.3],   # rear-left marker
    [1.0,  0.3],   # rear-right marker
])

# Pixel coordinates of those same markers (click on them in the image)
# Use cv2.imshow + mouse callback to get these
image_points = np.float32([
    [145, 380],   # ← replace with your actual click coordinates
    [495, 380],
    [220, 290],
    [420, 290],
])

# Compute the homography
H, _ = cv2.findHomography(image_points, floor_points)
print("Homography matrix:")
print(H)
# Save this matrix — paste it into detection_node.py as self.homography
np.save('homography.npy', H)
```

To use the homography in your detection node:
```python
def pixel_to_floor(self, px, py):
    """Convert pixel coords to robot-frame floor position (metres)"""
    pt = np.float32([[[px, py]]])
    result = cv2.perspectiveTransform(pt, self.homography)
    return result[0][0]  # [x_metres, y_metres] in robot frame
```

## ✅ Verify Detection Works

```bash
# Build and run the detection node
cd ~/unibots_ws && colcon build --symlink-install
ros2 run unibots_robot detection_node

# In another terminal, watch detections
ros2 topic echo /detections

# You should see Detection2DArray messages with bounding boxes
# when you wave orange balls in front of the camera
```

Also in RViz: add a **Camera** display and a **Detection2D** overlay to see boxes drawn on the video feed live.

---

# PART 4c — (Optional but Recommended) Hailo AI HAT

## 🧠 What it is

**Analogy:** Your Pi's main CPU is like a general-purpose chef who can cook anything but is slow at deep frying. The Hailo AI HAT is a specialist chef who does only deep frying but does it 10× faster. You hand the neural-network inference task to the Hailo chip so the CPU is free for everything else. Result: YOLO runs at ~30 FPS instead of ~3 FPS on CPU.

**Hardware:** The Hailo-8L (13 TOPS) fits over the Pi's M.2 slot. Cost: ~£60. Buy from the Raspberry Pi official store.

## The Catch: Model Conversion

The Hailo chip only runs models in its own format called **HEF** (Hailo Executable Format). You must convert your `.pt` → `.onnx` → `.hef`. The `.hef` conversion requires Hailo's **Dataflow Compiler (DFC)** which runs on x86 Linux (your laptop), not the Pi itself.

```bash
# Step 1: On your laptop — export YOLO model to ONNX
from ultralytics import YOLO
model = YOLO('runs/detect/unibots_detector/weights/best.pt')
model.export(format='onnx', imgsz=640, opset=11)
# Creates: runs/detect/unibots_detector/weights/best.onnx

# Step 2: Install Hailo Dataflow Compiler (requires a free Hailo account)
# Download from: https://hailo.ai/developer-zone/
# Install: pip install hailo_dataflow_compiler-*.whl

# Step 3: Convert ONNX → HEF (simplest path — use Hailo Model Zoo script)
# Follow: https://github.com/hailo-ai/hailo_model_zoo
# The key command (after setup):
hailomz compile yolov8n --ckpt best.onnx --hw-arch hailo8l --calib-path dataset/train/images/

# Step 4: Copy the .hef to your Pi
scp best.hef pi@raspberrypi.local:~/

# Step 5: On Pi — use hailo Python API instead of ultralytics
# Install: pip3 install hailo-platform (comes with Hailo SDK on Pi)
```

**If this conversion is too time-consuming before June 27:** First get the system working with CPU inference (slow but functional), then add Hailo as an upgrade. CPU at 5–8 FPS is enough to compete — the robot doesn't need to react faster than that.

---

# PART 5 — Where Am I? (AprilTags + EKF Localisation)

## 🧠 What it is

**Analogy 1 — AprilTags:** These are square black-and-white patterns on the arena walls (like QR codes but simpler). When your camera sees one, it can calculate its own position and orientation relative to that tag in full 3D — not just "I see it" but "I am 0.7 m in front of it, rotated 15 degrees to the right." Because the tags are at known positions on known walls (the rulebook tells you), this instantly tells the robot where it is in the arena.

**Analogy 2 — EKF (Extended Kalman Filter):** Imagine you are navigating blindfolded in a building. You count footsteps (that's wheel odometry — it drifts). You have a compass (that's the IMU — it's noisy). Every few seconds you hear a specific sound from a known location (that's an AprilTag sighting — accurate but infrequent). The EKF is a mathematician who sits in your ear and says: "Given all three imperfect inputs, the best estimate of your position right now is X=1.2m, Y=0.8m, heading=47°." It weights each source by how much it trusts it.

**Why you need both:** Wheel odometry alone drifts due to mecanum slip. IMU alone drifts in heading over seconds. AprilTags alone are only seen occasionally. Combined through EKF, you get smooth, accurate, continuous pose — and that fixes the strafe-curving problem because the EKF-corrected heading feeds Nav2's controller.

## AprilTags Setup

The rulebook specifies: **36h11 family**, IDs 0–23, 100×100mm, at the top of each wall. Before each match you're told your wall's IDs (North=0–5, East=6–11, South=12–17, West=18–23).

```bash
# Install apriltag_ros
sudo apt install ros-jazzy-apriltag-ros
```

Create `config/apriltag.yaml`:
```yaml
image_transport: raw
camera_frame: camera_link  # must match your URDF camera link name

# Tag family — must match the arena
tag_family: 36h11

# Detection parameters
max_hamming: 0   # 0 = only accept perfect tags, more robust
decimate: 2.0    # speed/accuracy tradeoff — 2.0 is good for Pi
blur: 0.0
refine_edges: 1
debug: 0

# Standalone tags — list ALL tags in the arena with their sizes
standalone_tags:
  - {id: 0,  size: 0.10}
  - {id: 1,  size: 0.10}
  - {id: 2,  size: 0.10}
  # ... repeat for ids 3–23
  # Quick way: use a YAML anchor or just list all 24
```

Create `launch/apriltag.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('unibots_robot')
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            remappings=[
                ('image_rect', '/camera/image_raw'),   # your camera topic
                ('camera_info', '/camera/camera_info'), # your camera info topic
            ],
            parameters=[os.path.join(pkg, 'config', 'apriltag.yaml')]
        )
    ])
```

## Camera Calibration (Required for AprilTags)

`apriltag_ros` needs the camera's intrinsic parameters (focal length, distortion) to compute accurate 3D pose. Print a chessboard calibration target and use the standard tool:

```bash
# Install camera calibration tool
sudo apt install ros-jazzy-camera-calibration

# Run calibration (with your camera publishing to /camera/image_raw)
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \       # number of inner corners on your chessboard
  --square 0.025 \   # square size in metres
  --ros-args \
  -r image:=/camera/image_raw \
  -r camera:=/camera

# Move the chessboard around in front of the camera
# Fill all four bars in the calibration GUI
# Click CALIBRATE, then SAVE
# Your calibration file is saved to /tmp/calibration.tar.gz
```

## EKF Setup (robot_localization)

The EKF fuses three sources:
1. Wheel odometry (from `mecanum_drive_controller`) — position + heading, drifts slowly
2. IMU (BNO055 or similar) — angular velocity + acceleration, very accurate short-term
3. AprilTag poses — absolute position, accurate but infrequent

Create `config/ekf.yaml`:
```yaml
ekf_filter_node:
  ros__parameters:
    # How often to publish (Hz)
    frequency: 30.0

    # Coordinate frames
    # 'map' = fixed arena frame (absolute position)
    # 'odom' = accumulated odometry (drifts)
    # 'base_link' = robot centre
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom   # start with odom, switch to map once AprilTags are visible

    # --- Source 1: Wheel Odometry ---
    odom0: /mecanum_drive_controller/odom
    odom0_config: [
      # x,    y,    z,     # position
      # roll, pitch, yaw,  # orientation
      # vx,   vy,   vz,    # linear velocity
      # vroll, vpitch, vyaw  # angular velocity
      true, true, false,   # x and y position from odometry (z always false for ground robot)
      false, false, false, # no orientation from odometry (IMU handles this)
      true, true, false,   # x and y velocity (useful)
      false, false, true   # yaw rate from odometry
    ]
    odom0_differential: false

    # --- Source 2: IMU ---
    imu0: /imu/data
    imu0_config: [
      false, false, false,  # no position from IMU
      true, true, true,     # roll, pitch, yaw from IMU (absolute orientation)
      false, false, false,
      true, true, true,     # angular velocity (gyro)
      true, false, false    # x linear acceleration (used to refine velocity)
    ]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true

    # --- Source 3: AprilTag (absolute pose corrections) ---
    pose0: /apriltag/pose    # published by your apriltag integration node
    pose0_config: [
      true, true, false,    # absolute x, y position
      false, false, true,   # absolute yaw
      false, false, false,
      false, false, false,
      false, false, false
    ]
    pose0_differential: false

    # Process noise — how much we trust the motion model between measurements
    # Increase if robot moves unpredictably (slip)
    process_noise_covariance: [
      0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0.05, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0.03, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0.06, 0,    0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0.025,0,    0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0.04, 0,    0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.02, 0,    0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,    0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.01, 0,
      0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0.015
    ]
```

Launch the EKF:
```python
# Add to your launch file:
from launch_ros.actions import Node

ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
    remappings=[('odometry/filtered', '/odometry/filtered')]
)
```

## IMU Node (BNO055)

```bash
# Install BNO055 driver
pip3 install adafruit-circuitpython-bno055

# Or use the ROS2 package:
sudo apt install ros-jazzy-bno055  # if available, otherwise pip install
```

Minimal IMU publisher (if no ROS package for your specific IMU):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board, adafruit_bno055
import numpy as np

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.timer = self.create_timer(0.02, self.publish)  # 50 Hz

    def publish(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        q = self.sensor.quaternion  # (w, x, y, z)
        if q and q[0] is not None:
            msg.orientation.w = q[0]
            msg.orientation.x = q[1]
            msg.orientation.y = q[2]
            msg.orientation.z = q[3]

        g = self.sensor.gyro  # rad/s
        if g:
            msg.angular_velocity.x = g[0]
            msg.angular_velocity.y = g[1]
            msg.angular_velocity.z = g[2]

        a = self.sensor.linear_acceleration  # m/s²
        if a:
            msg.linear_acceleration.x = a[0]
            msg.linear_acceleration.y = a[1]
            msg.linear_acceleration.z = a[2]

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(IMUNode())
    rclpy.shutdown()
```

## ✅ Verify Localisation Works

```bash
# Launch everything so far
ros2 launch unibots_robot robot.launch.py
ros2 launch unibots_robot apriltag.launch.py

# In RViz: add TF display, set fixed frame to 'map'
# Add Odometry display on /odometry/filtered
# Manually push the robot — you should see its estimated position move in RViz
# Hold an AprilTag in front of the camera — position should snap/correct
```

A good EKF is one where:
- Position drifts slowly when moving (not jumping)
- Corrects immediately when an AprilTag is seen
- Heading stays stable during pure translation (strafe)

---

# PART 6 — Getting Around (Nav2 Navigation)

## 🧠 What it is

**Analogy:** Nav2 is your robot's GPS + autopilot combined. You tell it "go to position (1.5, 0.8) in the arena." It:
1. Plans a safe path from your current position to the goal (avoiding known walls + seen obstacles)
2. Follows that path, continuously adjusting for drift
3. Replans if something gets in the way
4. Tells you when it has arrived

You never write the "how to drive there" code. You only write the "where to go" code (the behaviour tree, next section). Nav2 handles everything in between.

## The Arena Map

Because the arena is a known 2 m × 2 m box with fixed walls, you do not need SLAM (which builds a map as you go). You provide a **static map** — a pre-made image of the arena.

Create `maps/arena.pgm` — a 200×200 pixel greyscale image where:
- **White (255)** = free space (driveable)
- **Black (0)** = obstacle (walls)
- **Grey (127)** = unknown (not used here)

You can create this in any image editor. The arena is square: white inside, black 1-pixel border = walls.

Also create `maps/arena.yaml`:
```yaml
image: arena.pgm
resolution: 0.01   # 1 pixel = 1 cm (200px × 0.01 = 2m ✓)
origin: [0.0, 0.0, 0.0]   # map origin in world frame
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

## Nav2 Configuration

Nav2 has many parameters. Below is a simplified but complete config for your mecanum robot. Create `config/nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false
    # Not using AMCL — using EKF instead.

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered  # from EKF
    bt_loop_duration: 10            # ms between BT ticks
    default_server_timeout: 20
    # Path to your behaviour tree XML file (Part 7)
    default_nav_to_pose_bt_xml: "/home/pi/unibots_ws/src/unibots_robot/bt/main.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_is_path_valid_condition_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_recovery_node_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_reinitialize_global_localization_service_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0  # Hz
    # Using MPPI — handles holonomic robots, dynamic obstacles
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56          # how far ahead to plan (steps)
      model_dt: 0.05          # time per step (seconds)
      batch_size: 2000        # number of random trajectories sampled
      vx_std: 0.2             # randomness in forward velocity
      vy_std: 0.2             # randomness in lateral velocity (holonomic!)
      wz_std: 0.4             # randomness in rotation
      vx_max: 0.5             # max forward speed (m/s) — tune to your robot
      vx_min: -0.5
      vy_max: 0.5             # max strafe speed — set because you're holonomic
      wz_max: 1.0             # max rotation (rad/s)
      motion_model: "Omni"   # THIS IS CRITICAL for mecanum — not Diff
      visualize: true
      critics: ["GoalCritic", "GoalAngleCritic", "PathAlignCritic",
                "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic",
                "ObstaclesCritic"]
      ObstaclesCritic:
        enabled: true
        cost_power: 1
        repulsion_weight: 1.5
        critical_weight: 20.0
        consider_footprint: true
        collision_cost: 10000.0
        collision_margin_distance: 0.1  # 10cm safety bubble around obstacles
        near_goal_distance: 0.5

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.1   # goal tolerance in metres
      use_astar: true  # A* is good for small known arenas

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: odom
      robot_base_frame: base_link
      update_frequency: 20.0
      publish_frequency: 10.0
      width: 2.0    # covers entire arena
      height: 2.0
      resolution: 0.02   # 2 cm per cell
      rolling_window: false  # static for small arena
      # Robot footprint (rectangular, in metres — measure from Fusion)
      footprint: "[[0.10, 0.10], [0.10, -0.10], [-0.10, -0.10], [-0.10, 0.10]]"
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: camera_scan
        camera_scan:
          topic: /detections_as_obstacles   # your node converts detections → obstacles
          max_obstacle_height: 0.5
          clearing: true
          marking: true
          data_type: "LaserScan"  # or PointCloud2
          raytrace_max_range: 2.0
          obstacle_max_range: 2.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.15   # 15cm buffer around obstacles

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 5.0
      publish_frequency: 2.0
      width: 2.2   # slightly larger than arena
      height: 2.2
      resolution: 0.02
      footprint: "[[0.10, 0.10], [0.10, -0.10], [-0.10, -0.10], [-0.10, 0.10]]"
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: camera_scan
        camera_scan:
          topic: /detections_as_obstacles
          data_type: "LaserScan"
          max_obstacle_height: 0.5
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.15

recoveries_server:
  ros__parameters:
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries::Spin"
    backup:
      plugin: "nav2_recoveries::BackUp"
    wait:
      plugin: "nav2_recoveries::Wait"

map_server:
  ros__parameters:
    use_sim_time: false
    yaml_filename: "/home/pi/unibots_ws/src/unibots_robot/maps/arena.yaml"

lifecycle_manager:
  ros__parameters:
    use_sim_time: false
    autostart: true
    node_names:
      - map_server
      - controller_server
      - planner_server
      - recoveries_server
      - bt_navigator
```

## Converting Robot Detections to Obstacles

Other robots appear in your YOLO detections. Nav2's costmap needs them as an obstacle source. This node does that conversion:

```python
# obstacle_converter_node.py
# Reads /detections, filters for class 'robot', converts to LaserScan-like obstacle points
# so Nav2's costmap marks them as obstacles

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class ObstacleConverter(Node):
    def __init__(self):
        super().__init__('obstacle_converter')
        self.sub = self.create_subscription(
            Detection2DArray, '/detections', self.cb, 10)
        self.pub = self.create_publisher(
            LaserScan, '/detections_as_obstacles', 10)
        # You need the homography here too, to get floor positions

    def cb(self, msg):
        robot_detections = [
            d for d in msg.detections
            if d.results and d.results[0].hypothesis.class_id == '2'  # robot class
        ]
        if not robot_detections:
            return
        # Build a minimal LaserScan with one range per detected robot
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180  # 1 degree resolution
        scan.range_min = 0.1
        scan.range_max = 3.0
        ranges = [float('inf')] * 360
        for d in robot_detections:
            # floor position of robot centre (use homography)
            fx = d.bbox.center.position.x  # placeholder — use real homography
            fy = d.bbox.center.position.y
            dist = math.sqrt(fx**2 + fy**2)
            angle = math.atan2(fy, fx)
            idx = int(math.degrees(angle)) % 360
            ranges[idx] = min(ranges[idx], dist)
        scan.ranges = ranges
        self.pub.publish(scan)

def main():
    rclpy.init()
    rclpy.spin(ObstacleConverter())
    rclpy.shutdown()
```

## ✅ Verify Nav2 Works

```bash
# Launch Nav2 (with your map and params)
ros2 launch nav2_bringup bringup_launch.py \
  map:=/path/to/arena.yaml \
  params_file:=/path/to/nav2_params.yaml \
  use_sim_time:=false

# In RViz: use the "Nav2 Goal" button (green arrow) to click a target
# The robot should plan a path and drive to it
# Watch the costmap — walls should be red, free space should be grey
```

---

# PART 7 — The Brain (Behaviour Trees)

## 🧠 What it is

**Analogy:** Your old code was a state machine — like a flow diagram where you draw circles and arrows: "if in state SEARCH, do X; if in state APPROACH, do Y." That works but becomes a tangled mess when you add new behaviours, because every state needs to know about every other state.

A **Behaviour Tree (BT)** is different. It's a tree of reusable blocks that are evaluated top-down every "tick" (every 10–20 ms). The blocks are:

| Block type | What it does |
|---|---|
| **Sequence** (→) | Run children left-to-right. Stop and FAIL at first child that fails. Like AND. |
| **Fallback** (?) | Run children left-to-right. Stop and SUCCEED at first child that succeeds. Like OR. |
| **Condition** | Checks something. Returns SUCCESS or FAILURE instantly. |
| **Action** | Does something over time. Returns RUNNING while in progress, then SUCCESS or FAILURE. |
| **Decorator** | Wraps one child and modifies its behaviour (e.g., "retry 3 times", "invert result"). |

**Your robot's top-level logic in plain English:**

```
Every tick, ask: "Is it nearly the end of the match?"
  YES → go park on your wall, stop.
  NO  → ask: "Is the hopper full?"
        YES → go to your net and deposit.
        NO  → ask: "Can I see a ball?"
              YES → approach it and collect it.
              NO  → spin slowly to search.
```

That entire logic is a BT with ~15 nodes. It handles every competition situation, it's readable, and adding new behaviours is adding a new branch, not rewriting everything.

## BT XML Format (What Nav2 Uses)

Nav2 uses the BT.CPP library with XML files. Here is your robot's full behaviour tree:

Create `bt/main.xml`:
```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">

    <Fallback name="Root">

      <!-- PRIORITY 1: If last 20 seconds, go park and stop -->
      <Sequence name="EndGame">
        <TimeExpiredCondition seconds="160.0" />  <!-- 160s of 180s elapsed -->
        <NavigateToPose goal="{park_pose}" />
      </Sequence>

      <!-- PRIORITY 2: If hopper has balls, go deposit -->
      <Sequence name="DepositRun">
        <Condition ID="HopperHasBalls" />     <!-- your custom condition -->
        <NavigateToPose goal="{deposit_pose}" />
        <Action ID="LaunchBalls" />            <!-- your custom action -->
        <Action ID="ResetHopper" />
      </Sequence>

      <!-- PRIORITY 3: If we see a ball, collect it -->
      <Sequence name="CollectBall">
        <Condition ID="BallVisible" />         <!-- your custom condition -->
        <Action ID="SetBallGoal" />            <!-- converts detection to a goal pose -->
        <NavigateToPose goal="{ball_goal}" />
        <Condition ID="BallCaptured" />        <!-- break beam fired? -->
      </Sequence>

      <!-- PRIORITY 4: Search by rotating -->
      <Action ID="SearchSpin" />

    </Fallback>

  </BehaviorTree>
</root>
```

## Writing Custom BT Nodes

The BT nodes labelled as `Condition ID="..."` and `Action ID="..."` with custom names are your code. You register them as Nav2 BT plugins.

The simplest approach for your timeline: write a **single master node** that runs the BT logic in Python and publishes navigation goals via the Nav2 action interface. This avoids the C++ plugin boilerplate while keeping the same logical structure.

Create `unibots_robot/brain_node.py`:

```python
#!/usr/bin/env python3
"""
Brain Node — the robot's decision maker.
Runs a simple behaviour tree every 100ms.
Reads: /detections, /break_beam, /match_timer, /odometry/filtered
Writes: navigation goals to Nav2 action server, trigger commands to intake/launcher
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool, Int32
import time, math

# State machine states (the BT effectively implements these)
SEARCH   = 'SEARCH'
APPROACH = 'APPROACH'
DEPOSIT  = 'DEPOSIT'
PARK     = 'PARK'

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        # --- State ---
        self.state = SEARCH
        self.match_start_time = time.time()
        self.balls_held = 0          # from break beam count
        self.current_target = None   # (x, y) in map frame
        self.ball_captured = False   # from break beam

        # Your wall's deposit pose (set from AprilTag pre-match)
        # This is the position IN FRONT OF your net, facing the wall
        # You will update this from the AprilTag pose before the match
        self.deposit_pose = (0.1, 1.0, 0.0)  # (x, y, yaw) — PLACEHOLDER
        self.park_pose    = (0.05, 1.0, 0.0) # right against the wall

        # --- Subscribers ---
        self.create_subscription(
            Detection2DArray, '/detections', self.on_detections, 10)
        self.create_subscription(
            Bool, '/break_beam', self.on_break_beam, 10)

        # --- Publishers ---
        self.launch_pub = self.create_publisher(Bool, '/launcher/fire', 10)
        self.intake_pub = self.create_publisher(Bool, '/intake/enable', 10)

        # --- Nav2 Action Client ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        # --- Main BT loop ---
        self.create_timer(0.1, self.tick)  # 10 Hz decision rate

        # Enable intake immediately
        self.intake_pub.publish(Bool(data=True))
        self.get_logger().info('Brain node started')

    def elapsed(self):
        return time.time() - self.match_start_time

    def on_detections(self, msg):
        """Pick the best ping-pong ball target."""
        balls = [
            d for d in msg.detections
            if d.results and d.results[0].hypothesis.class_id == '0'
               and float(d.results[0].hypothesis.score) > 0.5
        ]
        if balls:
            # Pick closest ball (largest bounding box = closest)
            best = max(balls, key=lambda d: d.bbox.size_x * d.bbox.size_y)
            # Convert pixel centre to floor position
            # (simplified — use your homography in practice)
            cx = best.bbox.center.position.x
            cy = best.bbox.center.position.y
            self.current_target = (cx, cy)
        else:
            self.current_target = None

    def on_break_beam(self, msg):
        """A ball entered the hopper."""
        if msg.data:
            self.balls_held += 1
            self.ball_captured = True
            self.get_logger().info(f'Ball captured! Total: {self.balls_held}')

    def send_nav_goal(self, x, y, yaw=0.0):
        """Send a navigation goal to Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Convert yaw to quaternion (rotation around Z axis)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self.nav_client.send_goal_async(goal)

    def tick(self):
        """The behaviour tree — called every 100ms."""
        t = self.elapsed()

        # PRIORITY 1: End of match — park
        if t >= 160.0:
            if self.state != PARK:
                self.state = PARK
                self.get_logger().info('End of match — parking')
                px, py, pyaw = self.park_pose
                self.send_nav_goal(px, py, pyaw)
            return

        # PRIORITY 2: Hopper full (4 balls) OR been collecting for 60s → deposit
        if self.balls_held >= 4 or (self.balls_held > 0 and t - self.last_deposit_time > 60):
            if self.state != DEPOSIT:
                self.state = DEPOSIT
                dx, dy, dyaw = self.deposit_pose
                self.send_nav_goal(dx, dy, dyaw)
                # After arriving (TODO: check arrival), fire launcher
                # For now, fire after 3 seconds travel time (replace with arrival callback)
                self.create_timer(3.0, self.do_deposit)
            return

        # PRIORITY 3: Ball visible — go get it
        if self.current_target is not None:
            if self.state != APPROACH:
                self.state = APPROACH
                tx, ty = self.current_target
                self.send_nav_goal(tx, ty)
            return

        # PRIORITY 4: Search — spin in place
        if self.state != SEARCH:
            self.state = SEARCH
            self.get_logger().info('Searching...')
            # Spin in place: publish rotate command directly
            # (Nav2 Spin recovery behaviour can be used instead)

    def do_deposit(self):
        """Fire the launcher."""
        self.launch_pub.publish(Bool(data=True))
        self.balls_held = 0
        self.ball_captured = False
        self.last_deposit_time = self.elapsed()
        self.state = SEARCH

def main():
    rclpy.init()
    node = BrainNode()
    node.last_deposit_time = 0.0
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> **Note:** The `NavigateToPose` goal above uses simplified pixel-to-floor mapping. Replace it with the proper homography + coordinate frame transform. The brain node's `current_target` should be in the `map` frame, which requires your EKF pose + the floor projection.

## Coordinate Frame Conversion (Map Frame)

When you detect a ball, it's in pixel coords. Nav2 wants goals in `map` frame (absolute arena coordinates). The chain is:

```
Pixel (px, py)
  → Homography → Robot frame (forward=x, left=y, in metres)
  → TF transform (robot pose in map) → Map frame (x, y in metres from arena origin)
```

```python
import tf2_ros, tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class BrainNode(Node):
    def __init__(self):
        ...
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def robot_to_map(self, rx, ry):
        """Convert robot-frame point to map-frame point."""
        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = rx
        pt.point.y = ry
        pt.point.z = 0.0
        try:
            map_pt = self.tf_buffer.transform(pt, 'map')
            return map_pt.point.x, map_pt.point.y
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            return None, None
```

---

# PART 8 — Hardware Integration

## The Break Beam Sensor

The IR break beam has two sides: emitter (sends an IR beam) and receiver (detects it). When a ball passes through, the beam is interrupted → GPIO pin goes LOW. This is your reliable "ball is here" signal.

```python
#!/usr/bin/env python3
# break_beam_node.py — reads GPIO break beam and publishes events
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

BREAK_BEAM_PIN = 17  # ← change to your actual GPIO pin number

class BreakBeamNode(Node):
    def __init__(self):
        super().__init__('break_beam_node')
        self.pub = self.create_publisher(Bool, '/break_beam', 10)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BREAK_BEAM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Edge detection: fire callback when beam is broken (FALLING = HIGH→LOW)
        GPIO.add_event_detect(
            BREAK_BEAM_PIN,
            GPIO.FALLING,
            callback=self.beam_broken,
            bouncetime=200  # ignore bounces within 200ms
        )
        self.get_logger().info(f'Break beam on GPIO {BREAK_BEAM_PIN} ready')

    def beam_broken(self, channel):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info('BALL CAPTURED')

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    rclpy.spin(BreakBeamNode())
    rclpy.shutdown()
```

## Launcher and Intake GPIO

```python
# actuator_node.py — controls intake roller and ball launcher via GPIO/PWM
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

INTAKE_MOTOR_PIN = 23    # GPIO pin to enable intake roller
LAUNCHER_PIN     = 24    # GPIO pin to trigger launcher servo/motor

class ActuatorNode(Node):
    def __init__(self):
        super().__init__('actuator_node')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(INTAKE_MOTOR_PIN, GPIO.OUT)
        GPIO.setup(LAUNCHER_PIN, GPIO.OUT)
        GPIO.output(INTAKE_MOTOR_PIN, GPIO.LOW)
        GPIO.output(LAUNCHER_PIN, GPIO.LOW)

        self.create_subscription(Bool, '/intake/enable', self.on_intake, 10)
        self.create_subscription(Bool, '/launcher/fire', self.on_launch, 10)

    def on_intake(self, msg):
        GPIO.output(INTAKE_MOTOR_PIN, GPIO.HIGH if msg.data else GPIO.LOW)

    def on_launch(self, msg):
        if msg.data:
            # Fire sequence: extend, hold, retract
            GPIO.output(LAUNCHER_PIN, GPIO.HIGH)
            # In practice, use a timer callback instead of sleep
            import time; time.sleep(0.5)
            GPIO.output(LAUNCHER_PIN, GPIO.LOW)

def main():
    rclpy.init()
    rclpy.spin(ActuatorNode())
    rclpy.shutdown()
```

## The Magnetic Skirt (Hardware Only — Zero Software)

Attach a ring of neodymium magnets (grade N42 or stronger, disc shape ~20mm diameter) around the base skirt of your robot, low to the ground. The 20mm steel bearings will attract to them as you drive nearby. No GPIO. No node. No code. Just physics working for you, scoring 1 point per stuck bearing while your software focuses on orange balls.

**Magnet placement tip:** Mount them around the perimeter but slightly recessed so they don't scrape the foam mat. Test that the magnet pull is strong enough to grab a rolling bearing at 5 cm distance.

---

# PART 9 — The Physical Start Button

The rulebook requires a **physical start button** on the robot. Pressing it starts the match code. Here is how to wire that into your software:

```python
# start_node.py — waits for physical button press, then starts the match
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

START_BUTTON_PIN = 27  # GPIO pin connected to physical button

class StartNode(Node):
    def __init__(self):
        super().__init__('start_node')
        self.pub = self.create_publisher(Bool, '/match/started', 10)
        self.started = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            START_BUTTON_PIN,
            GPIO.FALLING,
            callback=self.button_pressed,
            bouncetime=500
        )
        self.get_logger().info('Waiting for start button...')

    def button_pressed(self, channel):
        if not self.started:
            self.started = True
            self.pub.publish(Bool(data=True))
            self.get_logger().info('MATCH STARTED')

def main():
    rclpy.init()
    rclpy.spin(StartNode())
    rclpy.shutdown()
```

Your `brain_node.py` should subscribe to `/match/started` and only begin ticking when it receives `True`:

```python
self.match_running = False
self.create_subscription(Bool, '/match/started', self.on_start, 10)

def on_start(self, msg):
    if msg.data:
        self.match_running = True
        self.match_start_time = time.time()
        self.get_logger().info('Match timer started!')

def tick(self):
    if not self.match_running:
        return  # do nothing until button pressed
    # ... rest of BT logic
```

---

# PART 10 — Bringing It All Together (Master Launch File)

Create `launch/competition.launch.py` — this starts everything:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('unibots_robot')
    nav2_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # 1. Robot hardware and control
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'robot.launch.py'))),

        # 2. IMU
        Node(package='unibots_robot', executable='imu_node'),

        # 3. Camera + Detection
        Node(package='unibots_robot', executable='detection_node'),
        Node(package='unibots_robot', executable='obstacle_converter'),

        # 4. AprilTag detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'apriltag.launch.py'))),

        # 5. EKF localisation
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]
        ),

        # 6. Nav2 (delayed 3s to let everything else start first)
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_dir, 'launch', 'bringup_launch.py')),
                    launch_arguments={
                        'map': os.path.join(pkg, 'maps', 'arena.yaml'),
                        'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                        'use_sim_time': 'false',
                    }.items()
                ),
            ]
        ),

        # 7. Sensors and hardware I/O
        Node(package='unibots_robot', executable='break_beam_node'),
        Node(package='unibots_robot', executable='actuator_node'),
        Node(package='unibots_robot', executable='start_node'),

        # 8. Brain (delayed 5s to let Nav2 start)
        TimerAction(
            period=5.0,
            actions=[
                Node(package='unibots_robot', executable='brain_node')
            ]
        ),
    ])
```

---

# PART 11 — Build Timeline (to June 27)

Work vertically — always have a robot that competes, even if it's limited. Each milestone is a working robot.

## Week 1 (June 10–15): Infrastructure

| Day | Goal | Done when |
|---|---|---|
| 1 | ROS 2 workspace compiles. `colcon build` passes. Teleop works in Gazebo | Robot moves with keyboard |
| 2 | URDF in Gazebo matches real robot dimensions (verify with Fusion). ros2_control wheels spin | Wheels move in sim |
| 3 | Camera node publishing. Detection node loads YOLO (even a pretrained model) | `/detections` topic has data |
| 4 | Collect 200+ labelled arena images. Submit to Roboflow. Train starts | Training run started |
| 5 | IMU node publishing. EKF running. `/odometry/filtered` is smooth | See robot drift + correct in RViz |

## Week 2 (June 16–22): Capability

| Day | Goal | Done when |
|---|---|---|
| 6 | Download trained model. Detection works on orange balls live | Boxes on screen over real balls |
| 7 | AprilTags detected. EKF corrects from tag sightings | Position snaps on tag sighting |
| 8 | Nav2 bringup. "Drive to coordinate" works | Robot navigates to 2D goal in arena |
| 9 | Brain node: SEARCH → APPROACH loop working | Robot drives to detected ball |
| 10 | Break beam wired. Ball collection confirmed. Counter works | `balls_held` increments on capture |

## Week 3 (June 23–27): Integration + Competition

| Day | Goal | Done when |
|---|---|---|
| 11 | Deposit sequence: robot navigates to wall, fires launcher | Ball lands in net |
| 12 | Parking: last 20s, robot touches wall | Parks reliably |
| 13 | Full match dry run (3 min timer, all states) | Completes without crash |
| 14 | Tune: Nav2 speed params, detection confidence threshold, heading hold | Smoother, faster |
| 15 | Pre-competition buffer. Fix everything that broke | Ready |

---

# PART 12 — Competition Day Checklist

## Before the Match (When They Tell You Your Wall)

```bash
# You are told your zone (e.g., "South wall, Purple, tags 12–17")
# Update your config:
nano ~/unibots_ws/src/unibots_robot/config/match_config.yaml
```

```yaml
# match_config.yaml — update before EACH match
my_wall: south         # north / south / east / west
my_color: purple       # yellow / orange / purple / green
my_apriltag_ids: [12, 13, 14, 15, 16, 17]  # the 6 tags on your wall
# Deposit position (in front of your net, in map frame)
deposit_x: 1.0
deposit_y: 0.1
deposit_yaw: 3.14  # facing south wall
park_x: 1.0
park_y: 0.05
```

## Starting the Robot

```bash
# Make sure all code is built
cd ~/unibots_ws && colcon build --symlink-install && source install/setup.bash

# Launch everything
ros2 launch unibots_robot competition.launch.py

# Check everything is up:
ros2 node list   # should show all your nodes
ros2 topic list  # should show /detections, /odometry/filtered, /tf, etc.

# When the judge says go: press the physical button on the robot
```

## If Something Goes Wrong During a Match

- A collision reset → a judge will hand the robot to you, you **press the physical start button** to restart. No SSH allowed.
- Your code crashes → it will restart because you should configure a **watchdog** (see below)

```bash
# Simple systemd watchdog (optional but good practice)
# Creates a service that auto-restarts your launch file if it crashes
# /etc/systemd/system/unibots.service

[Unit]
Description=Unibots Robot
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/unibots_ws
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch unibots_robot competition.launch.py"
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable unibots
sudo systemctl start unibots
# Now the robot auto-starts on boot and restarts if it crashes
```

---

# PART 13 — Debugging Tools You Will Use Every Day

```bash
# See all active topics
ros2 topic list

# See what's on a topic (live)
ros2 topic echo /detections

# How fast is a topic publishing?
ros2 topic hz /detections

# Is a node running?
ros2 node list

# See a node's parameters
ros2 param list /brain_node

# Change a parameter live (no restart needed)
ros2 param set /brain_node some_param 0.5

# Record a bag of all topics (for later analysis)
ros2 bag record -a -o my_test_run

# Replay the bag
ros2 bag play my_test_run

# RViz — your main visualisation tool
rviz2
# Add displays: TF, Map, Odometry, Camera, MarkerArray for detections

# View the behaviour tree live in Groot2
# Install: https://www.behaviortree.dev/groot/
# Run alongside your BT node to see it tick in real time
```

---

# PART 14 — Package Install Reference

```bash
# All ROS 2 packages you need
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-core \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-robot-localization \
  ros-jazzy-apriltag-ros \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-twist-mux \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-image-transport \
  ros-jazzy-camera-calibration \
  ros-jazzy-tf2-tools \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs

# Python packages
pip3 install \
  ultralytics \
  supervision \
  opencv-python \
  adafruit-circuitpython-bno055 \
  RPi.GPIO \
  numpy
```

---

# PART 15 — Key Things That Will Trip You Up

**1. TF tree must be complete.**
ROS 2 uses a tree of coordinate frames. If any frame is missing or broken, almost everything fails silently. Check with:
```bash
ros2 run tf2_tools view_frames  # generates a PDF of your TF tree
```
Your tree must be: `map → odom → base_link → camera_link`, `base_link → imu_link`, `base_link → front_left_wheel`, etc.

**2. Time synchronisation.**
All nodes must use the same clock. If you see TF errors like "Lookup would require extrapolation into the past," your clocks are out of sync. Set `use_sim_time: false` everywhere consistently. On the real robot, make sure the Pi's system clock is set correctly (use `timedatectl`).

**3. Nav2 lifecycle nodes.**
Nav2 nodes are lifecycle-managed — they must be activated before they do anything. The `lifecycle_manager` does this automatically if `autostart: true` is set. If Nav2 seems to do nothing, check lifecycle state:
```bash
ros2 lifecycle get /controller_server
# Should say 'active'. If 'inactive' or 'unconfigured', the lifecycle_manager hasn't started.
```

**4. The mecanum controller needs all 4 wheels.**
If any wheel joint is missing from the URDF, the controller won't activate. Check:
```bash
ros2 control list_controllers
# mecanum_drive_controller should show 'active'
```

**5. Confidence threshold tuning.**
Your YOLO model will occasionally make false detections (e.g., seeing a "ball" in the wall texture). If the robot keeps driving to walls, raise the confidence threshold in `detection_node.py` from `0.45` to `0.6` or higher. Balance this against missing real balls.

**6. The deposit pose changes every match.**
You are assigned a different wall each match. You MUST update `match_config.yaml` before each match. Set a 5-minute pre-match ritual: hear wall → update config → rebuild (or use `ros2 param set` to change it live without rebuilding).

**7. Strafe still curves until EKF is tuned.**
The mecanum slip-curve is corrected by the EKF providing accurate heading to Nav2's MPPI controller. If it still curves, the EKF heading is wrong → check IMU is publishing, check the `imu0_config` settings, check TF between `imu_link` and `base_link` is correct.

---

# Summary: The One-Page Version

| Layer | Package | Your file | What it does |
|---|---|---|---|
| Motors | `ros2_controllers` (mecanum_drive_controller) | `controllers.yaml` | Spins wheels from velocity commands |
| Sensing | `cv_bridge`, `apriltag_ros`, `RPi.GPIO` | `detection_node.py`, `imu_node.py`, `break_beam_node.py` | Sees balls, tags, IMU, beam breaks |
| Localise | `robot_localization` (EKF) | `ekf.yaml` | "Where am I?" — fuses all sensors |
| Navigate | `nav2_bringup` (Nav2 + MPPI) | `nav2_params.yaml` | "How do I get there?" |
| Decide | Custom Python | `brain_node.py` | "What should I do?" — the BT |
| Act | Custom Python | `actuator_node.py` | Fires launcher, runs intake |
| Score | Magnets | None | Passive bearing collection |

**The order everything connects:**

```
Camera → detection_node → /detections → brain_node → nav goal → Nav2
IMU → ekf_node → /odometry/filtered → Nav2
AprilTags → ekf_node (pose correction) → accurate heading → straight strafe
Nav2 → /mecanum_drive_controller/cmd_vel → motors → robot moves
Break beam → brain_node → "ball captured" → go deposit
Brain says "deposit" → actuator_node → launcher fires → ball in net → 4 points
Magnets → bearings stick → 1 point each, no code
End of match → brain says "park" → Nav2 → touches wall → 3 points
```
