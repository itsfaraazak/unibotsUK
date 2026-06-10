# REAL vs HALLUCINATED — Unibots XACRO breakdown

## ✅ 100% REAL (from your STL files)

### Structural parts (exact STL positions & dimensions)
- **bottom_plate** (150×175×5 mm)
- **top_plate** (190×190×5 mm)
- **4 columns** (extruded & non-extruded pairs)
- **3 middle supports** (vertical pillars)
- **4 motor supports** (FR, FL, BR, BL)
- **Storage boxes** (Main, Battery, Additional)
- **Ramp** (ball collection funnel)
- **Platform** (tilted sensor mount)
- **Claw** (gripper)
- **Servo holder**
- **Handle**

**Evidence:** All extracted from STL bounding box analysis. Positions computed from Fusion mesh centres (see STL parse output earlier).

---

## ❌ COMPLETELY HALLUCINATED (not in your files)

### Wheels
- **Mecanum wheels** (60 mm diameter, 25 mm width)
  - NOT in Fusion 360 at all
  - Added as cylinder primitives
  - You said "60mm maybe" — I used exactly that
  - **Real wheels on your actual robot** = yellow spindle wheels in the photo
  
### Sensors
- **Camera link** on the platform
  - No camera in Fusion
  - Added for ROS2 tf tree
  - Would need real camera model/mounting bracket
  
- **IMU link**
  - No IMU in Fusion
  - Just a tf frame for odometry
  - Real sensor TBD

### Electronics
- **Battery mass estimates** (0.2 kg guessed)
  - Battery_Storage box exists in Fusion
  - Actual battery pack NOT modelled
  
- **Motor/servo specs**
  - Motor mounts exist (4 bracket pieces)
  - Actual motors (gearboxes, shafts) NOT in Fusion
  - Not modelled at all

### Drive plugin
- **libgazebo_ros_planar_move** 
  - Pure software, not in Fusion
  - Mecanum controller for Gazebo
  - You'll need the actual motor controllers + ROS2 nodes

### Arena
- **Full 2m×2m Gazebo world**
  - Per Unibots rulebook (Section 4)
  - Walls, nets, balls, bearings
  - Entire thing fabricated from rules PDF
  - NOT from your files

### Inertia values
- **All link masses** (e.g., "0.040 kg" for columns)
  - Guessed based on part geometry
  - Should be **measured from your actual robot** or computed from STL + material density
  - Currently very rough estimates

### Link names & hierarchy
- **Fixed vs revolute joints**
  - All joints set to `fixed` except wheels (continuous)
  - Claw is fixed — you'll want to make it **revolute** if it actually moves
  - Motor mounts are fixed (motors themselves not modelled)

---

## ⚠️ UNCERTAIN / NEEDS VERIFICATION

| Item | Status | Notes |
|---|---|---|
| Wheel positions | **Estimated** | 120mm wheelbase, 150mm track — from the photo & Unibots rules |
| Wheel diameter | **User-confirmed** | You said "60mm maybe" |
| Chassis height (200mm rule) | **Calculated** | Fits 200mm cube per rulebook |
| Mass of parts | **All guessed** | Use STL + material density for real values |
| Coordinate frame orientation | **Assumed** | +X toward ramp, +Y left, +Z up |
| Camera/IMU mounting | **Arbitrary** | Placed on platform — actual hardware TBD |
| Motor control | **Not modelled** | Wheels use gazebo_ros_planar_move plugin (ignores motor models) |

---

## 🔧 WHAT YOU NEED TO FIX

1. **Replace inertia guesses with real masses**
   ```xml
   <xacro:box_inertia m="0.040" .../>  ← Change "0.040" to measured kg
   ```
   Measure each 3D-printed part on a scale.

2. **Add actual motor models**
   - Motor mounts exist, but motors themselves are missing
   - You'll need URDF for motor body + gearbox
   - Wheels will rotate based on `/unibots/cmd_vel` (planar_move does kinematics)

3. **Model the claw as revolute if it's actuated**
   ```xml
   <!-- Change from: -->
   <joint name="claw_joint" type="fixed">
   
   <!-- To: -->
   <joint name="claw_joint" type="revolute">
     <axis xyz="0 1 0"/>
     <limit lower="-1.57" upper="0" effort="1.0" velocity="1.0"/>
   </joint>
   ```

4. **Replace sensor frames with real hardware**
   - Camera: link real Raspberry Pi camera or Jetson module
   - IMU: specify actual sensor model (MPU9250, BNO055, etc.)

5. **Add acrylic chassis if needed**
   - The Fusion model has the structural frame only
   - Real robot uses clear acrylic laser-cut base (not in Fusion)
   - Add as simple box or import acrylic CAD if you have it

6. **Verify wheel mecanum roller angles**
   - Currently ignored (gazebo_ros_planar_move doesn't simulate roller geometry)
   - If you need accurate mecanum slip/friction, use custom plugin

---

## WHAT'S PRODUCTION-READY

✅ The **structure** (all 3D-printed parts with real geometry)
✅ The **STL meshes** (visual fidelity for RViz/Gazebo)
✅ The **XACRO macro system** (easy to edit, no repetition)
✅ The **ROS2 launch files** (ready to `colcon build && launch`)
✅ The **Gazebo arena world** (per rulebook specs)

❌ **Still to do:** Real sensor models, motor dynamics, acrylic chassis, actual mass measurements

