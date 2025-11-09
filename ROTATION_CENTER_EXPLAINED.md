# Rotation Center Diagnostic - Understanding the Issue

## The Problem: Map Overlaps During Turns

When your robot turns, the SLAM map distorts/overlaps. This happens when:

1. **RViz shows correct motion** (based on odometry published by diff_drive_controller)
2. **Gazebo shows different motion** (physics simulation)
3. **Or both are wrong** (rotation center misaligned with base_footprint)

## The Root Cause

For a differential drive robot, the **rotation center** (Instantaneous Center of Rotation, ICR) must be at the **base_footprint** frame.

If the actual rotation center in Gazebo is NOT at base_footprint:
- Odometry calculations will be wrong
- TF transforms will be wrong
- SLAM will build incorrect maps during turns

## What We're Testing

```
Gazebo Robot Motion During Pure Rotation:
=========================================

Ideal (base_fp at ICR):
    ↑ north
    |
    |  base_footprint (ICR)
    |        ●  ← Should stay fixed OR rotate about itself
    |       /|\
    |        |
    |  ←←←←←●→→→→→ Wheels rotating
    
    TF chain:
    odom → base_footprint → base_final-v3 → wheels
                   ↑
            Should be rotation center

Problematic (base_fp NOT at ICR):
    
    ↑ north
    |
    |  odom frame         actual ICR in odom
    |       |                    ●← Gazebo rotating about HERE
    |       |              /     
    |    base_fp ●    /
    |       |    /
    |  ←←←← ●→→→ Wheels
    
    TF chain incorrect:
    odom → base_footprint(WRONG origin)
                ↓
           base_final-v3 (but offset is wrong)
```

## How the Detector Works

1. **Records positions** of base_footprint, base_body, wheel1 during pure rotation
2. **Calculates ICR** for each frame using least-squares circle fitting
3. **Compares ICRs**:
   - If all the same → Great! Transforms are correct
   - If different → Problem! Frames are misaligned
4. **Reports offset** needed to align base_footprint with true ICR

## What the Output Means

### Scenario A: Everything is correct
```
base_footprint ICR: (0.0, 0.0) m, radius: 0.25 m
base_final-v3 ICR:  (0.0, 0.0) m, radius: 0.25 m
wheel1 ICR:         (0.0, 0.0) m, radius: 0.30 m

✓ All frames have same center → transforms are correct!
```

### Scenario B: base_footprint is offset
```
base_footprint ICR: (0.15, 0.05) m, radius: 0.25 m
base_final-v3 ICR:  (0.10, 0.02) m, radius: 0.25 m

⚠ PROBLEM: Different frames have different rotation centers!

SUGGESTION:
Update base_to_robot joint origin:
  From: xyz = (-0.36, 0.0, -0.15)
  To:   xyz = (-0.35, -0.03, -0.15)  ← Adjust X and Y

(The script will calculate exact values)
```

## Understanding the URDF Transform

Current in your URDF:
```xml
<joint name="base_to_robot" type="fixed">
    <origin xyz="-0.36 0.0 -0.15" rpy="1.57 0.0 1.57" />
    <parent link="base_footprint" />
    <child link="base_final-v3" />
</joint>
```

This means:
```
base_footprint ---xyz=(-0.36, 0.0, -0.15)---> base_final-v3
       ↑
   Rotation center (should be here)

       
base_final-v3 (actual robot body, where wheels are attached)
       ↑
   Wheels rotate around this link's origin
```

For pure rotation to work correctly:
- base_footprint must be at the geometric center where wheels pivot
- Currently it's at (-0.36, 0.0, -0.15) offset from base_final-v3 origin
- If Gazebo rotates about a different point, this offset is wrong

## How to Fix

### Step 1: Run the detector
```bash
cd /home/sanjay/ros2_workspaces/rcup_migration/src/rcup_garden/
python3 scripts/detect_rotation_center.py
```

Or automatically start Gazebo + run test:
```bash
bash scripts/run_rotation_detector.sh
```

### Step 2: Read the recommendations
The script will output:
```
RECOMMENDATIONS:
To fix the rotation center alignment:

Update base_to_robot joint origin in URDF:
Change xyz from (-0.36, 0.0, -0.15) to
(-0.35, 0.02, -0.15)

And update urdf/ArmPlate.urdf.xacro line ~310
```

### Step 3: Update URDF and retest
Edit `urdf/ArmPlate.urdf.xacro` around line 310:
```xml
<joint name="base_to_robot" type="fixed">
    <origin xyz="NEW_X NEW_Y -0.15" rpy="1.57 0.0 1.57" />
    ...
</joint>
```

### Step 4: Verify
```bash
ros2 launch rcup_garden gazebo_diff.launch.py
# Spin robot and verify RViz and Gazebo motion now match
```

## Key Insights

1. **wheel_separation and wheel_radius** affect forward/turn speed ratio
2. **base_footprint position** affects rotation center location
3. **Both must be correct** for proper odometry and SLAM

The detective found your wheel params were wrong → we fixed that.
Now we need to verify rotation center is at the right place!

## Differential Drive Physics

For a differential drive robot:

```
Left wheel velocity:  v_L
Right wheel velocity: v_R
Wheel separation:     L = 0.49 m (distance between wheels)
Wheel radius:         r = 0.075 m

Linear velocity:  v = (v_L + v_R) * r / 2
Angular velocity: ω = (v_R - v_L) * r / L

ICR (Instantaneous Center of Rotation):
  Distance from center: R = v / ω = (v_L + v_R) * L / (2 * (v_R - v_L))
  
Pure rotation (v_L = -v_R):
  R = 0 (rotation about center) ✓
  
Pure forward (v_L = v_R):
  R = ∞ (straight line) ✓
```

If base_footprint is not at center, all these calculations fail!
