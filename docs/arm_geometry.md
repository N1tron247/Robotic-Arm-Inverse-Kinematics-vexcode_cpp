# Arm Geometry

## Overview

The robotic arm is modeled as a **three jointed manipulator** consisting of a rotating base and a two-link planar arm. The arm operates in 3D space, but the inverse kinematics problem can be reduced to 2D planar geometry after accounting for base rotation.

The arm contains the following joints:

1. **Waist (Yaw)**
2. **Shoulder (Pitch)**
3. **Elbow (Pitch)**

A claw or gripper may be attached to the manipulator of the arm, but it does not affect the kinematics used to position the arm.

---

# Arm Link Structure

The arm is composed of two rigid links connected by revolute joints.

| Arm Link | Description |
|------|-------------|
| **Upper Arm (L1)** | Distance from shoulder joint to elbow joint |
| **Forearm (L2)** | Distance from elbow joint to end effector |

These link lengths are constant mechanical parameters of the robot.

Example variables used in code(C++):

```cpp
double upper_arm_length = L1;
double forearm_length = L2;
```













## Arm Links

- **Upper arm (L1)**: distance from shoulder joint to elbow joint
- **Forearm (L2)**: distance from elbow joint to end of the manipulator

## Arm Joints

1. **Waist (Yaw)**
   - Rotates the arm about the vertical(Z) axis
   - Determines the direction of the arm in the XY plane

2. **Shoulder (Pitch)**
   - Raises and lowers the upper arm

3. **Elbow (Pitch)**
   - Extends or retracts the forearm

## Workspace/Robot Work Envelope

   - The reachable workspace that the arm's reach is bounded by, computed as: |L1-L2|

## Joint Definitions

**Waist Joint**
  - The Waist joint rotates long the Z axis
  - Its goal is to align the arm with the target point in the horizontal plane
  - The needed rotation is calculated by:
    ```cpp
    waist_angle = atan2(y, x)
    ```
**Shoulder Joint**
  - After the waist rotates to the correct heading towards the target, the shoulder determines the vertical(Z) height needed in order to reach the vertical position of the target
  - In order to calculate the angle for the shoulder planar distances must be defined:
    ```cpp
    double r = sqrt(pow(x, 2) + pow(y, 2)); // horizontal distance from the base axis to the target
    double z_s = z - shoulder_height; // vertical distance between the shoulder joint and the target
    double d = sqrt(pow(r, 2) + pow(z_s, 2)); // straight distance between shoulder joint and target
    ```
  - Next, define angle from the shoulder to the target
    ```cpp
    double alpha = atan(z_s, r);
    ```
  - Use the law of cosines to calculate:
    ```cpp
    double beta = acos( ((pow(L1, 2) + pow(d, 2) - pow(L2, 2))) / (2 * L1 * d) );
    ```
  - So the shoulder angle is
    ```cpp
    double shoulder_angle = alpha - beta;
    ```
  - This shoulder movement allows the forearm to be in radial range with the target point

**Elbow Joint**
   - The elbow joint controls the angle between the upper arm and forearm
   - Its angle is determined similar to the angle of the shoulder joint, by applying the law of cosines to a planar triangle:
     ```
     cos(elbow_angle) = (d² − L1² − L2²) / (2×L1×L2)
     ```
   - This allows us to find:
     ```cpp
     double elbow_angle = acos( ((pow(d, 2) - pow(L1, 2) - pow(L2, 2))) / (2 * L1 * L2) );
     ```
   - This angle determines the amount of rotation needed in the elbow joint so that the end of the forearm(manipulator) is at the target point

## Coordinate system
   - **The code utilizes cartesian coordinates for input parameters and calculations but can also be converted to polar coordinates if neccassary with the use of new helper function**
   | Axis | Direction | Planes |
   |------|-----------|--------|
   | **X Axis** | Forward relative to the base | XY & XZ |
   | **Y Axis** | Left relative to the base | XY & YZ |
   | **Z Axis** | Upwards relative to the base | XZ & YZ |
