# Robotic Arm Inverse Kinematics (VEX V5)

Inverse kinematics (IK) implementation for a VEX V5 multi-joint robotic arm.

## Overview
This project computes joint angles required for the arm's manipulator(claw) to reach a target position.

- Platform: VEX V5
- Language: C++
- Arm DOF: **3 (Waist, Shoulder, Elbow | Claw is excluded from DOF)**
- IK method: **Geometric**

## Repo Layout
- `src/` / `include/` (or your VEXcode structure) contains the VEX project code
- `docs/` (optional) for diagrams and math notes

## How it works
1. Forward kinematics computes manipulator's position from joint angles
2. Inverse kinematics solves angles from a desired target position
3. Joint limits are enforced to keep motion within the robot's physical capabilites

## How to run
1. Open the project in VEXcode V5
2. Build + download to the brain
3. **Measure the lengths of your robotic arm's sub-lengths(upper arm, forearm) and enter them into the top of the code into their corresponding variables**

## Demo
Add a short video/gif later (optional).
