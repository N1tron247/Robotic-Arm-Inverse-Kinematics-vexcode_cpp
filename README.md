# Robotic Arm Inverse Kinematics (VEX V5)

Inverse kinematics (IK) implementation for a VEX V5 multi-joint robotic arm.

## Overview
This project computes joint angles required for the arm end-effector to reach a target position.

- Platform: VEX V5
- Language: C++
- Arm DOF: **(2-link planar / 3-link / etc.)**
- IK method: **(geometric / Jacobian / CCD / etc.)**

## Repo Layout
- `src/` / `include/` (or your VEXcode structure) contains the VEX project code
- `docs/` (optional) for diagrams and math notes

## How it works (high level)
1. Forward kinematics computes end-effector position from joint angles
2. Inverse kinematics solves angles from a desired target position
3. Joint limits are enforced to keep motion feasible

## How to run
1. Open the project in VEXcode V5
2. Build + download to the brain
3. **Describe how you set the target (buttons, constants, etc.)**

## Demo
Add a short video/gif later (optional).
