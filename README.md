# 4-DOF Robotic Arm

Simulation and kinematics code for a custom 4-DOF robotic arm. Full build writeup, including the physical prototype, servo controller redesign, and mechanical iteration, is on the [portfolio page](https://sanjayramkumar27.github.io/projects/robotic-arm/); this repo covers the simulation and kinematics side.

## What's here

- `IK_Path.py` — inverse kinematics and path generation for the arm.
- `reach_space.py` — computes and visualizes the reachable workspace from the IK model, accounting for actual servo travel limits (180°) rather than idealized ranges. Output: `workspace_cloud.png`.
- `Arm_python.py` — arm control/interface script.
- `mujoco_vis.py` — MuJoCo visualization for the simulated arm.
- `arm_model.xml` / `mjmodel.xml` — MJCF model of the arm used in MuJoCo.
- `platformio.ini`, `src/`, `include/`, `lib/`, `test/` — PlatformIO project for the embedded (Arduino) side.

## Background

The arm was built in two phases. Phase 1 was a physical prototype with a closed-form IK solution, derived by extending 2R planar-arm equations, running in real time on an Arduino Uno. Phase 2 moved the model into MuJoCo and switched to numerical IK using damped least-squares, which is what most of the code in this repo implements, chosen for robustness across joint configurations as the model kept changing during iteration.

The reachable workspace in `reach_space.py` is computed from the same validated IK model used for trajectory generation, so it reflects actual servo limits rather than the arm's full theoretical range of motion.

## Status

Active. Current work is a three-trace tracking error analysis comparing the commanded path, the MuJoCo-ideal trajectory, and the real arm's forward-kinematics output, to quantify where simulation and hardware diverge.

## Tools

Python, MuJoCo (MJCF), PlatformIO/Arduino.


[Watch the demo](https://youtu.be/enjijNBTH5s)
