# Inverse Kinematics Robot Arm

## Overview

This project is a custom-designed 4-degree-of-freedom robotic arm built from the ground up using Solidworks, Arduino, and 3D-printed components. The goal of this project is to develop practical skills in mechanical design, embedded systems, robotics, and control algorithms, and also feel like Tony Stark.
The robot is designed to perform precise pick-and-place operations using inverse kinematics, high-torque servo motors, and a custom rack-and-pinion gripper with auto table leveling.

---

## Objectives

- Design a fully custom robotic arm in SolidWorks
- Manufacture structural components using 3D printing
- Develop an Arduino-based control system
- Implement forward and inverse kinematics
- Create a modular robot that can be expanded with sensors and computer vision (hopefully)
- Document the engineering design process from concept to finished prototype

---

## Features

- Custom 3D-printed mechanical design
- 4 Degrees of Freedom
- Bearing-supported base, shoulder, elbow, and wrist joints
- High-torque DS3218 servos
- SG90 rack-and-pinion gripper
- Arduino Uno R4 WiFi controller
- PCA9685 16-channel servo driver (have not integrated yet)
- Manual joystick control
- Modular electronics design
- Expandable software architecture

Planned Features

- Cartesian position control
- Inverse kinematics
- Autonomous pick-and-place
- Computer vision using OpenCV
- Camera-based object detection
- Object size estimation
- Automatic gripping
- Motion recording and playback

---

### Hardware / Mechanical

- Custom CAD using SolidWorks
- Custom 3D Printed parts using PLA
- 2 M8 60mm Bolts
- 1 M8 50mm Bolt
- 2 M8 Nuts
- Various sizes of M2 and M3 bolts and nuts
- M2 and M8 threaded inserts
- 8 608 ZZ bearings

### Electronics

- Arduino Uno R4 WiFi
- PCA9685 Servo Driver
- 4× DS3218 20kg·cm Servos
- 2× SG90 Micro Servos
- Three Dual-Axis Joysticks (only 2 used for final controls)
- External 5V 10A Power Supply

---

## Software

- Arduino IDE
- C++
- Solidworks
- Git
- GitHub

Future Software

- Python
- OpenCV
- Serial Communication
- Autonomous Motion Planning

---

## Mechanical Design

The arm uses bearing-supported rotating joints to reduce servo loading and improve structural rigidity. The base, elbow, and wrist joints are gear-driven with a 1:1 ratio, while the shoulder joint is direct drive. For the final version of the gripper, I opted for a Rack and pinion design after 2 other past versions.

Major design goals include

- Lightweight construction
- High stiffness
- Modular assemblies
- Easy maintenance
- Minimal backlash
- Manufacturable using hobby-grade 3D printers

---

## Project Status

Current Progress

- [x] Overall robot architecture completed
- [x] CAD design completed
- [x] Mechanical design iterations
- [x] Gear train prototypes
- [x] Manufacturing
- [x] Assembly
- [x] Wiring
- [x] Servo calibration
- [x] Joystick control
- [x] Forward kinematics
- [x] Inverse kinematics
- [ ] Autonomous pick-and-place w computer vison

---

## Design Iterations

Version 1
- Initial geared shoulder joint (gears slipped) with parallel arm gripper
- bearing-supported joints
- heat set inserts

Version 2
- Reinforced the servo mount and made the shoulder direct drive
- Made a gripper with a closing mechanism supported by rubber bands (broke servo)

Version 3 (Current)
- Improved structural rigidity
- Rack-and-pinion gripper
- Modular arm assemblies

Future Versions
- Computer vision
- Force sensing
- Autonomous object sorting

---

## Lessons Learned

Throughout this project, I have gained experience in

- Mechanical design
- CAD modeling
- Design iteration
- Failure analysis
- Rapid prototyping
- Embedded programming
- Robotics
- Electronics integration
- Manufacturing constraints
- Engineering documentation

---

## Possible Future Improvements

- Computer vision
- Camera calibration
- Automatic object detection
- Object size estimation
- Adaptive gripping
- Motion planning
- Closed-loop control
- Force sensing

