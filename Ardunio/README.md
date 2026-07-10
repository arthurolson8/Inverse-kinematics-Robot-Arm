
# Software Architecture

The robot is controlled by a custom Arduino program written in C++ using the Arduino IDE. Rather than relying on existing robotic arm frameworks, the software was developed specifically for this robot's mechanical design, servo configuration, and joint geometry.

The software is organized around modular subsystems responsible for robot configuration, joint control, joystick input, calibration, gripper control, and inverse kinematics. During development these modules were combined into a single Arduino sketch (`MARK_IV_IK_ARM.ino`) to simplify deployment and debugging.

---
## Control Flow Chart

User Input (Joystick or Camera)
            │
            ▼
Desired Position (X, Y, Z)
            │
            ▼
Inverse Kinematics Solver
            │
            ▼
Joint Angles
            │
            ▼
Servo Calibration Offsets
            │
            ▼
Motion Limiting & Safety Checks
            │
            ▼
Servo Commands
            │
            ▼
Physical Robot Movement

---
# Software Design Philosophy

The primary goals during development were:

- Modular and readable code
- Easy servo calibration
- Smooth joint motion
- Expandability for inverse kinematics
- Support for future autonomous operation

Instead of directly writing servo positions throughout the program, each joint is treated as an independent object with its own limits, speed, home position, and calibration settings. This makes it possible to modify individual joints without rewriting the entire control system.

---

# Robot Configuration

The first section of the program defines the hardware configuration of the robot.

This includes:

- Servo pin assignments
- Joystick analog pins
- Joystick button pins
- Servo home positions
- Motion limits
- Servo directions
- Servo speeds
- Servo pulse widths
- Servo travel ranges

Separating these values from the rest of the code makes it possible to mechanically redesign the robot without changing the control algorithms.

---

# Joint Control

Each joint maintains its own state including:

- Current position
- Target position
- Home position
- Minimum angle
- Maximum angle
- Maximum movement speed
- Servo direction
- Servo type

Every program loop updates each joint independently.

Rather than instantly jumping to the requested angle, each joint moves toward its target position at a controlled speed. This produces smoother motion, reduces shock loading on the gears, and decreases current spikes.

---

# Joystick Control

The robot is manually controlled using three dual-axis joysticks. (But only 2 in Inverse kinematics mode)

The joysticks provide analog control inputs corresponding to the robot's six degrees of freedom.

Each joystick value is read using Arduino analog inputs.

The joystick values are converted into incremental motion commands rather than absolute positions. This allows the servos to hold their current position whenever the joystick returns to center.

Deadbands are implemented around the joystick center position to eliminate unwanted drift caused by analog noise.

---

# Servo Calibration

One of the largest challenges during development was aligning the mathematical model of the robot with the physical servo positions.

Although inverse kinematics computes joint angles mathematically, every servo is mounted differently and has its own mechanical zero position.

To solve this problem, calibration offsets were introduced.

The calibration process consists of:

1. Moving each joint into a known mechanical reference position.
2. Recording the raw servo angle.
3. Comparing the measured position to the desired mathematical angle.
4. Computing an offset for each servo.
5. Applying the offset whenever inverse kinematics calculates a new joint angle.

This allows the inverse kinematics model to remain independent of the physical mounting orientation of each servo.

---

# Motion Limits

Every servo is constrained to safe operating limits.

Before commanding any movement, the requested angle is clamped between predefined minimum and maximum values.

This prevents:

- Mechanical interference
- Gear damage
- Servo overtravel
- Wiring collisions

Software limits provide an additional layer of protection beyond the physical design.

---

# Inverse Kinematics

The long-term objective of the software is to allow the robot to move to any point within its reachable workspace using Cartesian coordinates.

Instead of manually controlling every servo, the operator specifies the desired end-effector position.

For example:

X = 180 mm

Y = 120 mm

Z = 90 mm

The inverse kinematics algorithm calculates the required joint angles for:

- Base rotation
- Shoulder
- Elbow
- Wrist pitch

Wrist Pitch is locked to be level with the table until a button is pressed then it is free to move. When the lick is engaged again it snaps back to normal horizontal position.

These calculated angles are then converted into calibrated servo commands before being sent to the motors.

This approach separates robot motion from individual servo control, making future autonomous operation possible.

---

# Future Autonomous Operation

The software architecture has been designed with future expansion in mind.

Planned capabilities include:

- Camera-based object detection
- Object size estimation
- Automatic pick-and-place
- Motion recording and playback
- Vision-guided positioning
- Adaptive gripping
- Closed-loop control

The modular design allows these features to be added without restructuring the existing software.

---

# Development Process

This software was developed iteratively alongside the mechanical design of the robot.

As each mechanical subsystem was redesigned, the software was updated to accommodate new joint limits, calibration offsets, and transmission configurations.

Several iterations were required before achieving reliable and repeatable motion due to:

- Servo mounting changes
- Gear ratio modifications
- Shoulder joint redesigns
- Mechanical backlash
- Servo calibration refinement

Developing the software in parallel with the hardware provided valuable experience integrating mechanical, electrical, and embedded software systems into a complete robotic platform.

---

# Lessons Learned

This project provided practical experience in:

- Embedded C++ programming
- Arduino development
- Robot kinematics
- Servo calibration
- Motion control
- Electronics integration
- Mechanical-software integration
- Iterative engineering design
- Debugging embedded systems
