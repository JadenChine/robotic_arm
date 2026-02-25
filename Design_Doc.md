# Six-Degree-of-Freedom Robotic Arm


## Table of Contents

- [Project Overview](#project-overview)
- [1. Robotic Arm Subsystem](#1-robotic-arm-subsystem)
- [2. Wireless Controller Subsystem](#2-wireless-controller-subsystem)
- [3. Web / Mobile Interface Subsystem](#3-web--mobile-interface-subsystem)
- [4. Unified Communication Protocol](#4-unified-communication-protocol)
- [5. Modularity & Future Expansion](#5-modularity--future-expansion)


## Project Overview

This project is designed to build a six-degree-of-freedom (6-DOF) robotic arm system capable of being controlled through both a custom wireless controller and a web/mobile application interface. The robotic arm will perform all motion processing and kinematics calculations onboard using a microcontroller, while external interfaces provide only high-level motion commands.

The robotic arm system should:

1. Provide smooth and reliable 6-DOF movement
2. Support control from multiple input sources (wireless controller and web app)
3. Use a unified communication format for all control inputs
4. Perform all motion calculations onboard the arm controller
5. Allow future expansion in hardware, software, and control modes
6. Support movement automation and scripting capabilities

The system consists of three major subsystems:

1. Robotic Arm Hardware & Control System
2. Wireless Physical Controller
3. Web / Mobile Interface Application

All command sources communicate with the robotic arm through a common command protocol, ensuring interchangeable control methods.


# MAIN SUBSYSTEMS

## 1. Robotic Arm Subsystem

### 1.1 Purpose

The robotic arm subsystem physically performs all movement and executes motion commands received from external controllers. All kinematics, motion planning, and motor control logic are done locally on the arm's MCU. The arm MCU must also support devices connecting wirlessly to it to send controls. The XY plan will serve as the ground plan on which the arm will sit on. The Z plane is the vertical plane in reference to the ground. The arm will start at with the shoulder joint at (0,0,0) and the rest of the arm will extend into the positive Z plane.

### 1.2 Mechanical Structure

The robotic arm includes the following joints:

- **Joint 1 - Shoulder Rotation**
  - The shoulder on this arm is made of 2 seperate joints that work together to function like a shoulder. This joint (joint 1) serves as the base of the shoulder/arm. A servo motor will lie parallel to the ground and rotate ±360° (720° total) to rotate the entire arm in a circle about the z axis. 
- **Joint 2 - Shoulder Bending**
  - This the second joint of the shoulder. This joint enables the shoulder to pivot/bend ±85° (170° total) about the y-axis. Joints 1 and 2 are controlled via 2 sperate servos, but work together to rotate and bend like a normal shoulder joint. 
- **Joint 3 - First Elbow**
  -  Provides bending movement about the y-axis
- **Joint 4 - Second Elbow**
  - Provides bending movement about the y-axis
- **Joint 5 - Wrist**
  -  Final 

_(Future additions such as grippers or tools are expected)_

### 1.3 Electronics & Processing

- Microcontroller (e.g., Wi-Fi/Bluetooth Pico or similar)
- Motor drivers
- Servo controllers
- Power regulation system
- Communication interfaces

All computation occurs here:

- Inverse kinematics
- Joint control
- Safety limits
- Motor timing
- Command decoding

### 1.4 Motion Control Responsibilities

The arm controller must:

- Decode incoming commands
- Perform kinematic calculations
- Convert commands to joint angles
- Apply movement constraints
- Execute motor control signals

Future upgrades may include:

- Path planning
- Collision avoidance
- Motion smoothing
- Acceleration profiles

---

## 2. Wireless Remote Subsystem

### 2.1 Purpose

The wireless remote will be a battery powered, hand-held device which provides manual physical control of the robotic arm. It will connect wirelessly to the robotic arm controller and send controls over the air. 

### 2.2 Control Modes

The controller will have operates in two selectable modes.

**Mode 1 — Direct Joint Control**

- 6 encoders control each joint on the arm. There will be one encoder for every single servo on the arm to individaully move each joint.  

**Mode 2 — Cartesian Position Control**

- In this mode, the first 3 encoders on the remote will control the X, Y, and Z location of the tip of the arm. Adjusting the encoders will move the arm in the respective direction. The robotic arm performs tge inverse kinematics to move accordingly and thus no calculations will be done on the remote.

### 2.3 Physical Design

The wireless remote will have a Pi Pico as the MCU and will be battery powered via parallel batteries. There will be 6 encoders on the remote which will serve as the main input devices. There will be a physcial switch on the device to set the controller into Joint Control Mode or Cartesion Position Mode. There will be 3 LED lights on the remote to indicate different statuses or messages. There will be 3 buttons on the device to be used for any software commands we'd like such as commanding the arm to go to a "home position".

### 2.4 Controller Responsibilities

- Connect wirelessly to arm
- Determine active mode
- Read encoder positions
- Package commands
- Transmit data wirelessly

---

## 3. Web / Mobile Interface Subsystem

### 3.1 Purpose

Provide remote control and monitoring capabilities via web or mobile devices. Allows control without the physical remote.

### 3.2 Planned Capabilities (Expandable)

Potential features include:

- Joint angle control
- Cartesian movement control
- Live arm status display
- Preset position control
- Motion recording & playback
- Diagnostics view

### 3.3 Application Architecture (Early Concept)

Possible architectures:

- Web server hosted on robotic arm controller
- Separate server relay
- Direct Wi-Fi connection

### 3.4 Responsibilities

The application must:

- Provide user input controls
- Send commands using standard protocol
- Display arm state data

---

## 4. Unified Communication Protocol

### 4.1 Purpose

The arm will be receiving movement commands from a number of sources (wireless controller in individual joint control  mode, wireless controller in cartesion coordinate mode, web app controls, etc). Because of this, there must be a univeral communication standard/form so that the arm can easily decode the controls from any source in the same way. Any aditional scripting or control mechanisms will also use this communication standard.

### 4.2 Communication Model

```
Controller Input → Command Packet → Arm Controller → Motion Execution
```

Both wireless remote and web interface must generate identical packet structures.

### 4.3 Proposed Command Packet Structure (Initial Draft)

```json
{
  "mode": "JOINT" | "CARTESIAN",
  "values": [v1, v2, v3],
  "timestamp": 12345678,
  "source": "REMOTE" | "WEB"
}
```

#### Fields

- **mode**
  - `JOINT`: direct joint control
  - `CARTESIAN`: XYZ movement
- **values**
  - Three numeric parameters depending on mode
- **timestamp**
  - Used for packet freshness
- **source**
  - Useful for debugging and logging

### 4.4 Transport Options (To Be Determined)

Possible methods:

- Wi-Fi (UDP/TCP or something else)
- Bluetooth

### 4.5 Command Processing on Arm

1. Validate packet
2. Decode command
3. Apply safety limits
4. Perform kinematics
5. Execute motion

---

## 5. Modularity & Future Expansion

The system should support:

- Additional control modes
- Additional joints and joint offsets attatched at the end of the arm
- Automation scripts to script movements and arm positions
- Current state knowledge (current arm position/state)
- Vision systems
- Sensor feedback

New features should plug into existing communication and control layers.
