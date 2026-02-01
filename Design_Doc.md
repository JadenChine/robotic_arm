# Six-Degree-of-Freedom Robotic Arm

## Table of Contents

- [1. Project Overview](#1-project-overview)
- [2. System-Level Goals](#2-system-level-goals)
- [3. High-Level System Architecture](#3-high-level-system-architecture)
- [4. Robotic Arm Subsystem](#4-robotic-arm-subsystem)
- [5. Wireless Controller Subsystem](#5-wireless-controller-subsystem)
- [6. Web / Mobile Interface Subsystem](#6-web--mobile-interface-subsystem)
- [7. Unified Communication Protocol](#7-unified-communication-protocol)
- [8. Modularity & Future Expansion](#8-modularity--future-expansion)
- [9. Future Sections (Planned Additions)](#9-future-sections-planned-additions)
- [10. Revision Strategy](#10-revision-strategy)

---

## System Design & Requirements Document (Living Document)

**Document Status:** Draft – Initial Structure  
**Document Type:** Design Specification & Requirements  
**Intended Audience:** Robotics, Embedded, Firmware, Electronics, Mechanical, and Software Developers  
**Document Nature:** Modular and expandable; sections may be added, replaced, or refined as the project evolves.

---

## 1. Project Overview

This project aims to design and build a six-degree-of-freedom (6-DOF) robotic arm system capable of being controlled through both a custom wireless controller and a web/mobile application interface. The robotic arm will perform all motion processing and kinematics calculations onboard using a microcontroller, while external interfaces provide only high-level motion commands.

The goal of this document is to:
	•	Define system goals and architecture
	•	Establish responsibilities of each subsystem
	•	Provide clear interaction rules between system components
	•	Serve as a reference for current and future team members
	•	Enable modular expansion of hardware and software features

This document is intentionally modular so future updates can refine or replace sections without restructuring the entire document.

---

## 2. System-Level Goals

The robotic arm system should:
	1.	Provide smooth and reliable 6-DOF movement.
	2.	Support control from multiple input sources.
	3.	Use a unified communication format for all control inputs.
	4.	Perform all motion calculations onboard the arm controller.
	5.	Allow future expansion in hardware, software, and control modes.
	6.	Support eventual automation and scripting capabilities.

---

## 3. High-Level System Architecture

The system consists of three major subsystems:
	1.	Robotic Arm Hardware & Control System
	2.	Wireless Physical Controller
	3.	Web / Mobile Interface Application

All command sources communicate with the robotic arm through a common command protocol, ensuring interchangeable control methods.

Wireless Controller ----->
                            \
                             --> Robotic Arm Controller --> Motors
                            /
Web / Mobile Interface --->

---
# MAIN SUBSYSTEMS
---
## 4. Robotic Arm Subsystem

### 4.1 Purpose

The robotic arm subsystem physically performs all movement and executes motion commands received from external controllers.

All kinematics, motion planning, and motor control logic are processed locally.

### 4.2 Mechanical Structure

The robotic arm includes the following joints:

#### Shoulder Assembly
	•	Shoulder Rotation
	•	Full 360° rotation.
	•	Enables horizontal positioning.
	•	Shoulder Pivot/Bend
	•	Controlled via servo motor.
	•	Enables vertical lifting motion.

#### Elbow Assembly
	•	First Elbow Joint
	•	Provides bending movement.
	•	Second Elbow Joint
	•	Bends in the same direction as the first elbow.
	•	Enables additional reach and articulation.

#### Wrist Assembly
	•	Wrist Rotation
	•	Capable of 360° rotation.

(Future additions such as grippers or tools are expected.)

### 4.3 Electronics & Processing

The arm will include:
	•	Microcontroller (e.g., Wi-Fi/Bluetooth Pico or similar)
	•	Motor drivers
	•	Servo controllers
	•	Power regulation system
	•	Communication interfaces

All computation occurs here:
	•	Inverse kinematics
	•	Joint control
	•	Safety limits
	•	Motor timing
	•	Command decoding

External controllers should never directly control motors.

### 4.4 Motion Control Responsibilities

The arm controller must:
	•	Decode incoming commands
	•	Perform kinematic calculations
	•	Convert commands to joint angles
	•	Apply movement constraints
	•	Execute motor control signals

Future upgrades may include:
	•	Path planning
	•	Collision avoidance
	•	Motion smoothing
	•	Acceleration profiles

### 4.5 Safety Requirements (Initial)

Planned safeguards:
	•	Joint motion limits
	•	Emergency stop handling
	•	Communication timeout stop
	•	Motor current limits

---

## 5. Wireless Controller Subsystem

### 5.1 Purpose

The wireless controller provides manual physical control of the robotic arm.

It sends commands wirelessly to the robotic arm controller.

### 5.2 Control Modes

The controller operates in two selectable modes.

#### Mode 1 — Direct Joint Control

Three potentiometers control:
	1.	Shoulder rotation
	2.	Shoulder bending
	3.	First elbow bending

Knob position directly maps to joint angle.

#### Mode 2 — Cartesian Position Control

Three potentiometers control:
	1.	X movement
	2.	Y movement
	3.	Z movement

The robotic arm controller performs inverse kinematics to move accordingly.

### 5.3 Hardware Components (Planned)

Possible components:
	•	Microcontroller
	•	3 potentiometers
	•	Mode selection switch
	•	Wireless module
	•	Battery system

Future versions may include:
	•	Displays
	•	Buttons
	•	Haptic feedback
	•	Preset buttons

### 5.4 Controller Responsibilities

The controller must:
	•	Read knob positions
	•	Determine active mode
	•	Package commands
	•	Transmit data wirelessly

It must not perform motion calculations.

---

## 6. Web / Mobile Interface Subsystem

### 6.1 Purpose

Provide remote control and monitoring capabilities via web or mobile devices.

Allows control without the physical remote.

### 6.2 Planned Capabilities (Expandable)

Potential features include:
	•	Joint angle control
	•	Cartesian movement control
	•	Live arm status display
	•	Preset position control
	•	Motion recording & playback
	•	Diagnostics view

Exact UI details will be defined later.

### 6.3 Application Architecture (Early Concept)

Possible architectures:
	•	Web server hosted on robotic arm controller
	•	Separate server relay
	•	Direct Wi-Fi connection

Final choice TBD.

### 6.4 Responsibilities

The application must:
	•	Provide user input controls
	•	Send commands using standard protocol
	•	Display arm state data

It does not control motors directly.

---

## 7. Unified Communication Protocol

### 7.1 Purpose

All controllers must communicate with the robotic arm using a single shared data format.

This ensures interchangeable input sources.

### 7.2 Communication Model

Command flow:

Controller Input → Command Packet → Arm Controller → Motion Execution

Both wireless remote and web interface must generate identical packet structures.

### 7.3 Proposed Command Packet Structure (Initial Draft)

Example logical structure:

{
  "mode": "JOINT" | "CARTESIAN",
  "values": [v1, v2, v3],
  "timestamp": 12345678,
  "source": "REMOTE" | "WEB"
}

#### Fields
	•	mode
	•	JOINT: direct joint control
	•	CARTESIAN: XYZ movement
	•	values
	•	Three numeric parameters depending on mode.
	•	timestamp
	•	Used for packet freshness.
	•	source
	•	Useful for debugging and logging.

### 7.4 Transport Options (TBD)

Possible methods:
	•	Wi-Fi UDP/TCP
	•	Bluetooth
	•	Serial RF modules

Final choice pending.

### 7.5 Command Processing on Arm

Upon packet reception:
	1.	Validate packet
	2.	Decode command
	3.	Apply safety limits
	4.	Perform kinematics
	5.	Execute motion

---

## 8. Modularity & Future Expansion

The system should support:
	•	Additional control modes
	•	Additional joints or tools
	•	Automation scripts
	•	Vision systems
	•	Sensor feedback

New features should plug into existing communication and control layers.

---

## 9. Future Sections (Planned Additions)

Future document expansions may include:
	•	Detailed mechanical dimensions
	•	Exact motor selection
	•	Electrical schematics
	•	Firmware architecture
	•	Communication standards
	•	Safety certification strategy
	•	Manufacturing plans

---

## 10. Revision Strategy

This document is expected to evolve as development progresses.

Recommended practice:
	•	Track version changes
	•	Document decisions
	•	Record subsystem updates

---

## End of Initial Draft

This document establishes the baseline architecture and will grow alongside system development.