# Rebuilding My VEX Robotics Experience from Scratch

## Overview

This project is my attempt to rebuild the robotics experience I had in VEX, but now using my current knowledge of embedded systems, control theory, sensors, odometry, and robotics software architecture.

Instead of using the VEX V5 ecosystem directly, this project uses low-cost Arduino-compatible hardware to explore the same core robotics ideas from the ground up.

The goal is not to replace the VEX V5 ecosystem or create a commercial robotics platform. This project is primarily for learning, experimentation, and understanding how a competition-style robot control stack works at the hardware and software level.

## Motivation

I used to experience robotics mainly through building, driving, and competing in VEX. At that time, many parts of the system were already provided by the VEX ecosystem.

Now, I want to understand what is happening underneath:

- How motor commands become physical motion
- How encoders measure wheel movement and speed
- How IMU data becomes heading
- How odometry estimates robot pose
- How PID controls speed and heading
- How autonomous routines are organized in software
- How a robot control stack can be built from low-level components

This project connects my past VEX robotics experience with my current learning in robotics, embedded systems, control systems, and engineering math.

## Project Goals

The long-term goal is to build a mini competition-style robotics control stack, including:

- Motor PWM control
- Encoder pulse counting
- Live RPM measurement
- Motor speed PID
- IMU heading estimation
- Differential-drive odometry
- Motion primitives such as drive-distance and turn-to-angle
- Autonomous routines using a state machine
- A simple scheduler for multi-rate robot updates
- Eventually, path following such as Pure Pursuit

## Hardware

| Component | Purpose |
|---|---|
| Arduino-compatible Uno board | Main microcontroller |
| TB6612 motor driver | Drives DC motors |
| DC motors with encoders | Drivetrain actuation and wheel feedback |
| MPU6050 IMU | Heading estimation |
| SSD1306 OLED | Debug display |
| Battery pack | Robot power |

## Software Architecture

The code is being designed as a small robotics framework rather than one giant Arduino sketch.

Planned modules:

- `Motor`
- `Encoder`
- `PID`
- `IMUHeading`
- `Odometry`
- `Drivetrain`
- `Scheduler`
- `Autonomous`

The general control flow is:

```text
Sensors
  ↓
State estimation
  ↓
Control / autonomous decision
  ↓
Motor command
  ↓
Actuation
  ↓
Telemetry
