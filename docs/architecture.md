# Software Architecture

## Overview

This project is designed as a small robotics control stack instead of one large Arduino sketch.

The goal is to separate the robot software into clear modules, where each module has one main responsibility.

---

## Main Control Flow

```text
Sensors
  ↓
State Estimation
  ↓
Control
  ↓
Motor Output
  ↓
Telemetry
```

In the robot loop, this means:

```text
Read sensors
  ↓
Update encoder RPM / IMU heading
  ↓
Update odometry
  ↓
Run autonomous or driver control
  ↓
Send motor commands
  ↓
Print/debug data
```

---

## Modules

### Motor

Responsible for sending PWM and direction signals to the motor driver.

Main responsibilities:

- Set motor PWM
- Set motor direction
- Stop or coast the motor
- Later: support RPM control through PID

---

### Encoder

Responsible for counting encoder pulses and estimating wheel RPM.

Main responsibilities:

- Count pulses using interrupts
- Apply minimum pulse interval filtering
- Convert pulse count into wheel revolutions
- Estimate RPM over a time window
- Provide wheel distance change for odometry

---

### PID

Responsible for feedback control.

Main responsibilities:

- Compare target value and measured value
- Compute error
- Apply proportional, integral, and derivative terms
- Output a correction signal

Example use:

```text
Target RPM
  ↓
PID
  ↓
PWM correction
```

---

### IMUHeading

Responsible for estimating robot heading from the MPU6050 gyro.

Main responsibilities:

- Read gyro angular velocity
- Subtract gyro bias
- Integrate angular velocity to estimate heading
- Provide heading in radians or degrees

---

### Odometry

Responsible for estimating robot pose.

The robot pose is:

$$
[\vec{x}]_W =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

Main responsibilities:

- Read left and right encoder distance changes
- Use IMU heading
- Estimate robot position $(x, y)$
- Estimate robot heading $(\theta)$

---

### Drivetrain

Responsible for controlling the left and right motors together.

Main responsibilities:

- Set left and right motor PWM
- Set left and right target RPM
- Drive a target distance
- Turn to a target angle
- Hold heading while driving

Example:

```text
driveDistance(0.5 m)
turnTo(90 degrees)
```

---

### Scheduler

Responsible for running different robot tasks at different update periods.

Main responsibilities:

- Run IMU updates frequently
- Run RPM updates at a slower rate
- Run odometry/control at a stable rate
- Run telemetry slowly
- Avoid blocking `delay()` in the main control loop

---

## Update Periods

| Task | Period | Purpose |
|---|---:|---|
| IMU | 5 ms | Update heading estimate |
| RPM | 50–100 ms | Estimate wheel speed |
| Odometry | 10–20 ms | Update robot pose |
| Control | 10–20 ms | Run autonomous / motor control |
| Telemetry | 200 ms | Serial/OLED debugging |

These periods are starting values. They may change after testing.

---

## Current System Diagram

```text
Encoder ISR
  ↓
Encoder count
  ↓
RPM measurement
  ↓
Motor speed PID
  ↓
PWM output
  ↓
Motor motion
```

```text
MPU6050 gyro
  ↓
IMU heading estimate
  ↓
Odometry
  ↓
Drivetrain control
  ↓
Autonomous routine
```

---

## Design Philosophy

The robot code should be organized so each part has a clear job.

Instead of writing everything inside `main.cpp`, the project will eventually be split into classes:

```text
Motor
Encoder
PID
IMUHeading
Odometry
Drivetrain
Scheduler
Autonomous
```

This makes the robot easier to debug, test, and expand.