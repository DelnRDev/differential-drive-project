# Differential Drive Robot

## Overview
This project aims to build a VEX-style differential drive robot using an Arduino-compatible microcontroller and low-cost hardware. The system will gradually implement motor control, encoder feedback, PID control, IMU integration, and odometry for autonomous movement.

The goal is not only to make the robot move, but also to understand the mathematics, control theory, and software architecture behind robotics systems

---

## Motivation

I previously participated in VEX Robotics as a builder and driver, but many systems such as PID control, odometry, and sensor fusion were treated as tools that I used rather than fully understood. 
This project connects my past VEX robotics experience with my current learning in robotics, embedded systems, control systems, and engineering math.

---

## Learning Goals

- Embedded systems
- Motor control
- Sensors and feedback
- PID control
- Odometry
- Robotics software architecture

---

## Hardware

| Component | Purpose |
|---|---|
| Arduino-compatible Uno board | Main microcontroller |
| TB6612 motor driver | Drives DC motors |
| DC motors with encoders | Drivetrain actuation and wheel feedback |
| MPU6050 IMU | Heading estimation |
| SSD1306 OLED | Debug display |
| Battery pack | Robot power |

---

## Project Structure

```text

src/       -> source code

include/   -> header files

docs/      -> project documentation

data/      -> CSV test data

images/    -> graphs and screenshots

test/      -> PlatformIO unit tests

```

---

## Documentation

- [Engineering Log](docs/engineering-log.md)
- [Wiring Guide](docs/wiring.md)
- [Architecture](docs/architecture.md)

## Mathematical Notes

- [PID derivation](docs/math/pid-derivation.md)
- [Odometry derivation](docs/math/odometry-derivation.md)
- [Encoder RPM calculation](docs/math/encoder-rpm.md)

## License

MIT License