# Differential Drive Robot

## Overview

This project is my attempt to rebuild the robotics experience I had in VEX, but now using my current knowledge of embedded systems, control theory, signal processing, mathematical, and robotics software architecture.

Instead of using the VEX V5 ecosystem directly, this project uses low-cost Arduino-compatible hardware to explore the same core robotics ideas from the ground up.

The goal is not to replace the VEX V5 ecosystem or create a commercial robotics platform. This project is primarily for learning, experimentation, and understanding how a competition-style robot control stack works at the hardware and software level.

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