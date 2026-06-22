# Encoder RPM Calculation

## Goal

Convert encoder pulses into wheel speed.

The encoder produces pulses as the motor shaft or wheel rotates. By counting pulses over a known time interval, the motor speed can be estimated.

---

## Variables

- $N$: number of encoder pulses counted
- $N_{rev}$: number of pulses per revolution
- $\Delta t$: sampling time interval in seconds
- $f$: revolutions per second
- $RPM$: revolutions per minute

---

## Revolutions

The number of revolutions during the sample interval is:

$$
R=
\frac{N}{N_{rev}}
$$

---

## Revolutions Per Second

$$
f=
\frac{R}{\Delta t}
$$

Substitute $R = \frac{N}{N_{rev}}$:

$$
f=
\frac{N}{N_{rev}\Delta t}
$$

---

## RPM Formula

Since:

$$
1 \text{ rev/s}=
60 \text{ RPM}
$$

then:

$$
RPM=
60f
$$

Therefore:

$$
RPM=
\frac{60N}{N_{rev}\Delta t}
$$

---

## Example

Given:

- $N = 45$
- $N_{rev} = 360$
- $\Delta t = 0.1$

Then:

$$
RPM=
\frac{60(45)}{360(0.1)}
$$

$$
RPM
=
75
$$

---

## Direction

If the encoder can detect direction, then RPM can be signed:

$$
RPM > 0
$$

means forward rotation.

$$
RPM < 0
$$

means reverse rotation.

A signed formula can be written as:

$$
RPM=
\frac{60N_{signed}}{N_{rev}\Delta t}
$$

---

## Wheel Linear Speed

If the wheel radius is $r$, then angular velocity in rad/s is:

$$
\omega=
\frac{2\pi RPM}{60}
$$

Wheel linear speed is:

$$
v=
r\omega
$$

Substitute:

$$
v=
r
\frac{2\pi RPM}{60}
$$

So:

$$
v=
\frac{2\pi r RPM}{60}
$$

---

## Robotics Use

Encoder RPM is used for:

- measuring motor speed
- speed PID control
- detecting motor mismatch
- estimating wheel velocity
- odometry

---

## C++ Function

```cpp
double calculateRPM(long pulses, double pulsesPerRevolution, double dt) {
    return (60.0 * pulses) / (pulsesPerRevolution * dt);
}
```

For signed encoder counts:

```cpp
double calculateSignedRPM(long signedPulses, double pulsesPerRevolution, double dt) {
    return (60.0 * signedPulses) / (pulsesPerRevolution * dt);
}
```

---

## Notes

The RPM estimate depends strongly on the sample interval.

Short $\Delta t$:

- faster response
- noisier RPM estimate

Long $\Delta t$:

- smoother estimate
- slower response

This is why encoder speed is often filtered using a moving average or exponential moving average filter.