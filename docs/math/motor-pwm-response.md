# Motor PWM Response

## Goal

Understand how a PWM command affects motor voltage, speed, and robot motion.

PWM is used because a microcontroller usually cannot output arbitrary analog voltage directly. Instead, it rapidly switches the motor driver input on and off.

---

## PWM Signal

A PWM signal is a discrete-time command that creates a repeating square wave.

The duty cycle is:

$$
D=
\frac{T_{on}}{T}
$$

where:

- $T_{on}$ is the time the signal is high
- $T$ is the total period
- $0 \leq D \leq 1$

---

## Average Voltage

If the motor supply voltage is $V_s$, the approximate average motor voltage is:

$$
V_{avg}=
D V_s
$$

Example:

If:

$$
V_s = 12V
$$

and:

$$
D = 0.5
$$

then:

$$
V_{avg}=
0.5(12)=
6V
$$

---

## Arduino PWM Value

Arduino-style PWM often uses values from 0 to 255.

The duty cycle is:

$$
D=
\frac{PWM}{255}
$$

So:

$$
V_{avg}=
\frac{PWM}{255}V_s
$$

---

## Motor Speed Relationship

For a simple DC motor, motor speed is roughly proportional to voltage after overcoming friction:

$$
\omega
\approx
K V_{avg}
$$

Substitute the PWM relationship:

$$
\omega
\approx
K
\frac{PWM}{255}
V_s
$$

This means higher PWM usually gives higher motor speed.

---

## Dead Zone

Real motors often do not move at very low PWM values because of static friction.

So the actual response may look like:

$$
\omega = 0
$$

for:

$$
PWM < PWM_{min}
$$

Then for larger PWM:

$$
\omega
\approx
K(PWM-PWM_{min})
$$

This is why a motor might not move until PWM is around 80 or 90.

---

## Motor as a Low-Pass System

A DC motor does not instantly change speed.

A simple first-order model is:

$$
\tau\frac{d\omega(t)}{dt}+
\omega(t)=
K u(t)
$$

where:

- $\omega(t)$ is angular velocity
- $u(t)$ is motor command or voltage input
- $K$ is motor gain
- $\tau$ is the motor time constant

---

## Transfer Function

Taking the Laplace transform with zero initial condition:

$$
\tau s\Omega(s)+
\Omega(s)=
K U(s)
$$

Factor:

$$
\Omega(s)(\tau s+1)=
K U(s)
$$

So:

$$
H(s)=
\frac{\Omega(s)}{U(s)}=
\frac{K}{\tau s+1}
$$

This is a first-order low-pass system.

---

## Step Response

If the input command suddenly changes to a constant value, the speed approaches the final value gradually:

$$
\omega(t)=
K u_0
\left(
1-e^{-t/\tau}
\right)
$$

This explains why motor speed ramps up instead of changing instantly.

---

## Discrete-Time Approximation

With sample time $\Delta t$, Euler integration gives:

$$
\omega[k+1]=
\omega[k]
+
\Delta t
\frac{Ku[k]-\omega[k]}{\tau}
$$

This can be rewritten as:

$$
\omega[k+1]=
\left(1-\frac{\Delta t}{\tau}\right)\omega[k]
+
\frac{K\Delta t}{\tau}u[k]
$$

---

## Signal and Systems Interpretation

The motor behaves like a low-pass filter:

```text
PWM command
    ↓
motor dynamics
    ↓
smoothed speed response
```

High-frequency switching from PWM is mostly smoothed by:

- motor inductance
- motor inertia
- mechanical load

So even though PWM is a square wave, the motor speed changes more smoothly.

---

## Experimental Response

A real PWM test may look like this:

| PWM | RPM |
|---:|---:|
| 0 | 0 |
| 60 | 0 |
| 90 | 35 |
| 120 | 80 |
| 180 | 150 |
| 255 | 240 |

This shows:

- low PWM dead zone
- increasing speed with PWM
- possible saturation near high PWM

---

## Robotics Use

Motor PWM response is useful for:

- finding minimum usable PWM
- estimating max speed
- tuning PID controllers
- comparing left and right motors
- detecting weak batteries or mechanical friction

---

## Notes

The relationship between PWM and motor speed is not perfectly linear.

It depends on:

- battery voltage
- motor load
- gear ratio
- friction
- motor driver voltage drop
- floor contact
- wheel slip