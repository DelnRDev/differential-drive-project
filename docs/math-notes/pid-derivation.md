# PID Controller Derivation

## Goal

A PID controller computes a control output based on the error between a target value and a measured value.

For motor speed control:

- target value = desired RPM
- measured value = actual encoder RPM
- output = PWM command

---

## Error Signal

Let the target signal be:

$$
r(t)
$$

Let the measured output be:

$$
y(t)
$$

The error is:

$$
e(t)=
r(t)-y(t)
$$

---

## Proportional Term

The proportional term reacts to the current error:

$$
u_P(t)=
K_Pe(t)
$$

If the error is large, the controller gives a large response.

---

## Integral Term

The integral term reacts to accumulated error:

$$
u_I(t)=
K_I
\int_0^t
e(\tau)\,d\tau
$$

This helps reduce steady-state error.

---

## Derivative Term

The derivative term reacts to how fast the error is changing:

$$
u_D(t)=
K_D
\frac{de(t)}{dt}
$$

This helps damp oscillation and overshoot.

---

## Continuous-Time PID Controller

Combining all three terms:

$$
u(t)=
K_Pe(t)
+
K_I
\int_0^t
e(\tau)\,d\tau
+
K_D
\frac{de(t)}{dt}
$$

---

## Discrete-Time Signals

In code, the controller runs at discrete time steps.

Let:

- $e[k]$: error at sample $k$
- $u[k]$: control output at sample $k$
- $\Delta t$: sample time

---

## Discrete Integral Approximation

The accumulated error can be updated using a running sum:

$$
I[k]=
I[k-1]
+
e[k]\Delta t
$$

---

## Discrete Derivative Approximation

The derivative can be approximated by the difference between the current error and previous error:

$$
D[k]=
\frac{e[k]-e[k-1]}{\Delta t}
$$

---

## Discrete PID Controller

$$
u[k]=
K_Pe[k]
+
K_II[k]
+
K_DD[k]
$$

Substitute $I[k]$ and $D[k]$:

$$
u[k]=
K_Pe[k]
+
K_I
\left(
I[k-1]+e[k]\Delta t
\right)
+
K_D
\left(
\frac{e[k]-e[k-1]}{\Delta t}
\right)
$$

---

## PWM Output

For a motor controller, the output is usually limited:

$$
-255 \leq u[k] \leq 255
$$

So the commanded PWM is:

$$
PWM[k]=
\operatorname{clamp}(u[k], -255, 255)
$$

---

## Anti-Windup

If the output saturates but the integral term keeps growing, the controller may overshoot badly later.

This is called integral windup.

A simple anti-windup method is to limit the integral term:

$$
I_{min}
\leq
I[k]
\leq
I_{max}
$$

---

## C++ Implementation

```cpp
struct PID {
    double kp;
    double ki;
    double kd;

    double integral;
    double previousError;

    double outputMin;
    double outputMax;
};

double clampValue(double value, double minValue, double maxValue) {
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

double updatePID(PID& pid, double target, double measured, double dt) {
    double error = target - measured;

    pid.integral += error * dt;

    double derivative = (error - pid.previousError) / dt;

    double output =
        pid.kp * error +
        pid.ki * pid.integral +
        pid.kd * derivative;

    output = clampValue(output, pid.outputMin, pid.outputMax);

    pid.previousError = error;

    return output;
}
```

---

## Robotics Use

PID control can be used for:

- motor speed control
- turning to an angle
- driving straight
- distance control
- heading correction

---

## Notes

- $K_P$ controls reaction strength.
- $K_I$ corrects long-term bias.
- $K_D$ reacts to fast changes.
- Too much $K_P$ can cause oscillation.
- Too much $K_I$ can cause windup.
- Too much $K_D$ can amplify noise.