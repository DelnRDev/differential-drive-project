# Exponential Moving Average Filter

## Goal

An exponential moving average filter smooths a noisy discrete-time signal.

In robotics, it can be used to smooth:

- encoder RPM
- IMU yaw rate
- distance sensor readings
- motor current measurements

---

## Discrete-Time Signal Notation

Let the raw sensor signal be:

$$
x[n]
$$

where $n$ is the discrete-time sample index.

The filtered output is:

$$
y[n]
$$

---

## Difference Equation

The exponential moving average filter is:

$$
y[n]=
\alpha x[n]
+
(1-\alpha)y[n-1]
$$

where:

- $x[n]$ is the current input sample
- $y[n]$ is the current filtered output
- $y[n-1]$ is the previous filtered output
- $\alpha$ is the smoothing factor

Usually:

$$
0 < \alpha \leq 1
$$

---

## Signal and Systems Form

Rewrite the equation:

$$
y[n]-
(1-\alpha)y[n-1]=
\alpha x[n]
$$

This is a first-order discrete-time LTI system.

---

## Transfer Function

Taking the $z$-transform:

$$
Y(z)-
(1-\alpha)z^{-1}Y(z)=
\alpha X(z)
$$

Factor out $Y(z)$:

$$
Y(z)
\left[
1-(1-\alpha)z^{-1}
\right]=
\alpha X(z)
$$

So the transfer function is:

$$
H(z)=
\frac{Y(z)}{X(z)}=
\frac{\alpha}
{1-(1-\alpha)z^{-1}}
$$

---

## Pole

The pole is found from:

$$
1-(1-\alpha)z^{-1}=0
$$

So:

$$
z=1-\alpha
$$

Since $0<\alpha\leq1$:

$$
0 \leq 1-\alpha < 1
$$

So the pole is inside the unit circle, meaning the filter is stable.

---

## Frequency Response

The frequency response is found by evaluating on the unit circle:

$$
z=e^{j\omega}
$$

So:

$$
H(e^{j\omega})=
\frac{\alpha}
{1-(1-\alpha)e^{-j\omega}}
$$

This is a low-pass filter:

- low-frequency changes pass through
- high-frequency noise is reduced

---

## Effect of Alpha

Large $\alpha$:

```text
trusts new data more
less smoothing
faster response
```

Small $\alpha$:

```text
trusts old filtered value more
more smoothing
slower response
```

Examples:

$$
\alpha=0.8
$$

fast but noisy

$$
\alpha=0.1
$$

smooth but delayed

---

## Impulse Response

The system transfer function is:

$$
H(z)=
\frac{\alpha}
{1-(1-\alpha)z^{-1}}
$$

Using the standard form:

$$
\frac{1}{1-az^{-1}}
\leftrightarrow
a^n u[n]
$$

where:

$$
a=1-\alpha
$$

Therefore:

$$
h[n]=
\alpha(1-\alpha)^n u[n]
$$

This means the filter remembers past inputs with exponentially decreasing weight.

---

## Convolution Form

Since this is an LTI system:

$$
y[n]=
x[n]*h[n]
$$

So:

$$
y[n]=
\sum_{k=-\infty}^{\infty}
x[k]h[n-k]
$$

Because $h[n]$ is causal:

$$
y[n]=
\sum_{k=0}^{n}
\alpha(1-\alpha)^k x[n-k]
$$

So the newest sample has weight $\alpha$, and older samples have smaller weights.

---

## Step Response Intuition

If the input suddenly changes from $0$ to $1$:

$$
x[n]=u[n]
$$

then the output approaches $1$ gradually:

$$
y[n]=
1-(1-\alpha)^{n+1}
$$

This shows why the filter creates lag.

---

## Robotics Interpretation

For encoder RPM:

$$
x[n]=
RPM_{raw}[n]
$$

$$
y[n]=
RPM_{filtered}[n]
$$

Update rule:

$$
RPM_{filtered}[n]=
\alpha RPM_{raw}[n]
+
(1-\alpha)RPM_{filtered}[n-1]
$$

---

## C++ Implementation

```cpp
double exponentialMovingAverage(double rawValue, double previousFiltered, double alpha) {
    return alpha * rawValue + (1.0 - alpha) * previousFiltered;
}
```

Example use:

```cpp
filteredRPM = exponentialMovingAverage(rawRPM, filteredRPM, 0.2);
```

---

## Notes

This filter is useful when the sensor data is noisy, but it adds delay.

For control systems, too much smoothing can make the robot react slowly.