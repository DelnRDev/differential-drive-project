# 1D Kalman Filter Math

## Goal

Estimate the true state of a 1D system by combining:

- model prediction
- sensor measurement

---

## State Vector

For 1D position and velocity:

$$
\vec{x}_k=
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
$$

where:

- $p_k$ is position
- $v_k$ is velocity

---

## State Transition Model

Assume constant velocity during one sample interval $\Delta t$:

$$
p_k=
p_{k-1}
+
v_{k-1}\Delta t
$$

$$
v_k=
v_{k-1}
$$

Matrix form:

$$
\vec{x}_k=
A\vec{x}_{k-1}
+
\vec{w}_k
$$

where:

$$
A=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
$$

---

## Measurement Model

If the sensor only measures position:

$$
z_k=
p_k
+
v_k^{noise}
$$

then:

$$
z_k=
H\vec{x}_k
+
v_k^{noise}
$$

where:

$$
H=
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

---

## Prediction Step

Predicted state:

$$
\hat{\vec{x}}_{k|k-1}=
A\hat{\vec{x}}_{k-1|k-1}
$$

Predicted covariance:

$$
P_{k|k-1}=
AP_{k-1|k-1}A^T
+
Q
$$

---

## Correction Step

Measurement residual:

$$
y_k=
z_k-
H\hat{\vec{x}}_{k|k-1}
$$

Innovation covariance:

$$
S_k=
HP_{k|k-1}H^T
+
R
$$

Kalman gain:

$$
K_k=
P_{k|k-1}H^TS_k^{-1}
$$

Updated state estimate:

$$
\hat{\vec{x}}_{k|k}=
\hat{\vec{x}}_{k|k-1}
+
K_ky_k
$$

Updated covariance:

$$
P_{k|k}=
(I-K_kH)P_{k|k-1}
$$

---

## Meaning of $P$, $Q$, and $R$

### $P$: estimate uncertainty

Large $P$ means the filter is less confident in its current estimate.

### $Q$: process noise covariance

Large $Q$ means the filter trusts the motion model less.

### $R$: measurement noise covariance

Large $R$ means the filter trusts the sensor less.

---

## Robotics Interpretation

Prediction:

```text
Use the previous position and velocity to predict the next position.
```

Correction:

```text
Use the sensor measurement to correct the prediction.
```

Example:

```text
Prediction says:  p = 1.05 m
Sensor says:      p = 1.00 m
Filter estimate:  p ≈ somewhere between them
```

---

## C++ Implementation

```cpp
struct Kalman1D {
    double p;
    double v;

    double P00;
    double P01;
    double P10;
    double P11;

    double Q00;
    double Q01;
    double Q10;
    double Q11;

    double R;
};

void predict(Kalman1D& kf, double dt) {
    kf.p = kf.p + kf.v * dt;

    double P00 = kf.P00 + dt * kf.P10 + dt * kf.P01 + dt * dt * kf.P11 + kf.Q00;
    double P01 = kf.P01 + dt * kf.P11 + kf.Q01;
    double P10 = kf.P10 + dt * kf.P11 + kf.Q10;
    double P11 = kf.P11 + kf.Q11;

    kf.P00 = P00;
    kf.P01 = P01;
    kf.P10 = P10;
    kf.P11 = P11;
}

void update(Kalman1D& kf, double measuredPosition) {
    double y = measuredPosition - kf.p;

    double S = kf.P00 + kf.R;

    double K0 = kf.P00 / S;
    double K1 = kf.P10 / S;

    kf.p = kf.p + K0 * y;
    kf.v = kf.v + K1 * y;

    double P00 = (1.0 - K0) * kf.P00;
    double P01 = (1.0 - K0) * kf.P01;
    double P10 = kf.P10 - K1 * kf.P00;
    double P11 = kf.P11 - K1 * kf.P01;

    kf.P00 = P00;
    kf.P01 = P01;
    kf.P10 = P10;
    kf.P11 = P11;
}
```