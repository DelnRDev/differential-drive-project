# Kalman Filter Math

## Goal

A Kalman filter estimates the true state of a system by combining:

```text
model prediction
+
sensor measurement
```

For a robot, this can mean estimating:

- position
- velocity
- heading
- sensor bias

---

## State Vector

Example 1D state:

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

## System Model

The state update model is:

$$
\vec{x}_k=
A\vec{x}_{k-1}
+
B\vec{u}_k
+
\vec{w}_k
$$

where:

- $A$ is the state transition matrix
- $B$ is the input matrix
- $\vec{u}_k$ is the control input
- $\vec{w}_k$ is process noise

The process noise is assumed to have covariance:

$$
Q
$$

---

## Measurement Model

The sensor measurement is modeled as:

$$
\vec{z}_k=
H\vec{x}_k
+
\vec{v}_k
$$

where:

- $\vec{z}_k$ is the measurement vector
- $H$ maps the state to the measurement
- $\vec{v}_k$ is measurement noise

The measurement noise is assumed to have covariance:

$$
R
$$

---

# Kalman Filter Algorithm

The Kalman filter has two main steps:

```text
Prediction
↓
Correction
```

---

## 1. Prediction Step

### Predicted State

$$
\hat{\vec{x}}_{k|k-1}=
A\hat{\vec{x}}_{k-1|k-1}
+
B\vec{u}_k
$$

This means:

```text
current estimate
=
model prediction from previous estimate
```

---

### Predicted Covariance

$$
P_{k|k-1}=
AP_{k-1|k-1}A^T
+
Q
$$

where:

- $P$ represents uncertainty in the state estimate
- $Q$ represents uncertainty added by the model

---

## 2. Correction Step

### Innovation / Measurement Residual

$$
\vec{y}_k =
\vec{z}_k-
H\hat{\vec{x}}_{k|k-1}
$$

This means:

```text
innovation
=
actual measurement
-
predicted measurement
```

---

### Innovation Covariance

$$
S_k=
HP_{k|k-1}H^T
+
R
$$

This represents uncertainty in the innovation.

---

### Kalman Gain

$$
K_k=
P_{k|k-1}H^T S_k^{-1}
$$

The Kalman gain controls how much the filter trusts the sensor measurement.

---

### Updated State Estimate

$$
\hat{\vec{x}}_{k|k}=
\hat{\vec{x}}_{k|k-1}
+
K_k\vec{y}_k
$$

---

### Updated Covariance

Simple form:

$$
P_{k|k}=
(I-K_kH)P_{k|k-1}
$$

More numerically stable Joseph form:

$$
P_{k|k}=
(I-K_kH)P_{k|k-1}(I-K_kH)^T
+
K_kRK_k^T
$$

---

# Example: 1D Position and Velocity

## State

$$
\vec{x}_k=
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
$$

---

## State Transition Matrix

Using constant velocity motion:

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
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
p_{k-1} \\
v_{k-1}
\end{bmatrix}
$$

So:

$$
A=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
$$

---

## Measurement Matrix

If the sensor only measures position:

$$
z_k=
p_k
$$

then:

$$
H=
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$

because:

$$
z_k=
\begin{bmatrix}
1 & 0
\end{bmatrix}
\begin{bmatrix}
p_k \\
v_k
\end{bmatrix}
$$

---

## Meaning of $Q$, $R$, and $P$

### $P$: Estimate uncertainty

$$
P_k=
\begin{bmatrix}
\sigma_p^2 & \sigma_{pv} \\
\sigma_{vp} & \sigma_v^2
\end{bmatrix}
$$

Larger $P$ means:

```text
I am less confident in my current estimate.
```

---

### $Q$: Process noise covariance

$Q$ describes how much uncertainty the motion model adds.

Large $Q$ means:

```text
I do not fully trust my model.
```

Small $Q$ means:

```text
I trust my model more.
```

---

### $R$: Measurement noise covariance

$R$ describes how noisy the sensor is.

Large $R$ means:

```text
I trust the sensor less.
```

Small $R$ means:

```text
I trust the sensor more.
```

---

## Robotics Interpretation

For a robot:

```text
Prediction:
Use motor/encoder/odometry model to guess where the robot is.

Correction:
Use sensor measurement to correct the guess.
```

Example:

```text
Encoder odometry says:
x = 1.05 m

External sensor says:
x = 1.00 m

Kalman filter combines them:
x ≈ 1.02 m
```

---

## Notes

A basic Kalman filter assumes:

- linear system model
- Gaussian noise
- known covariance matrices

For nonlinear robot motion, an Extended Kalman Filter is usually used.