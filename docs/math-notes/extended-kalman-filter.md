# Extended Kalman Filter Math

## Goal

An Extended Kalman Filter estimates the state of a nonlinear system.

For robotics, the system is often nonlinear because motion depends on heading:

$$
x_{k+1}=
x_k
+
v_k\cos\theta_k\Delta t
$$

$$
y_{k+1}=
y_k
+
v_k\sin\theta_k\Delta t
$$

$$
\theta_{k+1}=
\theta_k
+
\omega_k\Delta t
$$

Because of $\cos\theta_k$ and $\sin\theta_k$, this model is nonlinear.

---

## State Vector

For a differential drive robot:

$$
\vec{x}_k=
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix}
$$

where:

- $x_k$ is the robot position in the world x-direction
- $y_k$ is the robot position in the world y-direction
- $\theta_k$ is the robot heading

---

## Control Input

$$
\vec{u}_k=
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
$$

where:

- $v_k$ is the robot linear velocity
- $\omega_k$ is the robot angular velocity

---

## Nonlinear Motion Model

The motion model is written as:

$$
\vec{x}_k=
f(\vec{x}_{k-1},\vec{u}_k)
+
\vec{w}_k
$$

For differential drive odometry:

$$
f(\vec{x}_{k-1},\vec{u}_k)=
\begin{bmatrix}
x_{k-1}
+
v_k\cos\theta_{k-1}\Delta t
\\
y_{k-1}
+
v_k\sin\theta_{k-1}\Delta t
\\
\theta_{k-1}
+
\omega_k\Delta t
\end{bmatrix}
$$

where $\vec{w}_k$ is process noise.

---

## Nonlinear Measurement Model

The measurement model is:

$$
\vec{z}_k=
h(\vec{x}_k)
+
\vec{v}_k
$$

where:

- $\vec{z}_k$ is the sensor measurement
- $h(\vec{x}_k)$ predicts what the sensor should measure from the state
- $\vec{v}_k$ is measurement noise

Example: if the IMU measures heading only:

$$
z_k=
\theta_k
+
v_k
$$

then:

$$
h(\vec{x}_k)=
\theta_k
$$

---

# Extended Kalman Filter Algorithm

The EKF has two steps:

```text
Prediction
↓
Correction
```

The main difference from the normal Kalman filter is that the EKF uses **Jacobians** to locally linearize the nonlinear functions.

---

## 1. Prediction Step

### Predicted State

$$
\hat{\vec{x}}_{k|k-1}=
f(\hat{\vec{x}}_{k-1|k-1},\vec{u}_k)
$$

For differential drive:

$$
\hat{\vec{x}}_{k|k-1}=
\begin{bmatrix}
\hat{x}_{k-1}
+
v_k\cos\hat{\theta}_{k-1}\Delta t
\\
\hat{y}_{k-1}
+
v_k\sin\hat{\theta}_{k-1}\Delta t
\\
\hat{\theta}_{k-1}
+
\omega_k\Delta t
\end{bmatrix}
$$

---

## State Jacobian

The state Jacobian is:

$$
F_k=
\frac{\partial f}{\partial \vec{x}}
\bigg\rvert_{\hat{\vec{x}}_{k-1|k-1},\vec{u}_k}
$$

For:

$$
f(\vec{x}_{k-1},\vec{u}_k)=
\begin{bmatrix}
x_{k-1}
+
v_k\cos\theta_{k-1}\Delta t
\\
y_{k-1}
+
v_k\sin\theta_{k-1}\Delta t
\\
\theta_{k-1}
+
\omega_k\Delta t
\end{bmatrix}
$$

we get:

$$
F_k=
\begin{bmatrix}
1 & 0 & -v_k\sin\theta_{k-1}\Delta t \\
0 & 1 & v_k\cos\theta_{k-1}\Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

At the estimate:

$$
F_k=
\begin{bmatrix}
1 & 0 & -v_k\sin\hat{\theta}_{k-1}\Delta t \\
0 & 1 & v_k\cos\hat{\theta}_{k-1}\Delta t \\
0 & 0 & 1
\end{bmatrix}
$$

---

## Predicted Covariance

$$
P_{k|k-1}=
F_kP_{k-1|k-1}F_k^T
+
Q_k
$$

where:

- $P$ is estimate uncertainty
- $Q_k$ is process noise covariance

---

# 2. Correction Step

## Measurement Residual

$$
\vec{y}_k=
\vec{z}_k-
h(\hat{\vec{x}}_{k|k-1})
$$

This means:

```text
residual
=
actual measurement
-
predicted measurement
```

---

## Measurement Jacobian

The measurement Jacobian is:

$$
H_k=
\frac{\partial h}{\partial \vec{x}}
\bigg\rvert_{\hat{\vec{x}}_{k|k-1}}
$$

For IMU heading measurement:

$$
h(\vec{x}_k)=
\theta_k
$$

So:

$$
H_k=
\begin{bmatrix}
0 & 0 & 1
\end{bmatrix}
$$

---

## Innovation Covariance

$$
S_k=
H_kP_{k|k-1}H_k^T
+
R_k
$$

where $R_k$ is measurement noise covariance.

---

## Kalman Gain

$$
K_k=
P_{k|k-1}H_k^TS_k^{-1}
$$

---

## Updated State Estimate

$$
\hat{\vec{x}}_{k|k}=
\hat{\vec{x}}_{k|k-1}
+
K_k\vec{y}_k
$$

---

## Updated Covariance

Simple form:

$$
P_{k|k}=
(I-K_kH_k)P_{k|k-1}
$$

More stable Joseph form:

$$
P_{k|k}=
(I-K_kH_k)P_{k|k-1}(I-K_kH_k)^T
+
K_kR_kK_k^T
$$

---

# Example: Differential Drive Robot With IMU Heading

## Prediction

Use wheel encoder odometry to predict motion:

$$
\hat{x}_{k|k-1}=
\hat{x}_{k-1}
+
v_k\cos\hat{\theta}_{k-1}\Delta t
$$

$$
\hat{y}_{k|k-1}=
\hat{y}_{k-1}
+
v_k\sin\hat{\theta}_{k-1}\Delta t
$$

$$
\hat{\theta}_{k|k-1}=
\hat{\theta}_{k-1}
+
\omega_k\Delta t
$$

---

## Correction

Use IMU heading measurement:

$$
z_k=
\theta_{IMU,k}
$$

Measurement model:

$$
h(\hat{\vec{x}}_{k|k-1})=
\hat{\theta}_{k|k-1}
$$

Residual:

$$
y_k=
\theta_{IMU,k}-
\hat{\theta}_{k|k-1}
$$

Then the EKF uses this heading error to correct the full state estimate.

---

# Notes

A normal Kalman filter works with linear models:

$$
\vec{x}_k=
A\vec{x}_{k-1}
+
B\vec{u}_k
+
\vec{w}_k
$$

An Extended Kalman Filter works with nonlinear models:

$$
\vec{x}_k=
f(\vec{x}_{k-1},\vec{u}_k)
+
\vec{w}_k
$$

and uses Jacobians:

$$
F_k=
\frac{\partial f}{\partial \vec{x}}
$$

$$
H_k=
\frac{\partial h}{\partial \vec{x}}
$$

to approximate the nonlinear system locally.