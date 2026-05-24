# 2D Kalman Filter Math

## State Vector

For 2D motion, the state can include position and velocity:

$$
\vec{x}_k=
\begin{bmatrix}
x_k \\
y_k \\
v_{x,k} \\
v_{y,k}
\end{bmatrix}
$$

where:

- $x_k$ is x-position
- $y_k$ is y-position
- $v_{x,k}$ is x-velocity
- $v_{y,k}$ is y-velocity

---

## State Transition Model

Assume constant velocity during one time step $\Delta t$:

$$
x_k=
x_{k-1}
+
v_{x,k-1}\Delta t
$$

$$
y_k=
y_{k-1}
+
v_{y,k-1}\Delta t
$$

$$
v_{x,k}=
v_{x,k-1}
$$

$$
v_{y,k}=
v_{y,k-1}
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
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Measurement Model

If the sensor measures position only:

$$
\vec{z}_k=
\begin{bmatrix}
z_{x,k} \\
z_{y,k}
\end{bmatrix}
$$

then:

$$
\vec{z}_k=
H\vec{x}_k
+
\vec{v}_k
$$

where:

$$
H=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
$$

because:

$$
\begin{bmatrix}
z_{x,k} \\
z_{y,k}
\end{bmatrix}=
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
x_k \\
y_k \\
v_{x,k} \\
v_{y,k}
\end{bmatrix}
$$

---

## Prediction Step

### Predicted State

$$
\hat{\vec{x}}_{k|k-1}=
A\hat{\vec{x}}_{k-1|k-1}
$$

### Predicted Covariance

$$
P_{k|k-1}=
AP_{k-1|k-1}A^T
+
Q
$$

---

## Correction Step

### Measurement Residual

$$
\vec{y}_k=
\vec{z}_k-
H\hat{\vec{x}}_{k|k-1}
$$

### Innovation Covariance

$$
S_k=
HP_{k|k-1}H^T
+
R
$$

### Kalman Gain

$$
K_k=
P_{k|k-1}H^TS_k^{-1}
$$

### Updated State Estimate

$$
\hat{\vec{x}}_{k|k}=
\hat{\vec{x}}_{k|k-1}
+
K_k\vec{y}_k
$$

### Updated Covariance

$$
P_{k|k}=
(I-K_kH)P_{k|k-1}
$$

---

## Meaning of Matrices

### State covariance

$$
P
\in
\mathbb{R}^{4\times4}
$$

because the state has 4 variables:

$$
x,\ y,\ v_x,\ v_y
$$

### Process noise

$$
Q
\in
\mathbb{R}^{4\times4}
$$

describes uncertainty in the motion model.

### Measurement noise

$$
R
\in
\mathbb{R}^{2\times2}
$$

because the sensor measures 2 values:

$$
z_x,\ z_y
$$

---

## Robotics Interpretation

Prediction:

```text
Use previous position and velocity to predict new position.
```

Correction:

```text
Use sensor measurement to correct the predicted position.
```

Example:

```text
Prediction:
x = 1.05 m, y = 0.98 m

Sensor:
x = 1.00 m, y = 1.02 m

Kalman filter:
combines both based on uncertainty
```