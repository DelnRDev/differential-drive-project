# 2D Kalman Filter Math

## Goal

Estimate 2D position and velocity by combining:

- model prediction
- sensor measurement

---

## State Vector

For 2D motion:

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

Assume constant velocity during one sample interval $\Delta t$:

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
\vec{z}_k
=
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
\vec{y}_k=
\vec{z}_k-
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
K_k\vec{y}_k
$$

Updated covariance:

$$
P_{k|k}=
(I-K_kH)P_{k|k-1}
$$

---

## Matrix Sizes

State vector:

$$
\vec{x}_k
\in
\mathbb{R}^{4\times1}
$$

State transition matrix:

$$
A
\in
\mathbb{R}^{4\times4}
$$

Measurement vector:

$$
\vec{z}_k
\in
\mathbb{R}^{2\times1}
$$

Measurement matrix:

$$
H
\in
\mathbb{R}^{2\times4}
$$

State covariance:

$$
P
\in
\mathbb{R}^{4\times4}
$$

Process noise covariance:

$$
Q
\in
\mathbb{R}^{4\times4}
$$

Measurement noise covariance:

$$
R
\in
\mathbb{R}^{2\times2}
$$

Kalman gain:

$$
K
\in
\mathbb{R}^{4\times2}
$$

---

## Meaning of $P$, $Q$, and $R$

### $P$: Estimate Uncertainty

Large $P$ means the filter is less confident in the current estimate.

### $Q$: Process Noise Covariance

Large $Q$ means the filter trusts the motion model less.

### $R$: Measurement Noise Covariance

Large $R$ means the filter trusts the sensor less.

---

## Robotics Interpretation

Prediction:

```text
Use previous position and velocity to predict the next position.
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

---

## C++ Implementation

```cpp
struct Kalman2D {
    double x;
    double y;
    double vx;
    double vy;

    double P[4][4];
    double Q[4][4];
    double R[2][2];
};

void predict(Kalman2D& kf, double dt) {
    kf.x = kf.x + kf.vx * dt;
    kf.y = kf.y + kf.vy * dt;

    double A[4][4] = {
        {1.0, 0.0, dt,  0.0},
        {0.0, 1.0, 0.0, dt },
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    double AP[4][4] = {};
    double APA_T[4][4] = {};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                AP[i][j] += A[i][k] * kf.P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                APA_T[i][j] += AP[i][k] * A[j][k];
            }

            kf.P[i][j] = APA_T[i][j] + kf.Q[i][j];
        }
    }
}

void update(Kalman2D& kf, double measuredX, double measuredY) {
    double y0 = measuredX - kf.x;
    double y1 = measuredY - kf.y;

    double S00 = kf.P[0][0] + kf.R[0][0];
    double S01 = kf.P[0][1] + kf.R[0][1];
    double S10 = kf.P[1][0] + kf.R[1][0];
    double S11 = kf.P[1][1] + kf.R[1][1];

    double detS = S00 * S11 - S01 * S10;

    if (detS == 0.0) {
        return;
    }

    double invS00 = S11 / detS;
    double invS01 = -S01 / detS;
    double invS10 = -S10 / detS;
    double invS11 = S00 / detS;

    double K[4][2];

    for (int i = 0; i < 4; i++) {
        K[i][0] = kf.P[i][0] * invS00 + kf.P[i][1] * invS10;
        K[i][1] = kf.P[i][0] * invS01 + kf.P[i][1] * invS11;
    }

    kf.x  = kf.x  + K[0][0] * y0 + K[0][1] * y1;
    kf.y  = kf.y  + K[1][0] * y0 + K[1][1] * y1;
    kf.vx = kf.vx + K[2][0] * y0 + K[2][1] * y1;
    kf.vy = kf.vy + K[3][0] * y0 + K[3][1] * y1;

    double oldP[4][4];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            oldP[i][j] = kf.P[i][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf.P[i][j] =
                oldP[i][j]
                - K[i][0] * oldP[0][j]
                - K[i][1] * oldP[1][j];
        }
    }
}
```

---

## Notes

This is a linear Kalman filter because the model is linear:

$$
\vec{x}_k=
A\vec{x}_{k-1}
+
\vec{w}_k
$$

For nonlinear robot motion using heading $\theta$, use an Extended Kalman Filter.