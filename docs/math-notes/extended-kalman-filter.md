# Extended Kalman Filter Math

## Goal

Estimate the state of a nonlinear system.

For a differential drive robot, the motion is nonlinear because position update depends on heading:

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

---

## State Vector

$$
\vec{x}_k=
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix}
$$

where:

- $x_k$: robot x-position
- $y_k$: robot y-position
- $\theta_k$: robot heading

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

- $v_k$: linear velocity
- $\omega_k$: angular velocity

---

## Nonlinear Motion Model

$$
\vec{x}_k=
f(\vec{x}_{k-1},\vec{u}_k)
+
\vec{w}_k
$$

For differential drive:

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

---

## Measurement Model

$$
\vec{z}_k=
h(\vec{x}_k)
+
\vec{v}_k
$$

Example: IMU measures heading only.

$$
z_k=
\theta_k
+
v_k^{noise}
$$

So:

$$
h(\vec{x}_k)=
\theta_k
$$

---

# EKF Algorithm

The EKF has two steps:

```text
Prediction
↓
Correction
```

---

## Prediction Step

Predicted state:

$$
\hat{\vec{x}}_{k|k-1}=
f(\hat{\vec{x}}_{k-1|k-1},\vec{u}_k)
$$

Expanded:

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

The EKF linearizes the nonlinear motion model using a Jacobian.

$$
F_k=
\frac{\partial f}{\partial \vec{x}}
$$

For the differential drive model:

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

- $P$: estimate uncertainty
- $Q_k$: process noise covariance

---

## Correction Step

Measurement residual:

$$
\vec{y}_k=
\vec{z}_k-
h(\hat{\vec{x}}_{k|k-1})
$$

---

## Measurement Jacobian

$$
H_k=
\frac{\partial h}{\partial \vec{x}}
$$

For heading-only measurement:

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

Joseph form:

$$
P_{k|k}=
(I-K_kH_k)P_{k|k-1}(I-K_kH_k)^T
+
K_kR_kK_k^T
$$

---

## C++ Implementation

```cpp
#include <math.h>

struct EKF {
    double x;
    double y;
    double theta;

    double P[3][3];
    double Q[3][3];
    double R;
};

double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

void predict(EKF& ekf, double v, double omega, double dt) {
    double theta = ekf.theta;

    ekf.x += v * cos(theta) * dt;
    ekf.y += v * sin(theta) * dt;
    ekf.theta = wrapAngle(ekf.theta + omega * dt);

    double F[3][3] = {
        {1.0, 0.0, -v * sin(theta) * dt},
        {0.0, 1.0,  v * cos(theta) * dt},
        {0.0, 0.0,  1.0}
    };

    double FP[3][3] = {};
    double FPF_T[3][3] = {};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * ekf.P[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                FPF_T[i][j] += FP[i][k] * F[j][k];
            }

            ekf.P[i][j] = FPF_T[i][j] + ekf.Q[i][j];
        }
    }
}

void updateHeading(EKF& ekf, double measuredTheta) {
    double residual = wrapAngle(measuredTheta - ekf.theta);

    double S = ekf.P[2][2] + ekf.R;

    if (S == 0.0) {
        return;
    }

    double K0 = ekf.P[0][2] / S;
    double K1 = ekf.P[1][2] / S;
    double K2 = ekf.P[2][2] / S;

    ekf.x += K0 * residual;
    ekf.y += K1 * residual;
    ekf.theta = wrapAngle(ekf.theta + K2 * residual);

    double oldP[3][3];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            oldP[i][j] = ekf.P[i][j];
        }
    }

    for (int j = 0; j < 3; j++) {
        ekf.P[0][j] = oldP[0][j] - K0 * oldP[2][j];
        ekf.P[1][j] = oldP[1][j] - K1 * oldP[2][j];
        ekf.P[2][j] = oldP[2][j] - K2 * oldP[2][j];
    }
}
```

---

## Notes

A normal Kalman filter uses a linear model:

$$
\vec{x}_k=
A\vec{x}_{k-1}
+
\vec{w}_k
$$

An EKF uses a nonlinear model:

$$
\vec{x}_k=
f(\vec{x}_{k-1},\vec{u}_k)
+
\vec{w}_k
$$

and linearizes it using Jacobians:

$$
F_k=
\frac{\partial f}{\partial \vec{x}}
$$

$$
H_k=
\frac{\partial h}{\partial \vec{x}}
$$