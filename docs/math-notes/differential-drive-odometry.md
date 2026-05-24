# Differential Drive Odometry

## Definitions

- $v_L$: left wheel velocity
- $v_R$: right wheel velocity
- $L$: distance between wheels
- $(x,y)$: robot position
- $\theta$: heading

---

## Linear Velocity

Robot center velocity:

$$
v=
\frac{v_R+v_L}{2}
$$

---

## Angular Velocity

Robot rotational velocity:

$$
\omega =
\frac{v_R-v_L}
{L}
$$

---

## State Equations

Continuous system:

$$
\frac{dx}{dt}=
v \cdot \cos(\theta)
$$

$$
\frac{dy}{dt}=
v \cdot \sin(\theta)
$$

$$
\frac{d\theta}{dt}=
\omega
$$

---

## Discrete Update

Using Euler integration:

$$
x_{k+1}=
x_k
+
v_k \cdot
\cos(\theta_k) \cdot
\Delta t
$$

$$
y_{k+1}=
y_k
+
v_k \cdot
\sin(\theta_k) \cdot
\Delta t
$$

$$
\theta_{k+1}=
\theta_k
+
\omega_k\Delta t
$$