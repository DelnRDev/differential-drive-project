# Differential Drive Odometry

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

## Wheel Velocity Vector

$$
\vec{v}_{wheel}=
\begin{bmatrix}
v_R \\
v_L
\end{bmatrix}
$$

---

## Body Velocity

The robot center linear velocity and angular velocity are:

$$
\vec{u}_k=
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
$$

with:

$$
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}=
\begin{bmatrix}
\frac{1}{2} & \frac{1}{2} \\
\frac{1}{2L} & -\frac{1}{2L}
\end{bmatrix}
\begin{bmatrix}
v_R \\
v_L
\end{bmatrix}
$$

where $L$ is the half-distance between the left and right wheels.

---

## Rotation Matrix

The robot body-frame forward velocity is transformed into the world frame using:

$$
R_{WB}(\theta_k)=
\begin{bmatrix}
\cos(\theta_k) & -\sin(\theta_k) \\
\sin(\theta_k) & \cos(\theta_k)
\end{bmatrix}
$$

For a differential drive robot, the body-frame velocity is:

$$
[\vec{v}]_B=
\begin{bmatrix}
v_k \\
0
\end{bmatrix}
$$

So the world-frame velocity is:

$$
[\vec{v}]_W=
R_{WB}(\theta_k)
[\vec{v}]_B
$$

Therefore:

$$
[\vec{v}]_W=
\begin{bmatrix}
v_k\cos(\theta_k) \\
v_k\sin(\theta_k)
\end{bmatrix}
$$

---

## Continuous State Equation

$$
\frac{d}{dt}
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}=
\begin{bmatrix}
v\cos(\theta) \\
v\sin(\theta) \\
\omega
\end{bmatrix}
$$

---

## Discrete Euler Update

Using Euler integration:

$$
\vec{x}_{k+1}=
\vec{x}_k
+
\Delta t
\begin{bmatrix}
v_k\cos(\theta_k) \\
v_k\sin(\theta_k) \\
\omega_k
\end{bmatrix}
$$

Expanded:

$$
\begin{bmatrix}
x_{k+1} \\
y_{k+1} \\
\theta_{k+1}
\end{bmatrix}=
\begin{bmatrix}
x_k \\
y_k \\
\theta_k
\end{bmatrix}
+
\Delta t
\begin{bmatrix}
v_k\cos(\theta_k) \\
v_k\sin(\theta_k) \\
\omega_k
\end{bmatrix}
$$