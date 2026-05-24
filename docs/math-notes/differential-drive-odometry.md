# Differential Drive Odometry

## State Vector

The robot pose can be written as:

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

## Homogeneous Pose Matrix

The same pose can also be represented using a homogeneous transformation matrix:

$$
T_{WB,k}=
\begin{bmatrix}
\cos\theta_k & -\sin\theta_k & x_k \\
\sin\theta_k & \cos\theta_k & y_k \\
0 & 0 & 1
\end{bmatrix}
$$

where:

- $W$ is the world frame
- $B$ is the robot body frame
- $T_{WB,k}$ transforms coordinates from the robot body frame to the world frame

A point written in the robot body frame can be transformed into the world frame by:

$$
\begin{bmatrix}
p_x \\
p_y \\
1
\end{bmatrix}_W=
T_{WB,k}
\begin{bmatrix}
p_x \\
p_y \\
1
\end{bmatrix}_B
$$

---

## Wheel Velocity Vector

$$
\vec{v}_{wheel,k}=
\begin{bmatrix}
v_{R,k} \\
v_{L,k}
\end{bmatrix}
$$

where:

- $v_{R,k}$ is the right wheel velocity
- $v_{L,k}$ is the left wheel velocity

---

## Body Velocity

The robot linear velocity and angular velocity are:

$$
\vec{u}_k=
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
$$

The wheel velocities can be converted into body velocity by:

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
v_{R,k} \\
v_{L,k}
\end{bmatrix}
$$

where $L$ is the half-distance between the left and right wheels.

So:

$$
v_k=
\frac{v_{R,k}+v_{L,k}}{2}
$$

and:

$$
\omega_k=
\frac{v_{R,k}-v_{L,k}}{2L}
$$

---

## Rotation Matrix

The rotation from the robot body frame to the world frame is:

$$
R_{WB}(\theta_k)=
\begin{bmatrix}
\cos\theta_k & -\sin\theta_k \\
\sin\theta_k & \cos\theta_k
\end{bmatrix}
$$

In the robot body frame, the robot moves forward with velocity:

$$
[\vec{v}_k]_B=
\begin{bmatrix}
v_k \\
0
\end{bmatrix}
$$

Transforming this velocity into the world frame:

$$
[\vec{v}_k]_W=
R_{WB}(\theta_k)
[\vec{v}_k]_B
$$

Therefore:

$$
[\vec{v}_k]_W=
\begin{bmatrix}
v_k\cos\theta_k \\
v_k\sin\theta_k
\end{bmatrix}
$$

---

## Continuous-Time State Equation

The continuous-time odometry model is:

$$
\frac{d}{dt}
\begin{bmatrix}
x(t) \\
y(t) \\
\theta(t)
\end{bmatrix}=
\begin{bmatrix}
v(t)\cos\theta(t) \\
v(t)\sin\theta(t) \\
\omega(t)
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
v_k\cos\theta_k \\
v_k\sin\theta_k \\
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
v_k\cos\theta_k \\
v_k\sin\theta_k \\
\omega_k
\end{bmatrix}
$$

So:

$$
x_{k+1}=
x_k
+
v_k\cos\theta_k \Delta t
$$

$$
y_{k+1}=
y_k
+
v_k\sin\theta_k \Delta t
$$

$$
\theta_{k+1}=
\theta_k
+
\omega_k \Delta t
$$

---

## Homogeneous Transform Update

The pose matrix can be updated from the new state:

$$
T_{WB,k+1}=
\begin{bmatrix}
\cos\theta_{k+1} & -\sin\theta_{k+1} & x_{k+1} \\
\sin\theta_{k+1} & \cos\theta_{k+1} & y_{k+1} \\
0 & 0 & 1
\end{bmatrix}
$$

---

## Notes

In code, the robot pose can still be stored simply as:

```cpp
double x;
double y;
double theta;
```

The homogeneous matrix form is mainly useful for understanding coordinate frames and transforming points between the robot frame and the world frame.