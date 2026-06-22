# Differential Drive Odometry

## Goal

Estimate the robot position and heading using the left and right wheel motion.

A differential drive robot has two independently driven wheels:

- left wheel
- right wheel

By comparing how far each wheel moves, the robot can estimate:

- forward motion
- turning motion
- position
- heading

---

## State Vector

The robot pose is:

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

The same pose can be written as:

$$
T_{WB,k}=
\begin{bmatrix}
\cos\theta_k & -\sin\theta_k & x_k \\
\sin\theta_k & \cos\theta_k & y_k \\
0 & 0 & 1
\end{bmatrix}
$$

where $T_{WB,k}$ transforms coordinates from the robot body frame $B$ to the world frame $W$.

---

## Wheel Distances

Let:

- $\Delta s_R$: distance traveled by the right wheel
- $\Delta s_L$: distance traveled by the left wheel
- $L$: half-distance between wheels

The center distance traveled by the robot is:

$$
\Delta s=
\frac{\Delta s_R+\Delta s_L}{2}
$$

The heading change is:

$$
\Delta\theta=
\frac{\Delta s_R-\Delta s_L}{2L}
$$

---

## Wheel Velocities

If the wheel velocities are:

$$
\vec{v}_{wheel,k}=
\begin{bmatrix}
v_{R,k} \\
v_{L,k}
\end{bmatrix}
$$

then:

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

## Body Frame Velocity

In the robot body frame:

$$
[\vec{v}_k]_B=
\begin{bmatrix}
v_k \\
0
\end{bmatrix}
$$

The robot only moves forward/backward in its own body x-direction.

---

## Rotation to World Frame

The rotation from body frame to world frame is:

$$
R_{WB}(\theta_k)=
\begin{bmatrix}
\cos\theta_k & -\sin\theta_k \\
\sin\theta_k & \cos\theta_k
\end{bmatrix}
$$

The world-frame velocity is:

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

## Continuous-Time Model

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

Using sample time $\Delta t$:

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

## Distance-Based Update

Using wheel distance instead of velocity:

$$
x_{k+1}=
x_k
+
\Delta s\cos\theta_k
$$

$$
y_{k+1}=
y_k
+
\Delta s\sin\theta_k
$$

$$
\theta_{k+1}=
\theta_k
+
\Delta\theta
$$

where:

$$
\Delta s=
\frac{\Delta s_R+\Delta s_L}{2}
$$

and:

$$
\Delta\theta=
\frac{\Delta s_R-\Delta s_L}{2L}
$$

---

## Midpoint Update

A better approximation uses the heading halfway through the motion:

$$
\theta_{mid}=
\theta_k
+
\frac{\Delta\theta}{2}
$$

Then:

$$
x_{k+1}=
x_k
+
\Delta s\cos\theta_{mid}
$$

$$
y_{k+1}=
y_k
+
\Delta s\sin\theta_{mid}
$$

$$
\theta_{k+1}=
\theta_k
+
\Delta\theta
$$

This is usually more accurate than using only $\theta_k$.

---

## Updating the Homogeneous Pose

After computing $x_{k+1}$, $y_{k+1}$, and $\theta_{k+1}$:

$$
T_{WB,k+1}=
\begin{bmatrix}
\cos\theta_{k+1} & -\sin\theta_{k+1} & x_{k+1} \\
\sin\theta_{k+1} & \cos\theta_{k+1} & y_{k+1} \\
0 & 0 & 1
\end{bmatrix}
$$

---

## C++ Implementation

```cpp
struct Pose {
    double x;
    double y;
    double theta;
};

double wrapAngle(double angle);

void updateOdometry(
    Pose& pose,
    double deltaRight,
    double deltaLeft,
    double halfTrackWidth
) {
    double deltaS = (deltaRight + deltaLeft) / 2.0;
    double deltaTheta = (deltaRight - deltaLeft) / (2.0 * halfTrackWidth);

    double thetaMid = pose.theta + deltaTheta / 2.0;

    pose.x += deltaS * cos(thetaMid);
    pose.y += deltaS * sin(thetaMid);
    pose.theta = wrapAngle(pose.theta + deltaTheta);
}
```

---

## Notes

- Encoder errors accumulate over time.
- Wheel slip causes odometry drift.
- Unequal wheel diameters cause heading error.
- IMU heading can help correct drift.
- Odometry gives an estimate, not perfect truth.