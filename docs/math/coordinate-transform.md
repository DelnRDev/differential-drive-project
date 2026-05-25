# Coordinate Transforms

## Goal

Coordinate transforms let us describe the same point or vector in different reference frames.

For a robot, the two main frames are:

- $W$: world frame
- $B$: robot body frame

---

## Body Frame and World Frame

The world frame is fixed:

```text
W frame:
x-axis points forward in the field
y-axis points left/right on the field
```

The body frame moves with the robot:

```text
B frame:
x-axis points forward from the robot
y-axis points left from the robot
```

---

## Position Vector

A point can be written in the body frame as:

$$
[\vec{p}]_B=
\begin{bmatrix}
p_x \\
p_y
\end{bmatrix}_B
$$

The same point can be written in the world frame as:

$$
[\vec{p}]_W=
\begin{bmatrix}
p_x \\
p_y
\end{bmatrix}_W
$$

Same physical point, different coordinates.

---

## Rotation Matrix

If the robot heading is $\theta$, the rotation from body frame to world frame is:

$$
R_{WB}(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

This converts a vector from body coordinates to world coordinates:

$$
[\vec{v}]_W=
R_{WB}(\theta)
[\vec{v}]_B
$$

---

## Inverse Rotation

The inverse transform from world frame to body frame is:

$$
R_{BW}(\theta)=
R_{WB}^T(\theta)
$$

So:

$$
R_{BW}(\theta)=
\begin{bmatrix}
\cos\theta & \sin\theta \\
-\sin\theta & \cos\theta
\end{bmatrix}
$$

This converts a vector from world coordinates to body coordinates:

$$
[\vec{v}]_B=
R_{BW}(\theta)
[\vec{v}]_W
$$

---

## Translation

If the robot position in the world frame is:

$$
[\vec{r}_{WB}]_W=
\begin{bmatrix}
x \\
y
\end{bmatrix}
$$

then a point fixed on the robot transforms into the world frame by:

$$
[\vec{p}]_W=
R_{WB}[\vec{p}]_B
+
[\vec{r}_{WB}]_W
$$

This means:

```text
world point
=
rotated body point
+
robot position
```

---

## Homogeneous Transform

Rotation and translation can be combined into one matrix:

$$
T_{WB}=
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
$$

Using homogeneous coordinates:

$$
\begin{bmatrix}
p_x \\
p_y \\
1
\end{bmatrix}_W=
T_{WB}
\begin{bmatrix}
p_x \\
p_y \\
1
\end{bmatrix}_B
$$

---

## Inverse Homogeneous Transform

To transform from world frame back to body frame:

$$
T_{BW}=
T_{WB}^{-1}
$$

For a rigid transform:

$$
T_{BW}=
\begin{bmatrix}
R_{WB}^T & -R_{WB}^T[\vec{r}_{WB}]_W \\
0 & 1
\end{bmatrix}
$$

Expanded:

$$
T_{BW}=
\begin{bmatrix}
\cos\theta & \sin\theta & -x\cos\theta-y\sin\theta \\
-\sin\theta & \cos\theta & x\sin\theta-y\cos\theta \\
0 & 0 & 1
\end{bmatrix}
$$

---

## Transforming Robot Velocity

If the robot moves forward in its own body frame:

$$
[\vec{v}]_B=
\begin{bmatrix}
v \\
0
\end{bmatrix}
$$

then in the world frame:

$$
[\vec{v}]_W=
R_{WB}
[\vec{v}]_B
$$

So:

$$
[\vec{v}]_W=
\begin{bmatrix}
v\cos\theta \\
v\sin\theta
\end{bmatrix}
$$

This is why odometry uses:

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

---

## Example

Suppose the robot is at:

$$
x=2,\quad y=1,\quad \theta=90^\circ
$$

A point is one meter in front of the robot:

$$
[\vec{p}]_B=
\begin{bmatrix}
1 \\
0
\end{bmatrix}
$$

Since:

$$
R_{WB}(90^\circ)=
\begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
$$

Then:

$$
[\vec{p}]_W=
\begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
\begin{bmatrix}
1 \\
0
\end{bmatrix}
+
\begin{bmatrix}
2 \\
1
\end{bmatrix}
$$

$$
[\vec{p}]_W=
\begin{bmatrix}
2 \\
2
\end{bmatrix}
$$

So a point one meter in front of the robot is at world position:

$$
(2,2)
$$

---

## Robotics Use

Coordinate transforms are used for:

- odometry
- path following
- sensor placement
- converting local motion to field motion
- converting world targets into robot-frame errors

Example target error:

$$
[\vec{e}]_W=
[\vec{p}_{target}]_W-
[\vec{p}_{robot}]_W
$$

Convert world error into robot frame:

$$
[\vec{e}]_B=
R_{BW}
[\vec{e}]_W
$$

Then the robot can decide:

```text
target is in front
target is behind
target is left
target is right
```