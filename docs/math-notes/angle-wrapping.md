# Angle Wrapping

## Goal

Angle wrapping keeps an angle inside a chosen range.

For robot heading control, a common range is:

$$
-\pi < \theta \leq \pi
$$

or in degrees:

$$
-180^\circ < \theta \leq 180^\circ
$$

This prevents the robot from turning the long way around.

---

## Why It Is Needed

Suppose the target angle is:

$$
\theta_{target}=179^\circ
$$

and the current angle is:

$$
\theta_{current}=-179^\circ
$$

A direct subtraction gives:

$$
e=
179^\circ - (-179^\circ)=
358^\circ
$$

But the shortest rotation is actually:

$$
-2^\circ
$$

So the angle error must be wrapped.

---

## Angle Error

Define the raw error:

$$
e_{raw}=
\theta_{target}-
\theta_{current}
$$

Then wrap it:

$$
e=
\text{wrap}(e_{raw})
$$

---

## Wrapping to $(-\pi,\pi]$

A common formula is:

$$
\text{wrap}(\theta)=
\theta-
2\pi
\left\lfloor
\frac{\theta+\pi}{2\pi}
\right\rfloor
$$

This maps the angle into:

$$
-\pi \leq \theta < \pi
$$

Depending on convention, the endpoint may be written differently.

---

## Programming Version

In C++:

```cpp
double wrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}
```

This returns an angle in:

$$
-\pi < \theta \leq \pi
$$

---

## Example

Target:

$$
\theta_{target}=179^\circ
$$

Current:

$$
\theta_{current}=-179^\circ
$$

Raw error:

$$
e_{raw}=
358^\circ
$$

Wrapped error:

$$
e=
-2^\circ
$$

So the robot turns slightly clockwise instead of almost one full circle counterclockwise.

---

## Robotics Use

Angle wrapping is used in:

- heading control
- odometry
- IMU yaw processing
- turn-to-angle PID
- path following

Example:

$$
e_\theta=
\operatorname{wrap}
(\theta_{target}-\theta_{robot})
$$

Then the heading PID uses $e_\theta$ instead of the raw angle difference.
