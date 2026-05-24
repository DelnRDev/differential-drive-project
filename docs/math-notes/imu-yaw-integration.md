# IMU Yaw Integration

## Goal

Use the IMU gyroscope z-axis measurement to estimate robot heading.

For a flat ground robot, yaw is rotation around the vertical axis.

---

## Discrete-Time Signal Notation

Let the gyro z-axis angular velocity signal be:

$$
\omega_z[n]
$$

where:

- $n$ is the discrete-time sample index
- $\omega_z[n]$ is measured in rad/s

The estimated yaw angle is:

$$
\theta[n]
$$

---

## Continuous-Time Relationship

Angular velocity is the time derivative of angle:

$$
\omega_z(t)=
\frac{d\theta(t)}{dt}
$$

So yaw angle can be recovered by integration:

$$
\theta(t)=
\theta(0)
+
\int_0^t
\omega_z(\tau)
\,d\tau
$$

---

## Discrete Euler Integration

With sample time:

$$
\Delta t
$$

Euler integration gives:

$$
\theta[n]=
\theta[n-1]
+
\omega_z[n]\Delta t
$$

---

## Gyro Bias

Real IMUs usually have bias:

$$
\omega_{measured}[n]=
\omega_{true}[n]
+
b
+
\eta[n]
$$

where:

- $b$ is gyro bias
- $\eta[n]$ is measurement noise

Estimate and subtract the bias:

$$
\omega_{corrected}[n]=
\omega_{measured}[n]-
b
$$

Then integrate:

$$
\theta[n]=
\theta[n-1]
+
\omega_{corrected}[n]\Delta t
$$

---

## Bias Calibration

If the robot is stationary, the true angular velocity should be:

$$
\omega_{true}[n]=
0
$$

So the average measured gyro value estimates the bias:

$$
b=
\frac{1}{N}
\sum_{k=0}^{N-1}
\omega_{measured}[k]
$$

---

## Angle Wrapping

After integration, wrap the heading:

$$
\theta[n]=
\operatorname{wrap}
(\theta[n])
$$

For example:

$$
-\pi < \theta[n] \leq \pi
$$

---

## C++ Implementation

```cpp
double estimateGyroBias(double samples[], int sampleCount) {
    double sum = 0.0;

    for (int i = 0; i < sampleCount; i++) {
        sum += samples[i];
    }

    return sum / sampleCount;
}

double updateYaw(double previousYaw, double gyroZ, double gyroBias, double dt) {
    double correctedGyroZ = gyroZ - gyroBias;

    double yaw = previousYaw + correctedGyroZ * dt;

    return wrapAngle(yaw);
}
```

---

## Notes

Gyro integration drifts over time because even a small bias accumulates.

For example, if the gyro bias error is:

$$
0.01 \text{ rad/s}
$$

then after:

$$
100 \text{ s}
$$

the yaw error becomes approximately:

$$
1 \text{ rad}
$$

This is why IMU yaw is often corrected using:

- magnetometer
- wheel odometry
- external heading reference
- complementary filter
- Extended Kalman Filter