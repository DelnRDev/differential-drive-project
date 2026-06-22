# Encoder RPM Test

## Date
2026-05-24

## Branch
`test/encoder-rpm`

---

## Goal

Test whether the motor encoders can measure steady-state RPM reliably.

This test is part of the larger goal of building a VEX-style robotics control stack from low-cost Arduino-compatible hardware.

---

## Test Setup

| Parameter | Value |
|---|---:|
| Encoder PPR | 40 |
| Minimum pulse interval | 500 μs |
| Settle time | 5 s |
| Measurement time | 5 s |
| Motor command | Constant PWM |
| Stop mode | Coast |

---

## Method

The motor was commanded to spin at a constant PWM value.  
Before measuring RPM, the motor was allowed to settle for 5 seconds.

After the settle time:

1. Encoder count was reset.
2. Pulses were counted for 5 seconds.
3. The motor was stopped.
4. Average RPM was calculated from the pulse count.

The RPM formula used was:

$$
RPM=\frac{\text{pulses}}{PPR}\cdot\frac{60}{T}
$$

where:

- $PPR$ is encoder pulses per revolution
- $T$ is the measurement time in seconds

For this test:

$$
PPR=40
$$

$$
T=5s
$$

So:

$$
RPM=\text{pulses}\cdot 0.3
$$

## Results

### Left Motor

| Trial | Pulses | RPM |
|---:|---:|---:|
| 1 | 970 | 291.0 |
| 2 | 969 | 290.7 |
| 3 | 966 | 289.8 |
| 4 | 967 | 290.1 |
| 5 | 970 | 291.0 |

Average:

$$
RPM_{left,avg}\approx290.5
$$

### Right Motor

| Trial | Pulses | RPM |
|---:|---:|---:|
| 1 | 975 | 292.5 |
| 2 | 958 | 287.4 |
| 3 | 957 | 287.1 |
| 4 | 955 | 286.5 |
| 5 | 965 | 289.5 |

Average:

$$
RPM_{right,avg}\approx288.6
$$

## Analysis

The left motor was very consistent across trials, with RPM values around 290 RPM.

The right motor also stayed close to the same range, around 287–293 RPM.

The left and right motor averages were close:

$$
RPM_{left,avg}\approx290.5
$$

$$
RPM_{right,avg}\approx288.6
$$

The difference was approximately:

$$
290.5-288.6=1.9\ RPM
$$

This is small compared to the motor speed, so the encoder readings are consistent enough for the next stage.

## Conclusion

The encoder counting system is reliable after adding a 500 μs minimum pulse interval and using a 5 second settle time.

The steady-state RPM at the tested PWM is approximately:

$$
\boxed{290\ RPM}
$$

This result is stable enough to move on to live RPM measurement and motor speed PID.

## Next Step

Test live RPM measurement using shorter update windows:

- 50 ms
- 100 ms

Then compare the live RPM readings against the 5 second average RPM result.
