# Engineering Log

## Encoder RPM Test

### Goal

Test whether the encoders can measure steady-state RPM reliably.

### Setup

- PWM: 100
- Encoder PPR: 40
- Minimum pulse interval: 500 μs
- Settle time: 5 s
- Measurement time: 5 s

### Results

| Trial | Left RPM | Right RPM |
|---:|---:|---:|
| 1 | 291.0 | 292.5 |
| 2 | 290.7 | 287.4 |
| 3 | 289.8 | 287.2 |
| 4 | 290.0 | 286.5 |
| 5 | 291.0 | 289.5 |

### Conclusion

The encoder readings became stable after using a 500 μs minimum pulse interval and a 5 second settle time. Both motors measured around 290 RPM at the tested PWM.

### Next Step

Test live RPM measurement using 50 ms and 100 ms update periods.
