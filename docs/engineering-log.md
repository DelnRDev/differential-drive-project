# Engineering Log

---

## Date: May 23, 2026

## Encoder RPM Test
### Goal

Test whether the encoders can measure steady-state RPM reliably.

### Summary

| Trial | Left RPM | Right RPM |
|---:|---:|---:|
| 1 | 291.0 | 292.5 |
| 2 | 290.7 | 287.4 |
| 3 | 289.8 | 287.2 |
| 4 | 290.0 | 286.5 |
| 5 | 291.0 | 289.5 |

## Conclusion

The encoder readings became stable after using a 500 μs minimum pulse interval and a 5 second settle time. Both motors measured around 290 RPM at the tested PWM.

### Detailed Test

[Encoder RPM Test](tests/encoder-rpm-test.md)

### Next Step

Test live RPM measurement using 50 ms and 100 ms update periods.
