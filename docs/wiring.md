# Wiring

## Power Notes

- All modules must share common GND.
- Check voltage requirements before connecting power.
- Do not power 3.3V sensors from 5V unless they support it.

---

## Motor Driver

| Motor Driver Pin | Connects To | Purpose |
|---|---|---|
| VCC / VM | Motor battery + | Motor power |
| GND | Battery - and Arduino GND | Common ground |
| IN1 | Arduino pin __ | Left motor direction |
| IN2 | Arduino pin __ | Left motor direction |
| IN3 | Arduino pin __ | Right motor direction |
| IN4 | Arduino pin __ | Right motor direction |
| ENA | Arduino PWM pin __ | Left motor speed |
| ENB | Arduino PWM pin __ | Right motor speed |

---

## Encoders

| Encoder Pin | Connects To | Purpose |
|---|---|---|
| VCC | 5V or 3.3V | Encoder power |
| GND | GND | Ground |
| A | Arduino interrupt pin __ | Encoder signal A |
| B | Arduino pin __ | Encoder signal B |

---

## IMU / MPU6050

| MPU6050 Pin | Connects To |
|---|---|
| VCC | 5V or 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

---

## OLED Display

| OLED Pin | Connects To |
|---|---|
| VCC | 5V or 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

---

## Notes

- I2C devices can share SDA and SCL.
- Encoders need reliable signal wires.
- Motor power should not come directly from the Arduino 5V pin.
- If readings are noisy, check ground connections first.