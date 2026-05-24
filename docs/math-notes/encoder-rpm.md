# Encoder RPM Calculation

## Definitions

- $N$: encoder pulse count
- $N_{rev}$: pulses per revolution
- $\Delta t$: sampling interval

---

## Revolutions during sample period

$$
R =
\frac{N}{N_{rev}}
$$

---

## Revolutions per second

$$
f =
\frac{R}{\Delta t}
$$

Substitute:

$$
f =
\frac{N}
{N_{rev}\Delta t}
$$

---

## RPM

Convert revolutions per second to revolutions per minute:

$$
RPM =
60f
$$

Therefore:

$$
RPM =
\frac{60N}
{N_{rev}\Delta t}
$$

---

## Example

Given:

- $N = 45$
- $N_{rev}=360$
- $\Delta$ $t=0.1s$

$$
RPM =
\frac{60(45)}
{360(0.1)}
$$

$$
RPM=
75
$$