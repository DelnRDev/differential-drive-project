# PID Controller

## Error Definition

Target value:

$$
r(t)
$$

Measured value:

$$
y(t)
$$

Error:

$$
e(t)=
r(t)-y(t)
$$

---

## Proportional Term

Control output proportional to present error:

$$
u_P(t)=
K_P \cdot e(t)
$$

---

## Integral Term

Control output based on accumulated error:

$$
u_I(t)=
K_I \cdot
\int_0^t
e(\tau)d\tau
$$

---

## Derivative Term

Control output based on error rate of change:

$$
u_D(t)=
K_D \cdot
\frac{de(t)}{dt}
$$

---

## Complete PID Controller

$$
u(t)=
K_P \cdot e(t)
+
K_I \cdot
\int_0^t
e(\tau)d\tau
+
K_D \cdot
\frac{de(t)}{dt}
$$

---

## Discrete implementation

Sample period:

$$
\Delta t
$$

Approximate integral:

$$
I_k=
I_{k-1}
+
e_k \cdot \Delta t
$$

Approximate derivative:

$$
D_k=
\frac{e_k-e_{k-1}}
{\Delta t}
$$

Discrete controller:

$$
u_k=
K_P \cdot e_k
+
K_I \cdot I_k
+
K_D \cdot D_k
$$