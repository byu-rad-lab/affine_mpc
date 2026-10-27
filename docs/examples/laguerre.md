# Laguerre Basis

`examples/laguerre.py` parameterizes the input of the mass-spring-damper with discrete Laguerre functions instead of a B-spline, using `Parameterization.fromBasis()`.
It runs two forms of the controller in closed loop and prints the final position and tracking cost of each.

## What the Example Demonstrates

- Building a sampled basis matrix (discrete Laguerre functions) and passing it to `Parameterization.fromBasis()`
- Preparing a custom basis: zeroing the rows that have decayed to round-off and orthonormalizing the columns with `numpy.linalg.qr`
- The deviation form $\mathbf{u} = \mathbf{u}_{\mathrm{ss}} + \Psi \mathbf{c}$: the steady-state input $\mathbf{u}_{\mathrm{ss}}$ is computed outside the library and enters through the model offset $\mathbf{w}$ and shifted input limits
- The direct form $\mathbf{u} = \Psi \mathbf{c}$ for comparison, whose planned input decays to zero over the horizon

[Custom Bases](../concepts/input-parameterization.md#custom-bases) explains the forms, the requirements on a custom basis, and how a custom basis changes the input limits, input cost, and slew rates.

## Run the Example

The example is Python-only and needs NumPy; it prints its results and does not log or plot.

```sh
python examples/laguerre.py
```

## Results

```text
direct    form: final position 0.8697, cost 11.395
deviation form: final position 1.0000, cost 9.129
```

The reference position is 1.
The direct form settles short of it, because an input that decays to zero can not hold the steady force the spring needs; the deviation form reaches it at a lower cost.
