# Field-Oriented Control of an Induction Motor

Indirect field-oriented control (vector control) of a 5 kW / 380 V
squirrel-cage induction motor, implemented three times on the same machine
and the same test scenario:

| Implementation | Folder | Inverter |
|---|---|---|
| Python simulation | `python/` | average-value SVPWM (switching model optional) |
| Simulink, fundamental blocks only | `matlab/simulink_blocks/` | average-value SVPWM |
| Simulink, Simscape Electrical | `matlab/simulink_library/` | Universal Bridge, true 10 kHz switching |

The three agree on every physical quantity, which is the point: the hand-built
model validates the library one and vice versa.

**[REPORT.pdf](REPORT.pdf)** is the full write-up: machine equations,
Clarke/Park transforms, IFOC structure, controller tuning, results with
commentary, the Simulink block diagrams and the three-way comparison.

## The drive

* Machine modelled by its two-axis (dq) equations in the **stationary frame**,
  controller running in its own **estimated flux frame** - so an orientation
  error is visible instead of being assumed away.
* **Indirect** rotor-flux orientation: the flux angle comes from slip
  integration, the flux magnitude from a current-model estimator. No flux
  sensor.
* Cascaded control: speed PI -> torque -> `i_q*`, flux -> `i_d*`, two inner
  current PIs with decoupling feed-forward, all with back-calculation
  anti-windup and voltage/current limiting.
* Amplitude-invariant (2/3) transforms, so the torque equation carries the
  3/2 factor.

## Machine parameters

| Symbol | Value | | Symbol | Value |
|---|---|---|---|---|
| Rs | 0.435 Ω | | p | 2 pole pairs |
| Rr | 0.816 Ω | | J | 0.0056 kg·m² |
| Lls = Llr | 2 mH | | B | 0.0015 N·m·s |
| Lm | 69 mH | | Vdc | 540 V |

Derived: σ = 0.0555, τr = 87 ms, σLs = 3.94 mH. Control at 10 kHz
(Ts = 100 µs), ψ* = 0.8 Wb, Imax = 25 A, Te,max = 35 N·m.

## Running it

### Python

```powershell
cd python
pip install -r requirements.txt
python main.py
```

Every input - motor data, gains, references, scenario - is at the top of
`main.py`. Figures are written to `python/outputs/graphs/`.

Two scenarios run in sequence: a four-quadrant staircase (flux build-up,
steps to 800, 1400 and 600 rpm, reversal to -800 rpm, 10 N·m load step) and
an unloaded ramp + sine tracking study used to check the PI speed loop
against the internal model principle.

### MATLAB / Simulink (R2024b)

```matlab
cd matlab\simulink_blocks
run_foc_blocks        % seconds

cd ..\simulink_library
run_foc_library       % needs Simscape Electrical, ~2-3 min
```

Each run script loads the parameters (`m_files/foc_init.m`), simulates,
prints the performance metrics and saves figures to `matlab/outputs/graphs/`.

## Results

| | Python | Simulink V1 | Simulink V2 |
|---|---|---|---|
| Step settling (±10 rpm) | 79 / 75 / 71 / 95 ms | 70 / 68 / 52 / 59 ms | 63 / 62 / 67 / 72 ms |
| Speed error between steps | 0.25 rpm max | 0.16 rpm max | 4.9 rpm (PWM ripple) |
| Load dip (10 N·m) | 69.0 rpm | 69.2 rpm | 69.6 rpm |
| Rotor flux | 0.800 Wb | 0.800 Wb | 0.800 Wb |
| Peak phase current | 19.1 A | 19.9 A | 20.6 A |

No overshoot on any step, q-axis flux residual below 2 % throughout, and the
load disturbance is rejected in about 70 ms. Differences between the columns
are explained by continuous vs discrete control and averaged vs switching
inverter.

## Limitations

Constant flux, so the results hold below base speed - field weakening is not
implemented. Indirect FOC binds the orientation to the rotor time constant
τr, which drifts with rotor temperature; τr adaptation and sensorless
operation are the natural next steps.

## License

MIT - see [LICENSE](LICENSE).
