# Field-Oriented Control of an Induction Motor

Indirect field-oriented control (vector control) of a 5 kW / 380 V
squirrel-cage induction motor, implemented three times on the same machine
and the same test scenario:

| Implementation | Folder | Inverter |
|---|---|---|
| Python simulation | `python/` | average-value SVPWM (switching model optional) |
| Simulink, fundamental blocks only | `matlab/simulink_blocks/` | average-value SVPWM with an R–L source and dynamic DC-link capacitor |
| Simulink, Simscape Electrical | `matlab/simulink_library/` | IGBT Universal Bridge with custom SPWM and seven-segment SVPWM at 10 kHz |

The same speed and load test lets the three implementations be checked against
one another. The two custom PWM modes are compared on the same switching bridge.

The [English report](REPORT.pdf) and [French report](REPORT.fr.pdf) explain the
FOC architecture, Clarke/Park transforms, rotor-flux estimation, and both
switching methods. They include the three implementations and a close-up of the
Simscape switching results.

## The drive

* Machine modelled by its two-axis (dq) equations in the **stationary frame**,
  controller running in its own **estimated flux frame** - so an orientation
  error is visible instead of being assumed away.
* **Indirect** rotor-flux orientation: the flux angle comes from slip
  integration, the flux magnitude from a current-model estimator. No flux
  sensor.
* Cascaded control: speed PI to torque to `i_q*`, flux to `i_d*`, and two
  inner current PIs with feed-forward compensation and output limits.
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
an unloaded ramp-and-hold test of the PI speed loop.

### MATLAB / Simulink (R2024b)

```matlab
cd matlab\simulink_blocks
run_foc_blocks        % seconds

cd ..\simulink_library
run_foc_library       % default custom SVPWM
compare_foc_pwm       % run both custom modes on the same Simscape model
```

The run scripts load the parameters, print performance metrics, and save figures
to `matlab/outputs/graphs/`. The comparison also saves [the numeric metrics](matlab/outputs/graphs/library_pwm_metrics.csv)
and [a six-period close-up](matlab/outputs/graphs/library_pwm_zoom.png).

## Results

| | Python | Basic blocks | Simscape SPWM | Simscape SVPWM |
|---|---:|---:|---:|---:|
| Step settling (±10 rpm) | 79 / 75 / 71 / 95 ms | 70 / 68 / 52 / 59 ms | 65.3 / 59.0 / 67.7 / 75.0 ms | 69.1 / 57.5 / 70.3 / 73.1 ms |
| Maximum settled speed error | 0.25 rpm | 0.16 rpm | 1.67 rpm | 2.26 rpm |
| Load dip (10 N·m) | 69.00 rpm | 69.17 rpm | 69.86 rpm | 69.70 rpm |
| Final flux | 0.800 Wb | 0.800 Wb | 0.800 Wb | 0.800 Wb |
| Peak phase current | 19.1 A | 19.92 A | 20.48 A | 20.51 A |

On the 1400 rpm plateau, current ripple RMS was 0.254 A for SPWM and 0.234 A
for SVPWM after detrending each phase in each PWM period. The basic-block DC
link ranged from 536.4 to 544.9 V with the illustrative source components in
`motor_params.m`.

## Rebuilding the reports

With Chrome or Chromium installed, run these commands from the repository root.

```powershell
python -m pip install markdown pymupdf
python tools/make_report_pdf.py
python tools/make_report_pdf.py --language fr
```

The reports use the existing simulation figures and load MathJax to typeset equations.

## Limitations

Constant flux, so the results hold below base speed - field weakening is not
implemented. Indirect FOC binds the orientation to the rotor time constant
τr, which drifts with rotor temperature; τr adaptation and sensorless
operation are the natural next steps.

## License

MIT - see [LICENSE](LICENSE).
