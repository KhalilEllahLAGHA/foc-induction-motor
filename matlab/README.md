# MATLAB and Simulink FOC drive

The two R2024b models run the same 2.4 s speed-step and 10 N m load test
using the machine data and controller gains in `m_files/motor_params.m`.
The unloaded ramp-and-hold study is in the Python implementation.

| Model | Power stage | Time base |
|---|---|---|
| `simulink_blocks/foc_im_blocks.slx` | Fundamental blocks, average SVPWM duties, R-L source and DC-link capacitor | Continuous controller and variable-step `ode23tb` |
| `simulink_library/foc_im_library.slx` | Simscape Electrical SPS induction machine and IGBT Universal Bridge, custom SPWM or seven-segment SVPWM | Discrete controller at 100 µs and power stage at 1 µs |

## Run the models

```matlab
% From the matlab folder
cd simulink_blocks
run_foc_blocks
cd ../simulink_library
run_foc_library
compare_foc_pwm
```

`run_foc_library` uses the modulation selected by `P.use_svpwm`, which defaults
to SVPWM. Close the saved model before calling `compare_foc_pwm`. The comparison
runs both modes sequentially and writes the full overlay, six-period zoom,
and numeric CSV to `outputs/graphs/`. Its voltage trace is the ideal A-to-B
voltage implied by the gate signals. Its current and torque are the actual
simulation outputs.

Run `export_diagrams` and `export_pwm_explanations` with `m_files/` on the
MATLAB path to regenerate model snapshots and the PWM teaching figures. The
saved models include hand-edited layouts. The builders overwrite them, so
run the saved models for normal simulation.

## Verified comparison

| Result | Fundamental blocks | Simscape SPWM | Simscape SVPWM |
|---|---:|---:|---:|
| Load-step dip (rpm) | 69.17 | 69.86 | 69.70 |
| Final flux estimate (Wb) | 0.800 | 0.800 | 0.800 |
| Maximum settled speed error (rpm) | 0.16 | 1.67 | 2.26 |
| 1400 rpm current ripple RMS (A) | — | 0.254 | 0.234 |

For the ripple metric, the comparison removes a fitted line from each phase
and each PWM period between 1.05 and 1.25 s. The full figures and method are
in [the report](../REPORT.pdf).

## Model conventions

- The basic-block bridge current is the dot product of the average duty
  vector with the phase currents. The actual DC-link voltage scales the
  bridge output. The illustrative R-L-C values are in `motor_params.m`.
- The Simscape bridge receives upper/lower gate pairs in A, B, C order.
  It shares the same 10 kHz timing in both modes. Keep the power step an
  integer divisor of the PWM and control periods.
- The Simscape source uses the positive terminal on the right connection
  port and grounds the negative rail. The IGBT bridge is set to inverter
  mode. The machine uses SI parameters, a measurement bus and initial slip
  of 1 for standstill.
- The SPS Park transforms use the sine convention, so the controller maps
  the axes as `q_sps = d` and `d_sps = -q`.
