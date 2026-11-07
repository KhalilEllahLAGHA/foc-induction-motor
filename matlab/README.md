# MATLAB and Simulink FOC drive

The R2024b models run the same 2.4 s speed-step and 10 N m load test
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
to seven-segment SVPWM. Close the three graphical models before calling
`compare_foc_pwm`. It runs the saved variants and exports torque, phase-voltage
and gate-state data and plots to `outputs/graphs/`.

## Open the graphical PWM models

Run `foc_init` with `m_files/` on the MATLAB path, then open one of these
Simscape drive models and double-click **PWM Generator**.

- [SPWM drive](simulink_library/foc_im_spwm.slx) shows the shared triangle
  carrier and the three phase comparators.
- [SVPWM drive](simulink_library/foc_im_svpwm.slx) shows the Clarke transform,
  sector and dwell times `T1`, `T2` and `T0` in seconds, and the centered
  seven-segment gate sequence.
- [Four-segment SVPWM drive](simulink_library/foc_im_svpwm_4seg.slx) uses the
  same dwell-time calculation and steps forward through `000`, the two
  active vectors, and `111`. Each zero state uses half of `T0`. The sequence
  starts again at `000` in the next PWM period.

Run `build_foc_pwm_variants` in `simulink_library/` to regenerate all three
files from the saved Simscape drive, or pass a model name to rebuild one.

## Read the signals in Simulink

Run `foc_init` with `m_files/` on the MATLAB path, open either saved model,
and double-click its **Scopes** subsystem. Open the view you need and run
the model. The scopes show rotor speed and its reference in rpm, motor and
load torque, phase currents, the d and q currents against their references,
and the estimated rotor flux against its target. The fundamental-block model
also shows the DC-link voltage. The Simscape model shows the three upper IGBT
gate commands over a 0.6 ms window, which is six PWM periods at 10 kHz.
See the [fundamental-block](outputs/diagrams/blocks_scopes.png) and
[Simscape](outputs/diagrams/library_scopes.png) scope layouts.

## Export the diagrams

Run `export_diagrams` and `export_pwm_explanations` with `m_files/` on the
MATLAB path to regenerate model snapshots and the PWM teaching figures. The
saved models include hand-edited layouts. The builders overwrite them, so
run the saved models for normal simulation.

## Simscape results

The [report](../REPORT.pdf) shows the SVPWM speed, torque and d/q response
along with one complete electrical cycle of the three raw phase currents.
The three-mode [torque comparison](outputs/graphs/library_pwm_comparison.png)
and the pairwise phase-voltage plots use the 1.1050–1.1056 s window at the
1400 rpm command. The [recorded waveforms](outputs/graphs/library_pwm_waveforms.csv)
include the switching state as an integer from 0 (`000`) to 7 (`111`).

## Model conventions

- The basic-block bridge current is the dot product of the average duty
  vector with the phase currents. The actual DC-link voltage scales the
  bridge output. The illustrative R-L-C values are in `motor_params.m`.
- The Simscape bridge receives upper/lower gate pairs in A, B, C order.
  The graphical variants share the same 10 kHz timing. Keep the power step an
  integer divisor of the PWM and control periods.
- The Simscape source uses the positive terminal on the right connection
  port and grounds the negative rail. The IGBT bridge is set to inverter
  mode. The machine uses SI parameters, a measurement bus and initial slip
  of 1 for standstill.
- The SPS Park transforms use the sine convention, so the controller maps
  the axes as `q_sps = d` and `d_sps = -q`.
