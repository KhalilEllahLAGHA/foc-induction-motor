"""Field-Oriented Control (FOC) of an induction motor — simulation entry point.

ALL user inputs live in this file: motor parameters, inverter settings,
controller bandwidths/gains, references, the test scenario and the solver
setup.  Run it from the ``python/`` folder:

    py main.py

Every result figure is saved to ``outputs/graphs/``.

Two scenarios are simulated in sequence:

Scenario A (main): multi-level constant-speed profile, step changes only
------------------------------------------------------------------------
1. 0.00-0.30 s : flux build-up at standstill (i_d magnetizes the machine)
2. 0.30 s      : STEP 0 -> 800 rpm
3. 0.80 s      : STEP 800 -> 1400 rpm
4. 1.30 s      : STEP 1400 -> 600 rpm (deceleration)
5. 1.80 s      : STEP 600 -> -800 rpm (reversal, four-quadrant operation)
A 10 N*m load-torque step is applied at t = 0.55 s and kept for the rest of
the run.  Edit SPEED_SEGMENTS / LOAD_STEPS below to change the scenario.

Scenario B (unloaded speed tracking): ramp and hold
--------------------------------------------------
The drive builds flux at standstill, ramps to 1000 rpm, and holds that speed.
Figures are saved as study_*.png. See REPORT.md section 6.3 for the analysis.
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from models.induction_motor import InductionMotor, InductionMotorParams
from models.inverter import TwoLevelInverter
from control.foc import FOCGains, FOCLimits, IndirectFOC
from control.pid import park
from utils.plotting import save_all_plots, save_study_plots

# ===========================================================================
# 1. USER INPUTS
# ===========================================================================

# --- 1.1 Motor: 5 kW, 380 V, 10.5 A, 50 Hz squirrel-cage machine -----------
# (course data: Rs=0.435, Rr=0.816, Ls=Lr=0.071 H TOTAL, Lm=0.069 H, p=2,
#  J=0.0056 kg*m^2, f_r=0.0015 N*m*s -> leakages Lls=Llr=Ls-Lm=0.002 H)
MOTOR = InductionMotorParams(
    Rs=0.435,        # stator resistance                [ohm]
    Rr=0.816,        # rotor resistance (referred)      [ohm]
    Lls=0.002,       # stator leakage inductance        [H]  (Ls - Lm)
    Llr=0.002,       # rotor leakage inductance         [H]  (Lr - Lm)
    Lm=0.069,        # magnetizing inductance           [H]
    pole_pairs=2,    # pole pairs
    J=0.0056,        # rotor inertia                    [kg*m^2]
    B=0.0015,        # viscous friction                 [N*m*s]
)

# --- 1.2 Inverter -----------------------------------------------------------
V_DC = 540.0         # DC-link voltage [V]  (~ sqrt(2)*380: diode rectifier)
F_SW = 10e3          # switching / carrier frequency [Hz]
PWM_MODE = "average"  # "average" (fast) or "switching" (true PWM waveforms)

# --- 1.3 Control loop setup -------------------------------------------------
TS_CONTROL = 1.0e-4          # controller sample time [s] (10 kHz, = PWM rate)
BW_CURRENT = 2.0 * math.pi * 400.0   # current-loop bandwidth [rad/s]
BW_SPEED = 2.0 * math.pi * 30.0      # speed-loop bandwidth   [rad/s]

# PI gains by pole-zero cancellation / bandwidth placement.
# Current loops see the plant 1/(sigma*Ls*s + R_sigma) after decoupling.
KP_CURRENT = MOTOR.sigma * MOTOR.Ls * BW_CURRENT     # [V/A]
KI_CURRENT = MOTOR.R_sigma * BW_CURRENT              # [V/(A*s)]
# Speed loop commands torque against the inertia J.
KP_SPEED = MOTOR.J * BW_SPEED                        # [N*m*s/rad]
KI_SPEED = KP_SPEED * BW_SPEED / 5.0                 # [N*m/rad]

# --- 1.4 References and limits ---------------------------------------------
PSI_REF = 0.8        # rotor flux linkage reference [Wb] (-> id* = 11.6 A)
I_MAX = 25.0         # stator current amplitude limit [A]
TORQUE_MAX = 35.0    # speed-PI torque limit [N*m] (rated ~33 N*m)

# Speed reference profile: list of segments (t_start, t_end, kind, params).
#   "hold" : constant at params["rpm"] (or at the previous value if omitted)
#   "step" : jump to params["rpm"] at t_start and stay there
#   "ramp" : linear from the previous value to params["rpm"] over the segment
#   "sine" : params["amp_rpm"] / params["freq_hz"] oscillation around the
#            value reached at t_start
SPEED_SEGMENTS = [
    (0.00, 0.30, "hold", {"rpm": 0.0}),       # flux build-up at standstill
    (0.30, 0.80, "step", {"rpm": 800.0}),     # step up
    (0.80, 1.30, "step", {"rpm": 1400.0}),    # step up
    (1.30, 1.80, "step", {"rpm": 600.0}),     # step down
    (1.80, 2.40, "step", {"rpm": -800.0}),    # reversal step (four quadrants)
]

# Load torque schedule: list of (time, torque) - each entry applies from its
# time onward.
LOAD_STEPS = [
    (0.00, 0.0),
    (0.55, 10.0),    # 10 N*m load applied and kept until the end
]

# --- 1.5 Simulation ---------------------------------------------------------
T_END = 2.4                  # total simulated time [s]
# Integration substeps per control period: the average-value inverter needs
# only a few; the switching model must resolve the 10 kHz carrier.
N_SUBSTEPS = 5 if PWM_MODE == "average" else 50

OUT_DIR = Path(__file__).resolve().parent / "outputs" / "graphs"

# --- 1.6 Scenario B: unloaded tracking study (ramp and hold) ---------------
# Pure servo study - no load torque, references kept inside the linear
# (unsaturated) region so the measured errors reflect the PI loop itself.
STUDY_SPEED_SEGMENTS = [
    (0.00, 0.30, "hold", {"rpm": 0.0}),       # flux build-up at standstill
    (0.30, 0.90, "ramp", {"rpm": 1000.0}),    # linear ramp (type-2 tracking)
    (0.90, 1.20, "hold", {}),                 # settle at 1000 rpm
]
STUDY_LOAD_STEPS = [(0.00, 0.0)]              # unloaded
STUDY_T_END = STUDY_SPEED_SEGMENTS[-1][1]

# ===========================================================================
# 2. SCENARIO PROFILES
# ===========================================================================

_RPM2RAD = 2.0 * math.pi / 60.0   # rpm -> rad/s (mechanical)


def _prepare_speed_profile(segments: list) -> list:
    """Resolve each segment's entry and target values [rad/s] ahead of time.

    Returns a list of (t_start, t_end, kind, entry, target, params) with the
    profile value guaranteed continuous across "ramp"/"hold"/"sine" segment
    boundaries ("step" is deliberately discontinuous).
    """
    prepared = []
    value = 0.0
    for t0, t1, kind, prm in segments:
        entry = value
        if kind == "sine":
            target = entry                      # oscillates around entry value
        elif "rpm" in prm:
            target = prm["rpm"] * _RPM2RAD
        else:
            target = entry                      # hold previous level
        prepared.append((t0, t1, kind, entry, target, prm))
        value = target
    return prepared


def speed_reference(t: float, profile: list) -> float:
    """Multi-segment speed reference [rad/s mech] (prepared profile)."""
    seg = None
    for s in profile:
        if t >= s[0]:
            seg = s
        else:
            break
    if seg is None:
        return 0.0
    t0, t1, kind, entry, target, prm = seg
    if kind == "ramp":
        frac = min((t - t0) / (t1 - t0), 1.0)
        return entry + (target - entry) * frac
    if kind == "sine" and t < t1:
        return entry + prm["amp_rpm"] * _RPM2RAD * math.sin(
            2.0 * math.pi * prm["freq_hz"] * (t - t0))
    return target


def load_torque(t: float, load_steps: list) -> float:
    """Piecewise-constant load torque [N*m] from a (time, torque) schedule."""
    tl = 0.0
    for t_k, tau_k in load_steps:
        if t >= t_k:
            tl = tau_k
    return tl


# ===========================================================================
# 3. SIMULATION
# ===========================================================================

def run_simulation(segments: list = SPEED_SEGMENTS,
                   load_steps: list = LOAD_STEPS,
                   t_end: float = T_END) -> dict:
    """Run the closed-loop FOC simulation and return all logged signals."""
    profile = _prepare_speed_profile(segments)
    motor = InductionMotor(MOTOR)
    inverter = TwoLevelInverter(v_dc=V_DC, f_sw=F_SW, mode=PWM_MODE)
    foc = IndirectFOC(
        MOTOR,
        FOCGains(KP_CURRENT, KI_CURRENT, KP_SPEED, KI_SPEED),
        FOCLimits(v_dc=V_DC, i_max=I_MAX, torque_max=TORQUE_MAX),
        psi_ref=PSI_REF,
        ts=TS_CONTROL,
    )

    n_steps = int(round(t_end / TS_CONTROL))
    dt = TS_CONTROL / N_SUBSTEPS

    # --- preallocate logs (one sample per control period) -------------------
    log = {key: np.zeros(n_steps) for key in (
        "t", "omega_ref", "omega_m", "te", "t_load", "i_d", "i_q",
        "id_ref", "iq_ref", "psi_r_mag", "psi_est", "psi_qr_e",
        "v_d", "v_q", "omega_slip")}
    log["i_abc"] = np.zeros((n_steps, 3))

    t_wall = time.perf_counter()
    for k in range(n_steps):
        t = k * TS_CONTROL
        w_ref = speed_reference(t, profile)
        t_l = load_torque(t, load_steps)

        # -- measure and run the controller (10 kHz) -------------------------
        i_abc = motor.i_abc
        v_a_ref, v_b_ref, ctl = foc.step(motor.omega_m, i_abc, w_ref)

        # -- log at the measurement instant ----------------------------------
        psi_ra, psi_rb = motor.psi_r_alpha_beta
        _, psi_qr_e = park(psi_ra, psi_rb, ctl["theta_e"])
        log["t"][k] = t
        log["omega_ref"][k] = w_ref
        log["omega_m"][k] = motor.omega_m
        log["te"][k] = motor.torque
        log["t_load"][k] = t_l
        log["i_abc"][k] = i_abc
        log["i_d"][k], log["i_q"][k] = ctl["i_d"], ctl["i_q"]
        log["id_ref"][k], log["iq_ref"][k] = ctl["id_ref"], ctl["iq_ref"]
        log["psi_r_mag"][k] = motor.psi_r_mag
        log["psi_est"][k] = ctl["psi_est"]
        log["psi_qr_e"][k] = psi_qr_e
        log["v_d"][k], log["v_q"][k] = ctl["v_d"], ctl["v_q"]
        log["omega_slip"][k] = ctl["omega_slip"]

        # -- plant integration over one control period -----------------------
        for s in range(N_SUBSTEPS):
            v_sa, v_sb = inverter.apply(v_a_ref, v_b_ref, t + s * dt)
            motor.step(v_sa, v_sb, t_l, dt)

    log["wall_time"] = time.perf_counter() - t_wall
    log["psi_ref"] = PSI_REF
    log["v_dc"] = V_DC
    return log


# ===========================================================================
# 4. POST-PROCESSING
# ===========================================================================

def print_summary(r: dict) -> None:
    """Print closed-loop performance metrics for the multi-segment scenario."""
    t = r["t"]
    rpm = 60.0 / (2.0 * math.pi)
    err = (r["omega_ref"] - r["omega_m"]) * rpm

    # Steady tracking quality, ignoring the flux build-up phase and the
    # unavoidable transients right after reference steps and load changes.
    settled = t >= SPEED_SEGMENTS[1][0]
    step_transient = np.zeros_like(t, dtype=bool)
    for ts in (s[0] for s in SPEED_SEGMENTS if s[2] == "step"):
        step_transient |= (t >= ts) & (t < ts + 0.15)
    load_transient = np.zeros_like(t, dtype=bool)
    for t_k, _ in LOAD_STEPS[1:]:
        load_transient |= (t >= t_k) & (t < t_k + 0.15)
    track = settled & ~step_transient & ~load_transient

    # Load-disturbance dip around each load change, with reference steps
    # masked out.
    dips = []
    for t_k, _ in LOAD_STEPS[1:]:
        win = (t >= t_k) & (t <= t_k + 0.25) & ~step_transient
        dips.append(np.abs(err[win]).max())

    print(f"  simulated {t[-1] + (t[1] - t[0]):.2f} s "
          f"in {r['wall_time']:.1f} s of wall time")
    print(f"  RMS tracking error (settled) : "
          f"{np.sqrt(np.mean(err[track] ** 2)):7.2f} rpm")
    print(f"  max tracking error (settled) : {np.abs(err[track]).max():7.2f} rpm")
    for t_k, dip in zip([x[0] for x in LOAD_STEPS[1:]], dips):
        print(f"  speed dip at load step {t_k:.2f} s : {dip:7.2f} rpm")
    print(f"  speed error at end of run    : {err[-1]:7.2f} rpm")
    print(f"  rotor flux |psi_r| at end    : {r['psi_r_mag'][-1]:7.3f} Wb "
          f"(ref {r['psi_ref']:.2f})")
    print(f"  psi_qr in flux frame at end  : {abs(r['psi_qr_e'][-1]):7.4f} Wb "
          f"(0 = oriented)")
    print(f"  peak phase current           : {np.abs(r['i_abc']).max():7.2f} A "
          f"(limit {I_MAX:.0f})")


def print_study_summary(r: dict) -> None:
    """Report settled speed errors during the ramp and hold."""
    t = r["t"]
    rpm = 60.0 / (2.0 * math.pi)
    err = (r["omega_ref"] - r["omega_m"]) * rpm

    windows = [
        ("ramp (steady part)  ", 0.50, 0.88),
        ("hold 1000 rpm       ", 1.00, 1.20),
    ]
    for name, a, b in windows:
        m = (t >= a) & (t <= b)
        print(f"  {name}: max {np.abs(err[m]).max():6.2f} rpm, "
              f"rms {np.sqrt(np.mean(err[m] ** 2)):6.2f} rpm")


def main() -> None:
    print(f"FOC induction motor simulation ({PWM_MODE} inverter model)")
    print("--- scenario A: multi-level speed steps + load ---")
    results = run_simulation(SPEED_SEGMENTS, LOAD_STEPS, T_END)
    print_summary(results)
    paths = save_all_plots(results, str(OUT_DIR))

    print("--- scenario B: tracking study (ramp and hold, no load) ---")
    study = run_simulation(STUDY_SPEED_SEGMENTS, STUDY_LOAD_STEPS, STUDY_T_END)
    print_study_summary(study)
    paths += save_study_plots(study, str(OUT_DIR))

    print("saved figures:")
    for p in paths:
        print(f"  {p}")


if __name__ == "__main__":
    main()
