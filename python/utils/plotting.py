"""Result plotting: renders and saves every simulation figure as a PNG.

Series colours are assigned in a fixed order (blue, orange, aqua);
references and limits are drawn as dashed secondary-ink lines.
"""

from __future__ import annotations

import math
import os

import matplotlib

matplotlib.use("Agg")  # headless: render straight to files

import matplotlib.pyplot as plt

# --- palette ----------------------------------------------------------------
BLUE = "#2a78d6"      # series slot 1
ORANGE = "#eb6834"    # series slot 2
AQUA = "#1baf7a"      # series slot 3
SURFACE = "#fcfcfb"
INK = "#0b0b0b"       # primary text
INK2 = "#52514e"      # secondary ink: reference / limit lines
MUTED = "#898781"     # axis tick labels
GRID = "#e1e0d9"      # hairline gridlines
AXIS = "#c3c2b7"      # axis spines

_RPM = 60.0 / (2.0 * math.pi)   # rad/s (mech) -> rpm


def _apply_style() -> None:
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "text.color": INK,
        "axes.labelcolor": INK2,
        "axes.titlecolor": INK,
        "axes.titleweight": "bold",
        "axes.titlesize": 12,
        "axes.labelsize": 10.5,
        "axes.edgecolor": AXIS,
        "axes.linewidth": 1.0,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelsize": 9.5,
        "ytick.labelsize": 9.5,
        "legend.frameon": False,
        "legend.fontsize": 9.5,
        "lines.linewidth": 1.8,
        "font.family": "sans-serif",
    })


def _new_axes(title: str, ylabel: str, xlabel: str = "Time [s]"):
    fig, ax = plt.subplots(figsize=(9.0, 4.6), layout="constrained")
    ax.set_title(title, loc="left", pad=10)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    return fig, ax


def _save(fig, out_dir: str, name: str) -> str:
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# individual figures
# ---------------------------------------------------------------------------

def plot_speed(r: dict, out_dir: str, fname: str = "speed.png") -> str:
    fig, ax = _new_axes("Rotor speed — reference vs. measured", "Speed [rpm]")
    ax.plot(r["t"], r["omega_ref"] * _RPM, "--", color=INK2, lw=1.4,
            label="Reference")
    ax.plot(r["t"], r["omega_m"] * _RPM, color=BLUE, label="Measured")
    ax.legend(loc="upper right")
    return _save(fig, out_dir, fname)


def plot_torque(r: dict, out_dir: str) -> str:
    fig, ax = _new_axes("Electromagnetic torque and load torque", "Torque [N·m]")
    ax.plot(r["t"], r["t_load"], "--", color=INK2, lw=1.4, label="Load torque")
    ax.plot(r["t"], r["te"], color=BLUE, label="Electromagnetic torque")
    ax.legend(loc="upper right")
    return _save(fig, out_dir, "torque.png")


def plot_currents_dq(r: dict, out_dir: str,
                     fname: str = "currents_dq.png") -> str:
    fig, ax = _new_axes("Stator currents in the rotor-flux frame",
                        "Current [A]")
    ax.plot(r["t"], r["id_ref"], "--", color=BLUE, lw=1.3, alpha=0.85,
            label="i$_d$ reference")
    ax.plot(r["t"], r["i_d"], color=BLUE, label="i$_d$ measured")
    ax.plot(r["t"], r["iq_ref"], "--", color=ORANGE, lw=1.3, alpha=0.85,
            label="i$_q$ reference")
    ax.plot(r["t"], r["i_q"], color=ORANGE, label="i$_q$ measured")
    ax.legend(loc="lower left", ncols=2)
    return _save(fig, out_dir, fname)


def plot_currents_abc(r: dict, out_dir: str) -> str:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.0, 6.4),
                                   layout="constrained")
    for ax, title in ((ax1, "Stator phase currents — full run"),
                      (ax2, "Stator phase currents — zoom (last 100 ms)")):
        ax.set_title(title, loc="left", pad=8)
        ax.set_ylabel("Current [A]")
    ax2.set_xlabel("Time [s]")

    t = r["t"]
    for idx, (name, color) in enumerate((("i$_a$", BLUE), ("i$_b$", ORANGE),
                                         ("i$_c$", AQUA))):
        ax1.plot(t, r["i_abc"][:, idx], color=color, lw=1.2, label=name)
        ax2.plot(t, r["i_abc"][:, idx], color=color, lw=1.6, label=name)
    ax2.set_xlim(t[-1] - 0.10, t[-1])
    ax1.legend(loc="upper left", ncols=3)
    return _save(fig, out_dir, "currents_abc.png")


def plot_flux(r: dict, out_dir: str) -> str:
    fig, ax = _new_axes("Rotor flux linkage (field orientation quality)",
                        "Flux linkage [Wb]")
    ax.axhline(r["psi_ref"], ls="--", color=INK2, lw=1.4, label="Flux reference")
    ax.plot(r["t"], r["psi_r_mag"], color=BLUE, label="|ψ$_r$| (model)")
    ax.plot(r["t"], r["psi_est"], color=ORANGE, lw=1.4,
            label="ψ$_r$ estimate (controller)")
    ax.plot(r["t"], r["psi_qr_e"], color=AQUA, lw=1.4,
            label="ψ$_{qr}$ in flux frame (→ 0 if oriented)")
    ax.legend(loc="center right")
    return _save(fig, out_dir, "flux.png")


def plot_voltages_dq(r: dict, out_dir: str) -> str:
    fig, ax = _new_axes("Current-controller output voltages", "Voltage [V]")
    v_max = r["v_dc"] / math.sqrt(3.0)
    ax.axhline(v_max, ls="--", color=INK2, lw=1.2, label="SVPWM linear limit")
    ax.axhline(-v_max, ls="--", color=INK2, lw=1.2)
    ax.plot(r["t"], r["v_d"], color=BLUE, label="v$_d$")
    ax.plot(r["t"], r["v_q"], color=ORANGE, label="v$_q$")
    ax.legend(loc="upper right")
    return _save(fig, out_dir, "voltages_dq.png")


def plot_tracking_error(r: dict, out_dir: str,
                        fname: str = "tracking_error.png") -> str:
    """Speed tracking error vs. time (control-oriented study figure)."""
    fig, ax = _new_axes("Speed tracking error (reference − measured)",
                        "Error [rpm]")
    err = (r["omega_ref"] - r["omega_m"]) * _RPM
    ax.axhline(0.0, color=AXIS, lw=1.0)
    ax.plot(r["t"], err, color=BLUE, label="Tracking error")
    ax.legend(loc="upper left")
    return _save(fig, out_dir, fname)


# ---------------------------------------------------------------------------
# entry points
# ---------------------------------------------------------------------------

def save_all_plots(results: dict, out_dir: str) -> list[str]:
    """Render every result figure into ``out_dir`` and return the file paths.

    ``results`` is the dict assembled by ``main.py`` (time vector, logged
    signals and a few scalars such as ``psi_ref`` and ``v_dc``).
    """
    os.makedirs(out_dir, exist_ok=True)
    _apply_style()
    return [
        plot_speed(results, out_dir),
        plot_torque(results, out_dir),
        plot_currents_dq(results, out_dir),
        plot_currents_abc(results, out_dir),
        plot_flux(results, out_dir),
        plot_voltages_dq(results, out_dir),
    ]


def save_study_plots(results: dict, out_dir: str) -> list[str]:
    """Figures for the control-oriented tracking study (scenario B):
    speed, tracking error and dq currents, saved with a ``study_`` prefix."""
    os.makedirs(out_dir, exist_ok=True)
    _apply_style()
    return [
        plot_speed(results, out_dir, "study_speed.png"),
        plot_tracking_error(results, out_dir, "study_tracking_error.png"),
        plot_currents_dq(results, out_dir, "study_currents_dq.png"),
    ]
