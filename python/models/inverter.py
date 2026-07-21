"""Two-level voltage-source inverter with space-vector PWM (SVPWM).

The modulator uses min-max (third-harmonic-equivalent) common-mode injection,
which reproduces the duty cycles of classic space-vector PWM and extends the
linear range to a phase-voltage amplitude of Vdc/sqrt(3).

Two output models are available:

``average``  (default)
    Switching-cycle-averaged model: the phase voltage applied to the machine
    equals its duty cycle times Vdc with the zero-sequence removed.  Fast and
    accurate for control studies (the PWM ripple is not represented).

``switching``
    True two-level switched waveforms: each phase duty cycle is compared
    against a common triangular carrier at ``f_sw`` and the machine sees the
    resulting +/-Vdc pole-voltage steps.  Requires a much smaller simulation
    step (~1e-6 s) to resolve the carrier.
"""

from __future__ import annotations

import math

from control.pid import clarke, inv_clarke

_INV_SQRT3 = 1.0 / math.sqrt(3.0)


class TwoLevelInverter:
    """Ideal two-level VSI + SVPWM modulator feeding a star-connected machine."""

    def __init__(self, v_dc: float, f_sw: float, mode: str = "average") -> None:
        """
        Parameters
        ----------
        v_dc : float
            DC-link voltage [V].
        f_sw : float
            Switching (carrier) frequency [Hz] — used by the switching model
            and to define the modulator's linear voltage limit.
        mode : str
            ``"average"`` or ``"switching"`` (see module docstring).
        """
        if mode not in ("average", "switching"):
            raise ValueError(f"unknown inverter mode: {mode!r}")
        self.v_dc = v_dc
        self.f_sw = f_sw
        self.mode = mode
        # Last computed duty cycles (for logging/inspection).
        self.duties = (0.5, 0.5, 0.5)

    # ------------------------------------------------------------------ API
    @property
    def v_max(self) -> float:
        """Maximum phase-voltage amplitude in the linear SVPWM range [V]."""
        return self.v_dc * _INV_SQRT3

    def limit_reference(self, v_alpha: float, v_beta: float) -> tuple[float, float]:
        """Clamp the reference vector magnitude to the linear SVPWM limit."""
        mag = math.hypot(v_alpha, v_beta)
        if mag > self.v_max > 0.0:
            scale = self.v_max / mag
            return v_alpha * scale, v_beta * scale
        return v_alpha, v_beta

    def apply(self, v_alpha_ref: float, v_beta_ref: float,
              t: float) -> tuple[float, float]:
        """Modulate the reference vector and return the applied stator voltage.

        Parameters
        ----------
        v_alpha_ref, v_beta_ref : float
            Reference stator voltage in the stationary frame [V].
        t : float
            Current simulation time [s] (used by the carrier in switching mode).

        Returns
        -------
        (v_alpha, v_beta) : tuple of float
            Voltage actually applied to the machine terminals (zero-sequence
            removed, i.e. line-to-neutral of the star-connected stator) [V].
        """
        v_alpha_ref, v_beta_ref = self.limit_reference(v_alpha_ref, v_beta_ref)
        va, vb, vc = inv_clarke(v_alpha_ref, v_beta_ref)

        # --- SVPWM min-max common-mode injection --------------------------
        v_offset = -0.5 * (max(va, vb, vc) + min(va, vb, vc))
        duties = tuple(
            min(max(0.5 + (v + v_offset) / self.v_dc, 0.0), 1.0)
            for v in (va, vb, vc)
        )
        self.duties = duties

        if self.mode == "average":
            # Cycle-averaged pole voltages.
            pole = [d * self.v_dc for d in duties]
        else:
            # Symmetric triangular carrier in [0, 1] at f_sw.
            frac = (t * self.f_sw) % 1.0
            carrier = 1.0 - abs(2.0 * frac - 1.0)
            pole = [self.v_dc if d > carrier else 0.0 for d in duties]

        # Remove the zero-sequence (isolated neutral): v_xn = v_xN - mean(v_N).
        v_mean = sum(pole) / 3.0
        van, vbn, vcn = (v - v_mean for v in pole)
        return clarke(van, vbn, vcn)
