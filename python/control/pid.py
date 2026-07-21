"""Reusable PID controller and the Clarke/Park reference-frame transforms.

The PID class is shared by the two current loops (i_d, i_q) and the speed loop
of the FOC scheme.  All transforms use the *amplitude-invariant* convention
(factor 2/3), i.e. the dq/alpha-beta magnitudes equal the peak phase quantity.
With this convention the electromagnetic torque carries a 3/2 factor.
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# PID controller
# ---------------------------------------------------------------------------


class PIDController:
    """Discrete PI(D) controller with output saturation and anti-windup.

    Anti-windup uses unity-gain back-calculation: after the output is clamped,
    the integrator is corrected by the amount of the clamp so it never charges
    beyond what the actuator can deliver.

    Parameters
    ----------
    kp, ki, kd : float
        Proportional, integral and derivative gains (continuous-time values;
        integration/differentiation is done internally with the sample time).
    dt : float
        Controller sample time [s].
    out_min, out_max : float
        Output saturation limits (also the anti-windup limits).
    """

    def __init__(self, kp: float, ki: float, dt: float, kd: float = 0.0,
                 out_min: float = -math.inf, out_max: float = math.inf) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self.reset()

    def reset(self) -> None:
        """Zero the internal state (integrator and derivative memory)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def set_limits(self, out_min: float, out_max: float) -> None:
        """Update the saturation limits at run time (e.g. dynamic i_q limit)."""
        self.out_min = out_min
        self.out_max = out_max

    def update(self, error: float, feedforward: float = 0.0) -> float:
        """Advance the controller one sample.

        Parameters
        ----------
        error : float
            Reference minus measurement.
        feedforward : float
            Term added after the PID sum but *inside* the saturation, so the
            anti-windup accounts for it (used for the dq decoupling voltages).

        Returns
        -------
        float
            Saturated controller output.
        """
        # Derivative on error (skipped on the very first call).
        if self._first_call or self.kd == 0.0:
            derivative = 0.0
            self._first_call = False
        else:
            derivative = self.kd * (error - self._prev_error) / self.dt
        self._prev_error = error

        # Integrate, then clamp and back-calculate.
        self._integral += self.ki * error * self.dt
        unsaturated = self.kp * error + self._integral + derivative + feedforward
        output = min(max(unsaturated, self.out_min), self.out_max)
        self._integral += output - unsaturated  # anti-windup correction
        return output


# ---------------------------------------------------------------------------
# Clarke / Park transforms (amplitude-invariant)
# ---------------------------------------------------------------------------

_TWO_THIRDS = 2.0 / 3.0
_INV_SQRT3 = 1.0 / math.sqrt(3.0)
_HALF_SQRT3 = math.sqrt(3.0) / 2.0


def clarke(a: float, b: float, c: float) -> tuple[float, float]:
    """abc -> alpha-beta (stationary frame), amplitude invariant."""
    alpha = _TWO_THIRDS * (a - 0.5 * b - 0.5 * c)
    beta = _INV_SQRT3 * (b - c)
    return alpha, beta


def inv_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    """alpha-beta -> abc (zero-sequence free)."""
    a = alpha
    b = -0.5 * alpha + _HALF_SQRT3 * beta
    c = -0.5 * alpha - _HALF_SQRT3 * beta
    return a, b, c


def park(alpha: float, beta: float, theta: float) -> tuple[float, float]:
    """alpha-beta -> dq, rotating frame at angle ``theta`` [rad]."""
    ct, st = math.cos(theta), math.sin(theta)
    d = alpha * ct + beta * st
    q = -alpha * st + beta * ct
    return d, q


def inv_park(d: float, q: float, theta: float) -> tuple[float, float]:
    """dq -> alpha-beta."""
    ct, st = math.cos(theta), math.sin(theta)
    alpha = d * ct - q * st
    beta = d * st + q * ct
    return alpha, beta


def abc_to_dq(a: float, b: float, c: float, theta: float) -> tuple[float, float]:
    """Convenience: Clarke followed by Park."""
    return park(*clarke(a, b, c), theta)


def dq_to_abc(d: float, q: float, theta: float) -> tuple[float, float, float]:
    """Convenience: inverse Park followed by inverse Clarke."""
    return inv_clarke(*inv_park(d, q, theta))
