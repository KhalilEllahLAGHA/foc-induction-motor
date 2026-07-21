"""Indirect rotor-flux-oriented control (IFOC) of an induction motor.

Structure (classic cascaded scheme):

                        +--------------------------------------------+
  omega_ref ->(PI)-> Te* -> iq*  --+                                 |
                                   +--> (PI iq) --+ decoupling       |
  psi_ref  -> id* = psi_ref/Lm  ---+--> (PI id) --+--> vd*, vq*      |
                                   |                     |           |
        i_abc --Clarke/Park--> id, iq          inverse Park --> v_alpha*, v_beta*
                        ^                                ^
                        |                                |
   theta_e = integral( p*omega_m + omega_slip ),  omega_slip = Lm*iq/(tau_r*psi_r)

The rotor flux magnitude is not measured; it is estimated from i_d through the
rotor circuit's first-order dynamics  tau_r * d(psi_r)/dt + psi_r = Lm * i_d.
The estimate feeds both the slip calculation and the torque-to-iq conversion,
which keeps the torque scaling correct while the flux is still building up.

Decoupling feed-forward added to the current-loop PI outputs:

    vd_ff = -omega_e * sigma * Ls * iq
    vq_ff = +omega_e * (sigma * Ls * id + (Lm/Lr) * psi_r)

which cancels the cross-coupling between the d and q axes so each PI sees an
(approximately) first-order R-L plant.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from control.pid import PIDController, abc_to_dq, inv_park

if TYPE_CHECKING:  # importing at run time would be circular
    from models.induction_motor import InductionMotorParams

_PSI_MIN = 0.05  # [Wb] guard against division by zero before flux builds up


@dataclass
class FOCGains:
    """PI gains of the three control loops."""

    kp_current: float   # d and q current loops share the same plant -> same gains
    ki_current: float
    kp_speed: float
    ki_speed: float


@dataclass
class FOCLimits:
    """Actuator and safety limits used by the controller."""

    v_dc: float        # DC-link voltage [V] -> voltage limit Vdc/sqrt(3)
    i_max: float       # maximum stator current amplitude [A]
    torque_max: float  # speed-loop output (torque reference) limit [N*m]


class IndirectFOC:
    """Discrete-time indirect field-oriented controller."""

    def __init__(self, motor: "InductionMotorParams", gains: FOCGains,
                 limits: FOCLimits, psi_ref: float, ts: float) -> None:
        """
        Parameters
        ----------
        motor : InductionMotorParams
            Machine parameters (the controller uses Lm, Lr, tau_r, sigma, Ls,
            pole pairs — i.e. a *model-based* controller).
        gains : FOCGains
            PI gains for the current and speed loops.
        limits : FOCLimits
            Voltage/current/torque limits.
        psi_ref : float
            Rotor flux linkage reference [Wb].
        ts : float
            Controller sample time [s].
        """
        self.m = motor
        self.limits = limits
        self.psi_ref = psi_ref
        self.ts = ts

        self.v_max = limits.v_dc / math.sqrt(3.0)

        self.speed_pi = PIDController(gains.kp_speed, gains.ki_speed, ts,
                                      out_min=-limits.torque_max,
                                      out_max=limits.torque_max)
        self.id_pi = PIDController(gains.kp_current, gains.ki_current, ts,
                                   out_min=-self.v_max, out_max=self.v_max)
        self.iq_pi = PIDController(gains.kp_current, gains.ki_current, ts,
                                   out_min=-self.v_max, out_max=self.v_max)

        self.theta_e = 0.0    # rotor flux angle estimate [rad]
        self.psi_est = 0.0    # rotor flux magnitude estimate [Wb]

    def reset(self) -> None:
        """Return the controller to its power-up state."""
        self.speed_pi.reset()
        self.id_pi.reset()
        self.iq_pi.reset()
        self.theta_e = 0.0
        self.psi_est = 0.0

    # ------------------------------------------------------------------ API
    def step(self, omega_m: float, i_abc: tuple[float, float, float],
             omega_ref: float) -> tuple[float, float, dict]:
        """Run one control period.

        Parameters
        ----------
        omega_m : float
            Measured mechanical speed [rad/s].
        i_abc : (ia, ib, ic)
            Measured stator phase currents [A].
        omega_ref : float
            Mechanical speed reference [rad/s].

        Returns
        -------
        (v_alpha_ref, v_beta_ref, log)
            Stator voltage reference in the stationary frame [V] and a dict of
            internal signals for logging.
        """
        m = self.m

        # --- measurements into the (estimated) rotor flux frame -----------
        i_d, i_q = abc_to_dq(*i_abc, self.theta_e)

        # --- rotor flux estimator (forward-Euler on the rotor lag) ---------
        self.psi_est += self.ts * (m.Lm * i_d - self.psi_est) / m.tau_r
        psi = max(self.psi_est, _PSI_MIN)

        # --- outer loops: flux (feed-forward) and speed (PI) ---------------
        id_ref = self.psi_ref / m.Lm
        te_ref = self.speed_pi.update(omega_ref - omega_m)

        k_t = 1.5 * m.pole_pairs * (m.Lm / m.Lr) * psi   # torque per amp of iq
        iq_max = math.sqrt(max(self.limits.i_max**2 - id_ref**2, 0.0))
        iq_ref = min(max(te_ref / k_t, -iq_max), iq_max)

        # --- slip frequency and flux angle ---------------------------------
        omega_slip = m.Lm * i_q / (m.tau_r * psi)
        omega_e = m.pole_pairs * omega_m + omega_slip
        self.theta_e = (self.theta_e + omega_e * self.ts) % (2.0 * math.pi)

        # --- inner current loops with decoupling feed-forward ---------------
        vd_ff = -omega_e * m.sigma * m.Ls * i_q
        vq_ff = omega_e * (m.sigma * m.Ls * i_d + (m.Lm / m.Lr) * psi)
        v_d = self.id_pi.update(id_ref - i_d, feedforward=vd_ff)
        v_q = self.iq_pi.update(iq_ref - i_q, feedforward=vq_ff)

        # --- voltage-vector limitation (d axis has priority: it holds flux) -
        v_d = min(max(v_d, -self.v_max), self.v_max)
        vq_lim = math.sqrt(max(self.v_max**2 - v_d**2, 0.0))
        v_q = min(max(v_q, -vq_lim), vq_lim)

        v_alpha_ref, v_beta_ref = inv_park(v_d, v_q, self.theta_e)

        log = {
            "i_d": i_d, "i_q": i_q, "id_ref": id_ref, "iq_ref": iq_ref,
            "te_ref": te_ref, "psi_est": self.psi_est,
            "omega_slip": omega_slip, "omega_e": omega_e,
            "theta_e": self.theta_e, "v_d": v_d, "v_q": v_q,
        }
        return v_alpha_ref, v_beta_ref, log
