"""Dynamic dq-frame model of a squirrel-cage induction motor.

The electrical model is written in the stationary (alpha-beta) reference frame
with stator currents and rotor flux linkages as state variables.  Keeping the
plant in the stationary frame rather than the controller's synchronous frame
makes the motor model independent of the control algorithm: the controller
supplies phase voltages, the model returns phase currents, speed, torque and
rotor flux.

State vector
------------
    x = [i_s_alpha, i_s_beta, psi_r_alpha, psi_r_beta, omega_m]

Electrical equations (amplitude-invariant convention)
-----------------------------------------------------
With  sigma = 1 - Lm^2/(Ls*Lr),  tau_r = Lr/Rr,  R_sigma = Rs + Rr*(Lm/Lr)^2
and electrical rotor speed  omega_r = p * omega_m:

    d(psi_ra)/dt = -psi_ra/tau_r - omega_r*psi_rb + (Lm/tau_r)*i_sa
    d(psi_rb)/dt = -psi_rb/tau_r + omega_r*psi_ra + (Lm/tau_r)*i_sb
    d(i_sa)/dt   = [ v_sa - R_sigma*i_sa + (Lm/(Lr*tau_r))*psi_ra
                     + (Lm/Lr)*omega_r*psi_rb ] / (sigma*Ls)
    d(i_sb)/dt   = [ v_sb - R_sigma*i_sb + (Lm/(Lr*tau_r))*psi_rb
                     - (Lm/Lr)*omega_r*psi_ra ] / (sigma*Ls)

Torque and mechanics
--------------------
    Te = (3/2) * p * (Lm/Lr) * (psi_ra*i_sb - psi_rb*i_sa)
    J * d(omega_m)/dt = Te - T_load - B*omega_m

Integration uses a fixed-step 4th-order Runge-Kutta scheme with the stator
voltage held constant over the step (zero-order hold, consistent with an
inverter-fed drive).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from control.pid import inv_clarke


@dataclass
class InductionMotorParams:
    """Nameplate/equivalent-circuit parameters of the induction motor."""

    Rs: float = 1.405        # stator resistance                [ohm]
    Rr: float = 1.395        # rotor resistance (referred)      [ohm]
    Lls: float = 0.005839    # stator leakage inductance        [H]
    Llr: float = 0.005839    # rotor leakage inductance (ref.)  [H]
    Lm: float = 0.1722       # magnetizing inductance           [H]
    pole_pairs: int = 2      # number of pole pairs             [-]
    J: float = 0.0131        # rotor inertia                    [kg*m^2]
    B: float = 0.002985      # viscous friction coefficient     [N*m*s]

    # ---- derived quantities (computed, not user inputs) -------------------
    Ls: float = field(init=False)       # total stator inductance   [H]
    Lr: float = field(init=False)       # total rotor inductance    [H]
    sigma: float = field(init=False)    # total leakage factor      [-]
    tau_r: float = field(init=False)    # rotor time constant       [s]
    R_sigma: float = field(init=False)  # equivalent series resist. [ohm]

    def __post_init__(self) -> None:
        self.Ls = self.Lls + self.Lm
        self.Lr = self.Llr + self.Lm
        self.sigma = 1.0 - self.Lm**2 / (self.Ls * self.Lr)
        self.tau_r = self.Lr / self.Rr
        self.R_sigma = self.Rs + self.Rr * (self.Lm / self.Lr) ** 2


class InductionMotor:
    """Squirrel-cage induction motor, stationary-frame dq dynamic model."""

    def __init__(self, params: InductionMotorParams) -> None:
        self.p = params
        # State: [i_sa, i_sb, psi_ra, psi_rb, omega_m]
        self.x = np.zeros(5)

    # ------------------------------------------------------------------ API
    @property
    def i_alpha_beta(self) -> tuple[float, float]:
        """Stator current space vector components in the stationary frame [A]."""
        return float(self.x[0]), float(self.x[1])

    @property
    def i_abc(self) -> tuple[float, float, float]:
        """Instantaneous stator phase currents [A]."""
        return inv_clarke(self.x[0], self.x[1])

    @property
    def psi_r_alpha_beta(self) -> tuple[float, float]:
        """Rotor flux linkage components in the stationary frame [Wb]."""
        return float(self.x[2]), float(self.x[3])

    @property
    def psi_r_mag(self) -> float:
        """Rotor flux linkage magnitude [Wb]."""
        return math.hypot(self.x[2], self.x[3])

    @property
    def omega_m(self) -> float:
        """Mechanical rotor speed [rad/s]."""
        return float(self.x[4])

    @property
    def torque(self) -> float:
        """Electromagnetic torque at the current state [N*m]."""
        return self._torque(self.x)

    # ------------------------------------------------------------ internals
    def _torque(self, x: np.ndarray) -> float:
        p = self.p
        return 1.5 * p.pole_pairs * (p.Lm / p.Lr) * (x[2] * x[1] - x[3] * x[0])

    def _derivatives(self, x: np.ndarray, v_sa: float, v_sb: float,
                     t_load: float) -> np.ndarray:
        """Right-hand side of the state equations."""
        p = self.p
        i_sa, i_sb, psi_ra, psi_rb, omega_m = x
        omega_r = p.pole_pairs * omega_m          # electrical rotor speed

        lm_tr = p.Lm / p.tau_r
        k_r = p.Lm / p.Lr                          # rotor coupling factor

        dpsi_ra = -psi_ra / p.tau_r - omega_r * psi_rb + lm_tr * i_sa
        dpsi_rb = -psi_rb / p.tau_r + omega_r * psi_ra + lm_tr * i_sb

        di_sa = (v_sa - p.R_sigma * i_sa + k_r / p.tau_r * psi_ra
                 + k_r * omega_r * psi_rb) / (p.sigma * p.Ls)
        di_sb = (v_sb - p.R_sigma * i_sb + k_r / p.tau_r * psi_rb
                 - k_r * omega_r * psi_ra) / (p.sigma * p.Ls)

        te = self._torque(x)
        domega_m = (te - t_load - p.B * omega_m) / p.J

        return np.array([di_sa, di_sb, dpsi_ra, dpsi_rb, domega_m])

    # ---------------------------------------------------------------- solve
    def step(self, v_sa: float, v_sb: float, t_load: float, dt: float) -> None:
        """Advance the model by ``dt`` seconds (RK4, voltages held constant).

        Parameters
        ----------
        v_sa, v_sb : float
            Stator voltage in the stationary frame [V] (from the inverter).
        t_load : float
            Load torque [N*m].
        dt : float
            Integration step [s].
        """
        x = self.x
        k1 = self._derivatives(x, v_sa, v_sb, t_load)
        k2 = self._derivatives(x + 0.5 * dt * k1, v_sa, v_sb, t_load)
        k3 = self._derivatives(x + 0.5 * dt * k2, v_sa, v_sb, t_load)
        k4 = self._derivatives(x + dt * k3, v_sa, v_sb, t_load)
        self.x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
