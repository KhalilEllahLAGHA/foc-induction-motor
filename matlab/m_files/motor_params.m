function P = motor_params()
%MOTOR_PARAMS  All parameters of the FOC induction-motor drive (struct P).
%
%   Single source of truth for the MATLAB side of the project; the values
%   match python/main.py so the results are directly comparable.
%
%   The Simulink models reference these values as P.<field> in their block
%   dialogs, so P must exist in the base workspace before simulating
%   (run foc_init, or the run_foc_* scripts which call it).

% ---- Motor: 5 kW, 380 V, 10.5 A, 50 Hz squirrel cage --------------------
% Ls and Lr are given as total inductances (0.071 H), so the leakages are
% Lls = Llr = Ls - Lm = 0.002 H.
P.Pn  = 5e3;        % nominal power                     [W]
P.Vn  = 380;        % nominal line-line voltage (rms)   [V]
P.fn  = 50;         % nominal frequency                 [Hz]
P.Rs  = 0.435;      % stator resistance                 [ohm]
P.Rr  = 0.816;      % rotor resistance (referred)       [ohm]
P.Lls = 0.002;      % stator leakage inductance         [H]
P.Llr = 0.002;      % rotor leakage inductance (ref.)   [H]
P.Lm  = 0.069;      % magnetizing inductance            [H]
P.p   = 2;          % pole pairs                        [-]
P.J   = 0.0056;     % rotor inertia                     [kg*m^2]
P.B   = 0.0015;     % viscous friction                  [N*m*s]

% ---- Derived machine quantities -----------------------------------------
P.Ls    = P.Lls + P.Lm;                    % total stator inductance [H]
P.Lr    = P.Llr + P.Lm;                    % total rotor inductance  [H]
P.sigma = 1 - P.Lm^2/(P.Ls*P.Lr);          % total leakage factor    [-]
P.tau_r = P.Lr/P.Rr;                       % rotor time constant     [s]
P.Rsig  = P.Rs + P.Rr*(P.Lm/P.Lr)^2;       % equivalent resistance   [ohm]
P.Kt    = 1.5*P.p*P.Lm/P.Lr;               % Te = Kt * psi_r * iq    [N*m/(Wb*A)]

% ---- Inverter / sampling -------------------------------------------------
P.Vdc      = 540;            % DC-link voltage (~sqrt(2)*380)    [V]
P.fsw      = 10e3;           % switching / carrier frequency     [Hz]
P.Ts_ctrl  = 1e-4;           % control sample time (= PWM rate)  [s]
% Must divide both the control period (100 us) and the PWM generator's
% internal quarter-carrier offset 1/(4*fsw) = 25 us, or Simulink rejects
% the fixed-step size.
P.Ts_power = 2.5e-6;         % power-stage discretization step   [s]
P.v_max    = P.Vdc/sqrt(3);  % SVPWM linear phase-voltage limit  [V]

% ---- References and limits ----------------------------------------------
P.psi_ref = 0.8;                         % rotor flux reference        [Wb]
P.id_ref  = P.psi_ref/P.Lm;              % flux-producing current ref  [A]
P.I_max   = 25;                          % stator current limit        [A]
P.iq_max  = sqrt(P.I_max^2 - P.id_ref^2);% torque-current limit        [A]
P.Te_max  = 35;                          % speed-loop torque limit     [N*m]
P.psi_min = 0.05;                        % divide-by-zero guard        [Wb]

% ---- PI gains (bandwidth design, see CONTEXT.md sec. 2.5) ---------------
P.bw_i = 2*pi*400;                 % current-loop bandwidth  [rad/s]
P.bw_w = 2*pi*30;                  % speed-loop bandwidth    [rad/s]
P.Kp_i = P.sigma*P.Ls*P.bw_i;      % current PI              [V/A]
P.Ki_i = P.Rsig*P.bw_i;            %                         [V/(A*s)]
P.Kp_w = P.J*P.bw_w;               % speed PI                [N*m*s/rad]
P.Ki_w = P.Kp_w*P.bw_w/5;          %                         [N*m/rad]

% Anti-windup: the integrator is unwound with time constant 1/Kb_aw
% whenever a controller output saturates.
P.Kb_aw = 1/(10*P.Ts_ctrl);        % [1/s]

% ---- Discrete flux estimator  psi(k+1) = a*psi(k) + k*id(k) -------------
% (forward-Euler version of  Lm/(tau_r*s + 1), sampled at Ts_ctrl)
P.k_est = P.Lm*P.Ts_ctrl/P.tau_r;
P.a_est = 1 - P.Ts_ctrl/P.tau_r;

% =========================================================================
% TEST SCENARIO - multi-level constant speeds, step changes only
% =========================================================================
% A staircase of constant speed levels, each reached by a sudden step, with
% a load torque applied part way through.  The ramp/sine tracking study of
% REPORT.md section 7 is run only by the Python simulation.

P.step_t   = [0.30 0.80 1.30 1.80];   % step instants                  [s]
P.step_rpm = [ 800 1400  600 -800];   % constant level reached by each [rpm]
P.step_w   = P.step_rpm*2*pi/60;      %                                [rad/s]
P.step_d   = diff([0 P.step_w]);      % increment applied at each step [rad/s]

P.t_load   = 0.55;                    % load-torque step instant       [s]
P.TL       = 10;                      % load torque after the step     [N*m]
P.T_end    = 2.4;                     % simulated time                 [s]
end
