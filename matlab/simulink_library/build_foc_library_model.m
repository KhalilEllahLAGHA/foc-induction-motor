function build_foc_library_model()
%BUILD_FOC_LIBRARY_MODEL  Generate foc_im_library.slx from library blocks.
%
%   Version 2 of the Simulink FOC drive, targeting MATLAB R2024b with
%   Simscape Electrical (Specialized Power Systems).  Ready-made library
%   components are used for the drive hardware and the standard control
%   functions:
%
%     * Asynchronous Machine SI Units      (powerlib/Machines)
%     * Universal Bridge (IGBT/diode VSI)  (powerlib/Power Electronics)
%     * DC Voltage Source + powergui       (powerlib)
%     * Current Measurement blocks         (powerlib/Measurements)
%     * abc to dq0 / dq0 to abc transforms (SPS Control & Measurements)
%     * custom SPWM / seven-segment SVPWM  (MATLAB Function)
%     * PID Controller blocks (discrete)   (simulink/Continuous)
%
%   The FOC structure (indirect rotor-flux orientation, slip integration,
%   decoupled current loops) is identical to Version 1.  Block dialogs
%   reference the parameter struct P (foc_init).
%
%   Library block paths and mask parameter names differ between releases,
%   so they are resolved through find_lib_block / set_param_try, which try
%   several candidates and report anything left to set by hand.

mdl = 'foc_im_library';
here = fileparts(mfilename('fullpath'));
slx = fullfile(here, [mdl '.slx']);

% Block dialogs evaluate P.* expressions in the base workspace.
if ~evalin('base', 'exist(''P'', ''var'')')
    addpath(fullfile(here, '..', 'm_files'));
    evalin('base', 'foc_init;');
end

if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(slx), delete(slx); end

% Specialized Power Systems must be available.
try
    load_system('powerlib');
catch
    error(['Simscape Electrical (Specialized Power Systems) not found. ' ...
           'Version 2 requires the powerlib library.']);
end

new_system(mdl);
set_param(mdl, ...
    'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'P.Ts_power', 'StopTime', 'P.T_end', ...
    'AutoInsertRateTranBlk', 'on', ...
    'ReturnWorkspaceOutputs', 'on', 'ScreenColor', 'white');

warn = {};   % collected manual-fix notes

% ======================================================================
% Power stage
% ======================================================================
LIBS = {'sps_lib', 'powerlib', 'powerlib_extras'};
lib = @(name) require_block(find_lib_block(name, LIBS), name);

add_block(lib('powergui'), [mdl '/powergui'], 'Position', [40 30 140 70]);
warn = set_param_try([mdl '/powergui'], warn, ...
    {'SimulationMode'}, 'Discrete', {'SampleTime'}, 'P.Ts_power');

blk(mdl, lib('DC Voltage Source'), 'Vdc', 60, 140, 40, 60, ...
    'Amplitude', 'P.Vdc');

% An SPS network needs one voltage reference node or it is singular.
blk(mdl, lib('Ground'), 'DC ground', 60, 250, 30, 30);

% Device and converterType must be set explicitly: the block defaults to
% thyristors in rectifier configuration, and thyristors cannot be gated off
% by a PWM pattern (they conduct until their current reverses), which
% short-circuits the DC bus through both devices of a leg.
blk(mdl, lib('Universal Bridge'), 'Inverter', 220, 120, 80, 90);
warn = set_param_try([mdl '/Inverter'], warn, ...
    {'Arms'}, '3', ...
    {'Device'}, 'IGBT / Diodes', ...
    {'converterType'}, 'Inverter', ...
    {'Ron'}, '1e-3', ...
    {'SnubberResistance'}, '1e5', ...
    {'SnubberCapacitance'}, 'inf');

blk(mdl, lib('Asynchronous Machine SI Units'), 'IM 5kW', 560, 110, 90, 100);
warn = set_param_try([mdl '/IM 5kW'], warn, ...
    {'RotorType'}, 'Squirrel-cage', ...
    {'PresetModel'}, 'No', ...
    {'Units'}, 'SI', ...
    {'ShowDetailedParameters'}, 'on', ...
    {'MechanicalLoad'}, 'Torque Tm', ...
    {'MeasurementBus'}, 'on', ...   % named signals for the Bus Selector
    {'NominalParameters'}, '[P.Pn P.Vn P.fn]', ...
    {'Stator'}, '[P.Rs P.Lls]', ...
    {'Rotor'}, '[P.Rr P.Llr]', ...
    {'Lm', 'Mutual'}, 'P.Lm', ...
    {'Mechanical'}, '[P.J P.B P.p]', ...
    ...% [slip th ias ibs ics phias phibs phics]; slip = 1 is standstill
    ...% (the default slip of 0 starts the rotor at synchronous speed)
    {'InitialConditions'}, '[1 0 0 0 0 0 0 0]');

blk(mdl, lib('Current Measurement'), 'Ia meas', 390, 125, 40, 30);
blk(mdl, lib('Current Measurement'), 'Ib meas', 390, 175, 40, 30);

% ---- physical (SPS) wiring -------------------------------------------
% DC link polarity: on the DC Voltage Source the RIGHT connection port is
% the positive terminal; on the Universal Bridge RConn1 is "+", RConn2 is
% "-".  Reversing them forward-biases all six freewheel diodes, tying the
% three AC nodes together so the machine sees 0 V and draws no current.
warn = connect_phys(mdl, warn, 'Vdc', 'RConn', 1, 'Inverter', 'RConn', 1);
warn = connect_phys(mdl, warn, 'Vdc', 'LConn', 1, 'Inverter', 'RConn', 2);
warn = connect_phys(mdl, warn, 'Vdc', 'LConn', 1, 'DC ground', 'LConn', 1);
warn = connect_phys(mdl, warn, 'Inverter', 'LConn', 1, 'Ia meas', 'LConn', 1);
warn = connect_phys(mdl, warn, 'Inverter', 'LConn', 2, 'Ib meas', 'LConn', 1);
warn = connect_phys(mdl, warn, 'Ia meas', 'RConn', 1, 'IM 5kW', 'LConn', 1);
warn = connect_phys(mdl, warn, 'Ib meas', 'RConn', 1, 'IM 5kW', 'LConn', 2);
warn = connect_phys(mdl, warn, 'Inverter', 'LConn', 3, 'IM 5kW', 'LConn', 3);

% ---- phase-current vector  ic = -(ia+ib) ------------------------------
blk(mdl, 'simulink/Math Operations/Sum', 'ic calc', 470, 230, 35, 45, ...
    'Inputs', '--', 'IconShape', 'rectangular');
blk(mdl, 'simulink/Signal Routing/Mux', 'mux iabc', 530, 240, 8, 90, ...
    'Inputs', '3');
ln(mdl, 'Ia meas/1', 'ic calc/1');
ln(mdl, 'Ib meas/1', 'ic calc/2');
ln(mdl, 'Ia meas/1', 'mux iabc/1');
ln(mdl, 'Ib meas/1', 'mux iabc/2');
ln(mdl, 'ic calc/1', 'mux iabc/3');

% ---- machine measurement bus: rotor speed + torque --------------------
blk(mdl, 'simulink/Signal Routing/Bus Selector', 'm select', ...
    700, 120, 8, 80);

% ======================================================================
% References and controller
% ======================================================================
add_block('built-in/Subsystem', [mdl '/Speed Reference'], ...
          'Position', [40 410 180 480]);
build_speed_reference([mdl '/Speed Reference']);

blk(mdl, 'simulink/Sources/Step', 'Load torque', 420, 40, 45, 40, ...
    'Time', 'P.t_load', 'Before', '0', 'After', 'P.TL');

add_block('built-in/Subsystem', [mdl '/FOC Controller'], ...
          'Position', [240 390 420 560]);
warn = build_controller([mdl '/FOC Controller'], warn, LIBS);

configure_foc_pwm(mdl);

ln(mdl, 'Speed Reference/1', 'FOC Controller/1');
ln(mdl, 'Load torque/1', 'IM 5kW/1');           % Tm input
ln(mdl, 'IM 5kW/1', 'm select/1');              % measurement bus
ln(mdl, 'mux iabc/1', 'FOC Controller/3');

% ======================================================================
% Logging (same variable names as Version 1 -> shared plot_results.m)
% ======================================================================
tw = {'w_ref', 950, 420; 'TL', 950, 40; 'i_abc', 950, 300; ...
      'wm', 950, 100; 'Te', 950, 160; 'id', 950, 480; 'iq', 950, 530; ...
      'iq_ref', 950, 580; 'psi_est', 950, 630};
for k = 1:size(tw, 1)
    blk(mdl, 'simulink/Sinks/To Workspace', ['log_' tw{k,1}], ...
        tw{k,2}, tw{k,3}, 90, 30, ...
        'VariableName', tw{k,1}, 'SaveFormat', 'Timeseries');
end
ln(mdl, 'Speed Reference/1', 'log_w_ref/1');
ln(mdl, 'Load torque/1', 'log_TL/1');
ln(mdl, 'mux iabc/1', 'log_i_abc/1');
ln(mdl, 'm select/1', 'log_wm/1');
ln(mdl, 'm select/2', 'log_Te/1');
ln(mdl, 'm select/1', 'FOC Controller/2');      % wm feedback
ln(mdl, 'FOC Controller/2', 'log_id/1');
ln(mdl, 'FOC Controller/3', 'log_iq/1');
ln(mdl, 'FOC Controller/4', 'log_iq_ref/1');
ln(mdl, 'FOC Controller/5', 'log_psi_est/1');

% ======================================================================
% Resolve the machine measurement bus (signal names differ per release):
% compile once, read the available signals, pick rotor speed + torque.
% ======================================================================
% Resolve the machine measurement bus.  The names are read without
% compiling: a Bus Selector holding a selection that is not on the bus
% makes compilation fail, so a compile-first approach cannot repair itself.
sigs = {};
try
    sigs = flatten_signals(get_param([mdl '/m select'], 'InputSignals'), '');
catch
end
if isempty(sigs)
    try
        feval(mdl, [], [], [], 'compile');
        sigs = flatten_signals(get_param([mdl '/m select'], 'InputSignals'), '');
    catch
    end
    try  %#ok<*CTCH>  a failed compile must still leave the model editable
        feval(mdl, [], [], [], 'term');
    catch
    end
end

% R2024b uses short bus names ('w', 'Te'); older releases spell them out.
wm_sig = pick_signal(sigs, {'w', 'wm', 'rotor speed'});
te_sig = pick_signal(sigs, {'Te', 'electromagnetic torque'});
if isempty(wm_sig) || isempty(te_sig)
    warn{end+1} = ['Bus Selector ''m select'': could not identify the ' ...
                   'rotor-speed / torque signals - select them by hand.'];
else
    set_param([mdl '/m select'], 'OutputSignals', [wm_sig ',' te_sig]);
end

arrange_all(mdl);
save_system(mdl, slx);
fprintf('generated %s\n', slx);
if ~isempty(warn)
    fprintf('\nmanual checks needed:\n');
    fprintf('  - %s\n', warn{:});
end
end

% ======================================================================
% FOC controller  (library transforms + discrete PID blocks)
%   in : 1 w_ref, 2 wm, 3 i_abc
%   out: 1 Uref(3ph, p.u.), 2 id, 3 iq, 4 iq_ref, 5 psi_est
% ======================================================================
function warn = build_controller(sys, warn, LIBS)
blk(sys, 'simulink/Sources/In1', 'w_ref', 30, 300, 30, 15);
blk(sys, 'simulink/Sources/In1', 'wm', 30, 360, 30, 15);
blk(sys, 'simulink/Sources/In1', 'i_abc', 30, 40, 30, 15);

% controller sampling (drive measurements to the 10 kHz control rate)
blk(sys, 'simulink/Discrete/Zero-Order Hold', 'zoh i', 90, 38, 50, 30, ...
    'SampleTime', 'P.Ts_ctrl');
blk(sys, 'simulink/Discrete/Zero-Order Hold', 'zoh w', 90, 353, 50, 30, ...
    'SampleTime', 'P.Ts_ctrl');

% ---- abc -> dq0 (library transform, angle input wt = theta) -----------
abc2dq = find_lib_block('abc to dq0', LIBS);
dq2abc = find_lib_block('dq0 to abc', LIBS);
if isempty(abc2dq) || isempty(dq2abc)
    error(['Transformation blocks ''abc to dq0'' / ''dq0 to abc'' not ' ...
           'found in the SPS libraries - check the Simscape Electrical install.']);
end
add_block(abc2dq, [sys '/abc to dq0'], 'Position', [200 30 280 80]);
add_block(dq2abc, [sys '/dq0 to abc'], 'Position', [830 330 910 380]);

blk(sys, 'simulink/Signal Routing/Demux', 'split dq0', 320, 30, 8, 60, ...
    'Outputs', '3');
blk(sys, 'simulink/Sinks/Terminator', 'zero seq', 370, 100, 30, 25);

% Convention adapter.  The SPS transform blocks are sine-referenced:
%     d_sps = 2/3*sum(x_k*sin(wt - k*2pi/3))
%     q_sps = 2/3*sum(x_k*cos(wt - k*2pi/3))
% whereas this controller uses the cosine-referenced Park transform, so
%     q_sps = d,   d_sps = -q
% The axes must therefore be swapped and one sign flipped on both the
% measurement and the actuation path; otherwise the frame sits 90 degrees
% off the rotor flux and field orientation is lost.
blk(sys, 'simulink/Math Operations/Gain', 'iq sign', 380, 33, 55, 30, ...
    'Gain', '-1');

% ---- flux estimator + slip + angle ------------------------------------
blk(sys, 'simulink/Discrete/Discrete Transfer Fcn', 'rotor lag', ...
    420, 120, 90, 45, 'Numerator', '[P.k_est]', ...
    'Denominator', '[1 -P.a_est]', 'SampleTime', 'P.Ts_ctrl');
blk(sys, 'simulink/Sources/Constant', 'psi min', 420, 185, 60, 30, ...
    'Value', 'P.psi_min');
blk(sys, 'simulink/Math Operations/MinMax', 'guard', 540, 140, 55, 40, ...
    'Function', 'max', 'Inputs', '2');
blk(sys, 'simulink/Math Operations/Divide', 'iq over psi', 540, 220, 45, 40, ...
    'Inputs', '*/');
blk(sys, 'simulink/Math Operations/Gain', 'Lm over tau', 620, 225, 70, 30, ...
    'Gain', 'P.Lm/P.tau_r');
blk(sys, 'simulink/Math Operations/Gain', 'pole pairs', 540, 290, 50, 30, ...
    'Gain', 'P.p');
blk(sys, 'simulink/Math Operations/Sum', 'we sum', 720, 250, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Discrete/Discrete-Time Integrator', 'theta int', ...
    790, 252, 45, 40, 'SampleTime', 'P.Ts_ctrl');

% ---- outer speed loop (library PID block, PI form) --------------------
blk(sys, 'simulink/Continuous/PID Controller', 'Speed PI', 150, 415, 60, 45);
warn = set_param_try([sys '/Speed PI'], warn, ...
    {'Controller'}, 'PI', {'TimeDomain'}, 'Discrete-time', ...
    {'SampleTime'}, 'P.Ts_ctrl', {'P'}, 'P.Kp_w', {'I'}, 'P.Ki_w', ...
    {'LimitOutput'}, 'on', {'UpperSaturationLimit'}, 'P.Te_max', ...
    {'LowerSaturationLimit'}, '-P.Te_max', {'AntiWindupMode'}, 'clamping');
blk(sys, 'simulink/Math Operations/Sum', 'w err', 90, 420, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');

blk(sys, 'simulink/Math Operations/Gain', 'Kt psi', 620, 165, 55, 30, ...
    'Gain', 'P.Kt');
blk(sys, 'simulink/Math Operations/Divide', 'Te to iq', 260, 420, 45, 40, ...
    'Inputs', '*/');
blk(sys, 'simulink/Discontinuities/Saturation', 'iq limit', 330, 425, 45, 30, ...
    'UpperLimit', 'P.iq_max', 'LowerLimit', '-P.iq_max');
blk(sys, 'simulink/Sources/Constant', 'id ref', 330, 520, 60, 30, ...
    'Value', 'P.id_ref');

% ---- current loops (library PID blocks + decoupling) -------------------
blk(sys, 'simulink/Math Operations/Sum', 'iq err', 420, 430, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'id err', 420, 520, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
for nm = {'Iq PI', 'Id PI'}
    y = 425 + 95*strcmp(nm{1}, 'Id PI');
    blk(sys, 'simulink/Continuous/PID Controller', nm{1}, 480, y, 60, 45);
    warn = set_param_try([sys '/' nm{1}], warn, ...
        {'Controller'}, 'PI', {'TimeDomain'}, 'Discrete-time', ...
        {'SampleTime'}, 'P.Ts_ctrl', {'P'}, 'P.Kp_i', {'I'}, 'P.Ki_i', ...
        {'LimitOutput'}, 'on', {'UpperSaturationLimit'}, 'P.v_max', ...
        {'LowerSaturationLimit'}, '-P.v_max', {'AntiWindupMode'}, 'clamping');
end

% decoupling feed-forward (see CONTEXT.md sec. 2.2)
blk(sys, 'simulink/Math Operations/Product', 'we x iq', 560, 600, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'neg sigmaLs', 620, 602, 75, 30, ...
    'Gain', '-P.sigma*P.Ls');
blk(sys, 'simulink/Math Operations/Gain', 'sigmaLs', 480, 660, 60, 30, ...
    'Gain', 'P.sigma*P.Ls');
blk(sys, 'simulink/Math Operations/Gain', 'Lm over Lr', 480, 710, 60, 30, ...
    'Gain', 'P.Lm/P.Lr');
blk(sys, 'simulink/Math Operations/Sum', 'ff sum', 570, 675, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Product', 'we x ff', 630, 670, 40, 35);
blk(sys, 'simulink/Math Operations/Sum', 'vq sum', 620, 430, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'vd sum', 620, 520, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');

% dq -> abc, normalised to the PWM generator's +/-1 input range.
blk(sys, 'simulink/Math Operations/Gain', 'vq sign', 700, 425, 55, 30, ...
    'Gain', '-1');
blk(sys, 'simulink/Sources/Constant', 'zero', 700, 375, 45, 25, 'Value', '0');
blk(sys, 'simulink/Signal Routing/Mux', 'mux dq0', 770, 330, 8, 80, ...
    'Inputs', '3');
blk(sys, 'simulink/Math Operations/Gain', 'to pu', 950, 340, 55, 30, ...
    'Gain', '2/P.Vdc');

blk(sys, 'simulink/Sinks/Out1', 'Uref', 1040, 347, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'id out', 1040, 40, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'iq out', 1040, 85, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'iq_ref out', 1040, 130, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'psi_est out', 1040, 175, 30, 15);

% ---- wiring ------------------------------------------------------------
ln(sys, 'i_abc/1', 'zoh i/1');
ln(sys, 'zoh i/1', 'abc to dq0/1');
ln(sys, 'theta int/1', 'abc to dq0/2');         % wt input
% id = 'split dq0/2' (= q_sps), iq = 'iq sign/1' (= -d_sps).
ln(sys, 'abc to dq0/1', 'split dq0/1');
ln(sys, 'split dq0/1', 'iq sign/1');
ln(sys, 'split dq0/3', 'zero seq/1');
ln(sys, 'wm/1', 'zoh w/1');

ln(sys, 'split dq0/2', 'rotor lag/1');          % id -> flux estimator
ln(sys, 'rotor lag/1', 'guard/1');
ln(sys, 'psi min/1', 'guard/2');
ln(sys, 'iq sign/1', 'iq over psi/1');
ln(sys, 'guard/1', 'iq over psi/2');
ln(sys, 'iq over psi/1', 'Lm over tau/1');
ln(sys, 'zoh w/1', 'pole pairs/1');
ln(sys, 'Lm over tau/1', 'we sum/1');
ln(sys, 'pole pairs/1', 'we sum/2');
ln(sys, 'we sum/1', 'theta int/1');

ln(sys, 'w_ref/1', 'w err/1');
ln(sys, 'zoh w/1', 'w err/2');
ln(sys, 'w err/1', 'Speed PI/1');
ln(sys, 'guard/1', 'Kt psi/1');
ln(sys, 'Speed PI/1', 'Te to iq/1');
ln(sys, 'Kt psi/1', 'Te to iq/2');
ln(sys, 'Te to iq/1', 'iq limit/1');

ln(sys, 'iq limit/1', 'iq err/1');
ln(sys, 'iq sign/1', 'iq err/2');
ln(sys, 'iq err/1', 'Iq PI/1');
ln(sys, 'id ref/1', 'id err/1');
ln(sys, 'split dq0/2', 'id err/2');
ln(sys, 'id err/1', 'Id PI/1');

ln(sys, 'we sum/1', 'we x iq/1');
ln(sys, 'iq sign/1', 'we x iq/2');
ln(sys, 'we x iq/1', 'neg sigmaLs/1');
ln(sys, 'split dq0/2', 'sigmaLs/1');
ln(sys, 'guard/1', 'Lm over Lr/1');
ln(sys, 'sigmaLs/1', 'ff sum/1');
ln(sys, 'Lm over Lr/1', 'ff sum/2');
ln(sys, 'we sum/1', 'we x ff/1');
ln(sys, 'ff sum/1', 'we x ff/2');

ln(sys, 'Iq PI/1', 'vq sum/1');
ln(sys, 'we x ff/1', 'vq sum/2');
ln(sys, 'Id PI/1', 'vd sum/1');
ln(sys, 'neg sigmaLs/1', 'vd sum/2');

ln(sys, 'vq sum/1', 'vq sign/1');
ln(sys, 'vq sign/1', 'mux dq0/1');              % d_sps = -vq
ln(sys, 'vd sum/1', 'mux dq0/2');               % q_sps =  vd
ln(sys, 'zero/1', 'mux dq0/3');
ln(sys, 'mux dq0/1', 'dq0 to abc/1');
ln(sys, 'theta int/1', 'dq0 to abc/2');         % wt input
ln(sys, 'dq0 to abc/1', 'to pu/1');
ln(sys, 'to pu/1', 'Uref/1');

ln(sys, 'split dq0/2', 'id out/1');
ln(sys, 'iq sign/1', 'iq out/1');
ln(sys, 'iq limit/1', 'iq_ref out/1');
ln(sys, 'rotor lag/1', 'psi_est out/1');
end

% ======================================================================
% Auto-arrange the top level and every subsystem so the saved diagram is
% readable (block spacing, signal routing, no overlaps).
% ======================================================================
function arrange_all(mdl)
systems = [{mdl}; find_system(mdl, 'BlockType', 'SubSystem')];
for k = 1:numel(systems)
    try
        Simulink.BlockDiagram.arrangeSystem(systems{k});
    catch
    end
end
end

% ======================================================================
% Speed Reference generator (same staircase profile as Version 1).
%
% Four Step blocks, each carrying the *increment* between consecutive
% levels, summed together ->
%   0 -> 800 -> 1400 -> 600 -> -800 rpm at the instants P.step_t.
%   out: 1 w_ref [rad/s mech]
% ======================================================================
function build_speed_reference(sys)
for k = 1:4
    blk(sys, 'simulink/Sources/Step', sprintf('step%d', k), ...
        40, 30 + 50*(k-1), 45, 35, ...
        'Time', sprintf('P.step_t(%d)', k), 'Before', '0', ...
        'After', sprintf('P.step_d(%d)', k));
end
blk(sys, 'simulink/Math Operations/Sum', 'staircase', 150, 60, 35, 150, ...
    'Inputs', '++++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Sinks/Out1', 'w_ref', 240, 127, 30, 15);

for k = 1:4
    ln(sys, sprintf('step%d/1', k), sprintf('staircase/%d', k));
end
ln(sys, 'staircase/1', 'w_ref/1');
end

% ======================================================================
% helpers
% ======================================================================
function blk(sys, lib, name, x, y, w, h, varargin)
add_block(lib, [sys '/' name], 'Position', [x y x+w y+h], varargin{:});
end

function ln(sys, from, to)
add_line(sys, from, to, 'autorouting', 'smart');
end

function path = find_lib_block(name, libs)
%FIND_LIB_BLOCK  Resolve an SPS library block path from its name.
%
%   The lookup is whitespace-insensitive because several library block
%   names contain line breaks ('PWM Generator\n(2-Level)'), and it looks
%   under masks because the blocks are masked subsystems.  The index is
%   built once per session and cached.
persistent index
if isempty(index)
    index = containers.Map('KeyType', 'char', 'ValueType', 'char');
    for k = 1:numel(libs)
        try
            load_system(libs{k});
            hits = find_system(libs{k}, 'LookUnderMasks', 'all', ...
                               'FollowLinks', 'off', 'Type', 'Block');
        catch
            continue
        end
        for j = 1:numel(hits)
            key = norm_name(get_param(hits{j}, 'Name'));
            if ~isKey(index, key)
                index(key) = hits{j};
            end
        end
    end
end

key = norm_name(name);
if isKey(index, key)
    path = index(key);
else
    path = '';
end
end

function s = norm_name(s)
s = strtrim(lower(regexprep(s, '\s+', ' ')));
end

function p = require_block(p, name)
% Fail early and clearly if an expected library block is missing.
if isempty(p)
    error('build_foc_library_model:missingBlock', ...
          ['Library block ''%s'' not found in the Specialized Power ' ...
           'Systems libraries - check the Simscape Electrical install.'], ...
          name);
end
end

function warn = set_param_try(block, warn, varargin)
% set_param that tries several candidate parameter names per value:
%   set_param_try(blk, warn, {'Fc','CarrierFrequency'}, 'P.fsw', ...)
for k = 1:2:numel(varargin)
    names = varargin{k};
    value = varargin{k+1};
    ok = false;
    for n = 1:numel(names)
        try
            set_param(block, names{n}, value);
            ok = true;
            break
        catch
        end
    end
    if ~ok
        warn{end+1} = sprintf('%s: set ''%s'' = %s manually.', ...
                              block, names{1}, value); %#ok<AGROW>
    end
end
end

function warn = connect_phys(mdl, warn, src, srcSide, srcIdx, dst, dstSide, dstIdx)
% Connect two Specialized Power Systems (physical) ports via port handles.
try
    hs = get_param([mdl '/' src], 'PortHandles');
    hd = get_param([mdl '/' dst], 'PortHandles');
    add_line(mdl, hs.(srcSide)(srcIdx), hd.(dstSide)(dstIdx));
catch
    warn{end+1} = sprintf('connect %s(%s%d) to %s(%s%d) manually.', ...
                          src, srcSide, srcIdx, dst, dstSide, dstIdx); %#ok<AGROW>
end
end

function names = flatten_signals(s, prefix)
% Flatten a Bus Selector 'InputSignals' cell tree into dotted signal paths.
names = {};
if ischar(s)
    s = {s};
end
for k = 1:numel(s)
    e = s{k};
    if ischar(e)
        names{end+1} = join_dot(prefix, e); %#ok<AGROW>
    elseif iscell(e) && numel(e) == 2 && ischar(e{1}) && iscell(e{2})
        names = [names, flatten_signals(e{2}, join_dot(prefix, e{1}))]; %#ok<AGROW>
    elseif iscell(e)
        names = [names, flatten_signals(e, prefix)]; %#ok<AGROW>
    end
end
end

function s = join_dot(a, b)
if isempty(a), s = b; else, s = [a '.' b]; end
end

function match = pick_signal(sigs, needles)
% Resolve a bus signal from a list of candidate names, most specific
% first: exact leaf-name matches are preferred over substring matches so
% that a short name like 'w' cannot be shadowed by 'vqs'/'vds'.
if ischar(needles), needles = {needles}; end
leaf = cell(size(sigs));
for k = 1:numel(sigs)
    parts = strsplit(sigs{k}, '.');
    leaf{k} = lower(strtrim(parts{end}));
end

for n = 1:numel(needles)
    hit = find(strcmp(leaf, lower(needles{n})), 1);
    if ~isempty(hit), match = sigs{hit}; return; end
end
for n = 1:numel(needles)
    hit = find(contains(lower(sigs), lower(needles{n})), 1);
    if ~isempty(hit), match = sigs{hit}; return; end
end
match = '';
end
