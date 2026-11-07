function build_foc_blocks_model()
%BUILD_FOC_BLOCKS_MODEL  Generate foc_im_blocks.slx from fundamental blocks.
%
%   Version 1 of the Simulink FOC drive: everything (induction-motor dq
%   model, average-value SVPWM inverter, Clarke/Park transforms, PI loops,
%   slip/angle computation) is built ONLY from basic Simulink blocks —
%   Gain, Sum, Product, Integrator, Transfer Fcn, Trigonometry, MinMax,
%   Saturation, Mux/Demux.  No toolbox blocks are used.
%
%   The generated model references the parameter struct P (run foc_init
%   first).  Layout is organized in subsystems:
%
%     [Speed ref] --> [FOC Controller] --> [SVPWM Inverter (avg)] --> [Induction Motor]
%                        ^      ^                                        |  |  |
%                        |      +--- i_abc ------------------------------+  |  |
%                        +---------- wm ------------------------------------+  |
%                                                    To Workspace logging <----+
%
%   Written for MATLAB/Simulink R2024b (works on most earlier releases too).

mdl = 'foc_im_blocks';
here = fileparts(mfilename('fullpath'));
slx = fullfile(here, [mdl '.slx']);
addpath(fullfile(here, '..', 'm_files'));

if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile(slx), delete(slx); end

new_system(mdl);
set_param(mdl, ...
    'SolverType', 'Variable-step', 'Solver', 'ode23tb', ...
    'StopTime', 'P.T_end', 'MaxStep', '1e-4', 'RelTol', '1e-4', ...
    'ReturnWorkspaceOutputs', 'on', 'ScreenColor', 'white');

% ======================================================================
% Top level
% ======================================================================
add_block('built-in/Subsystem', [mdl '/Speed Reference'], ...
          'Position', [40 50 180 120]);
build_speed_reference([mdl '/Speed Reference']);

blk(mdl, 'simulink/Sources/Step', 'Load torque', 640, 330, 45, 40, ...
    'Time', 'P.t_load', 'Before', '0', 'After', 'P.TL');

add_block('built-in/Subsystem', [mdl '/FOC Controller'], ...
          'Position', [260 40 420 220]);
add_block('built-in/Subsystem', [mdl '/SVPWM Inverter (avg)'], ...
          'Position', [500 55 620 125]);
add_block('built-in/Subsystem', [mdl '/Induction Motor'], ...
          'Position', [700 50 860 210]);

build_controller([mdl '/FOC Controller']);
build_inverter([mdl '/SVPWM Inverter (avg)']);
build_motor([mdl '/Induction Motor']);

% Logging (one To Workspace block per signal, all Timeseries)
tw = {'w_ref', 950, 260; 'TL', 950, 330; 'i_abc', 950, 40; ...
      'wm', 950, 95; 'Te', 950, 150; 'id', 950, 400; 'iq', 950, 455; ...
      'iq_ref', 950, 510; 'psi_est', 950, 565};
for k = 1:size(tw, 1)
    blk(mdl, 'simulink/Sinks/To Workspace', ['log_' tw{k,1}], ...
        tw{k,2}, tw{k,3}, 90, 30, ...
        'VariableName', tw{k,1}, 'SaveFormat', 'Timeseries');
end

% Wiring
ln(mdl, 'Speed Reference/1', 'FOC Controller/1');
ln(mdl, 'Speed Reference/1', 'log_w_ref/1');
ln(mdl, 'Load torque/1', 'Induction Motor/3');
ln(mdl, 'Load torque/1', 'log_TL/1');
ln(mdl, 'FOC Controller/1', 'SVPWM Inverter (avg)/1');
ln(mdl, 'FOC Controller/2', 'SVPWM Inverter (avg)/2');
ln(mdl, 'SVPWM Inverter (avg)/1', 'Induction Motor/1');
ln(mdl, 'SVPWM Inverter (avg)/2', 'Induction Motor/2');
ln(mdl, 'Induction Motor/1', 'FOC Controller/3');   % i_abc feedback
ln(mdl, 'Induction Motor/2', 'FOC Controller/2');   % wm feedback
ln(mdl, 'Induction Motor/1', 'log_i_abc/1');
ln(mdl, 'Induction Motor/2', 'log_wm/1');
ln(mdl, 'Induction Motor/3', 'log_Te/1');
ln(mdl, 'FOC Controller/3', 'log_id/1');
ln(mdl, 'FOC Controller/4', 'log_iq/1');
ln(mdl, 'FOC Controller/5', 'log_iq_ref/1');
ln(mdl, 'FOC Controller/6', 'log_psi_est/1');

configure_foc_dc_link(mdl);
arrange_all(mdl);
add_foc_scopes(mdl);
straighten_foc_wires(mdl);
save_system(mdl, slx);
fprintf('generated %s\n', slx);
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
% FOC controller:  in: w_ref, wm, i_abc   out: v_alpha, v_beta, id, iq,
%                                              iq_ref, psi_est
% ======================================================================
function build_controller(sys)
blk(sys, 'simulink/Sources/In1', 'w_ref', 30, 430, 30, 15);
blk(sys, 'simulink/Sources/In1', 'wm', 30, 490, 30, 15);
blk(sys, 'simulink/Sources/In1', 'i_abc', 30, 60, 30, 15);

blk(sys, 'simulink/Signal Routing/Demux', 'split abc', 110, 40, 8, 70, ...
    'Outputs', '3');

add_block('built-in/Subsystem', [sys '/Clarke abc-ab'], ...
          'Position', [190 35 290 115]);
build_clarke([sys '/Clarke abc-ab']);

add_block('built-in/Subsystem', [sys '/Park ab-dq'], ...
          'Position', [370 35 470 125]);
build_park([sys '/Park ab-dq']);

add_block('built-in/Subsystem', [sys '/Flux estimator & slip'], ...
          'Position', [370 220 500 320]);
build_fluxslip([sys '/Flux estimator & slip']);

% --- outer speed loop -> torque -> iq reference ------------------------
add_block('built-in/Subsystem', [sys '/Speed PI'], ...
          'Position', [150 420 250 500]);
build_pi([sys '/Speed PI'], 'P.Kp_w', 'P.Ki_w', 'P.Te_max', false);

blk(sys, 'simulink/Math Operations/Gain', 'Kt*psi', 560, 320, 55, 30, ...
    'Gain', 'P.Kt');
blk(sys, 'simulink/Math Operations/Divide', 'Te to iq', 660, 425, 45, 40, ...
    'Inputs', '*/');
blk(sys, 'simulink/Discontinuities/Saturation', 'iq limit', 740, 430, 45, 30, ...
    'UpperLimit', 'P.iq_max', 'LowerLimit', '-P.iq_max');
blk(sys, 'simulink/Sources/Constant', 'id ref', 660, 530, 60, 30, ...
    'Value', 'P.id_ref');

% --- decoupling feed-forward terms -------------------------------------
% vd_ff = -we*sigma*Ls*iq ; vq_ff = we*(sigma*Ls*id + (Lm/Lr)*psi)
blk(sys, 'simulink/Math Operations/Product', 'we x iq', 620, 620, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'neg sigmaLs', 690, 622, 70, 30, ...
    'Gain', '-P.sigma*P.Ls');
blk(sys, 'simulink/Math Operations/Gain', 'sigmaLs', 560, 690, 60, 30, ...
    'Gain', 'P.sigma*P.Ls');
blk(sys, 'simulink/Math Operations/Gain', 'Lm over Lr', 560, 740, 60, 30, ...
    'Gain', 'P.Lm/P.Lr');
blk(sys, 'simulink/Math Operations/Sum', 'ff sum', 660, 705, 35, 35, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Product', 'we x ff', 720, 700, 40, 35);

% --- inner current loops ------------------------------------------------
add_block('built-in/Subsystem', [sys '/Id PI'], ...
          'Position', [840 520 940 600]);
build_pi([sys '/Id PI'], 'P.Kp_i', 'P.Ki_i', 'P.v_max', true);
add_block('built-in/Subsystem', [sys '/Iq PI'], ...
          'Position', [840 410 940 490]);
build_pi([sys '/Iq PI'], 'P.Kp_i', 'P.Ki_i', 'P.v_max', true);

add_block('built-in/Subsystem', [sys '/Inv Park dq-ab'], ...
          'Position', [1020 430 1120 520]);
build_invpark([sys '/Inv Park dq-ab']);

blk(sys, 'simulink/Sinks/Out1', 'v_alpha', 1190, 445, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'v_beta', 1190, 495, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'id out', 1190, 55, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'iq out', 1190, 100, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'iq_ref out', 1190, 145, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'psi_est out', 1190, 190, 30, 15);

% --- wiring -------------------------------------------------------------
ln(sys, 'i_abc/1', 'split abc/1');
ln(sys, 'split abc/1', 'Clarke abc-ab/1');
ln(sys, 'split abc/2', 'Clarke abc-ab/2');
ln(sys, 'split abc/3', 'Clarke abc-ab/3');
ln(sys, 'Clarke abc-ab/1', 'Park ab-dq/1');
ln(sys, 'Clarke abc-ab/2', 'Park ab-dq/2');
ln(sys, 'Flux estimator & slip/1', 'Park ab-dq/3');     % theta
ln(sys, 'Park ab-dq/1', 'Flux estimator & slip/1');     % id
ln(sys, 'Park ab-dq/2', 'Flux estimator & slip/2');     % iq
ln(sys, 'wm/1', 'Flux estimator & slip/3');

ln(sys, 'w_ref/1', 'Speed PI/1');
ln(sys, 'wm/1', 'Speed PI/2');
ln(sys, 'Flux estimator & slip/2', 'Kt*psi/1');         % psi_safe
ln(sys, 'Speed PI/1', 'Te to iq/1');
ln(sys, 'Kt*psi/1', 'Te to iq/2');
ln(sys, 'Te to iq/1', 'iq limit/1');

ln(sys, 'Flux estimator & slip/3', 'we x iq/1');        % omega_e
ln(sys, 'Park ab-dq/2', 'we x iq/2');
ln(sys, 'we x iq/1', 'neg sigmaLs/1');
ln(sys, 'Park ab-dq/1', 'sigmaLs/1');
ln(sys, 'Flux estimator & slip/2', 'Lm over Lr/1');
ln(sys, 'sigmaLs/1', 'ff sum/1');
ln(sys, 'Lm over Lr/1', 'ff sum/2');
ln(sys, 'Flux estimator & slip/3', 'we x ff/1');
ln(sys, 'ff sum/1', 'we x ff/2');

ln(sys, 'id ref/1', 'Id PI/1');
ln(sys, 'Park ab-dq/1', 'Id PI/2');
ln(sys, 'neg sigmaLs/1', 'Id PI/3');
ln(sys, 'iq limit/1', 'Iq PI/1');
ln(sys, 'Park ab-dq/2', 'Iq PI/2');
ln(sys, 'we x ff/1', 'Iq PI/3');

ln(sys, 'Id PI/1', 'Inv Park dq-ab/1');
ln(sys, 'Iq PI/1', 'Inv Park dq-ab/2');
ln(sys, 'Flux estimator & slip/1', 'Inv Park dq-ab/3');
ln(sys, 'Inv Park dq-ab/1', 'v_alpha/1');
ln(sys, 'Inv Park dq-ab/2', 'v_beta/1');

ln(sys, 'Park ab-dq/1', 'id out/1');
ln(sys, 'Park ab-dq/2', 'iq out/1');
ln(sys, 'iq limit/1', 'iq_ref out/1');
ln(sys, 'Flux estimator & slip/4', 'psi_est out/1');
end

% ======================================================================
% Clarke:  ialpha = 2/3*ia - 1/3*ib - 1/3*ic ;  ibeta = (ib - ic)/sqrt(3)
% ======================================================================
function build_clarke(sys)
blk(sys, 'simulink/Sources/In1', 'a', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'b', 30, 100, 30, 15);
blk(sys, 'simulink/Sources/In1', 'c', 30, 160, 30, 15);
blk(sys, 'simulink/Math Operations/Gain', 'Ga', 110, 33, 50, 30, 'Gain', '2/3');
blk(sys, 'simulink/Math Operations/Gain', 'Gb', 110, 93, 50, 30, 'Gain', '1/3');
blk(sys, 'simulink/Math Operations/Gain', 'Gc', 110, 153, 50, 30, 'Gain', '1/3');
blk(sys, 'simulink/Math Operations/Sum', 'sum alpha', 200, 85, 35, 60, ...
    'Inputs', '+--', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'sum bc', 200, 200, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'inv sqrt3', 270, 207, 65, 30, ...
    'Gain', '1/sqrt(3)');
blk(sys, 'simulink/Sinks/Out1', 'alpha', 380, 108, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'beta', 380, 214, 30, 15);
ln(sys, 'a/1', 'Ga/1');  ln(sys, 'b/1', 'Gb/1');  ln(sys, 'c/1', 'Gc/1');
ln(sys, 'Ga/1', 'sum alpha/1');
ln(sys, 'Gb/1', 'sum alpha/2');
ln(sys, 'Gc/1', 'sum alpha/3');
ln(sys, 'b/1', 'sum bc/1');
ln(sys, 'c/1', 'sum bc/2');
ln(sys, 'sum alpha/1', 'alpha/1');
ln(sys, 'sum bc/1', 'inv sqrt3/1');
ln(sys, 'inv sqrt3/1', 'beta/1');
end

% ======================================================================
% Park:  d =  alpha*cos + beta*sin ;  q = -alpha*sin + beta*cos
% ======================================================================
function build_park(sys)
blk(sys, 'simulink/Sources/In1', 'alpha', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'beta', 30, 100, 30, 15);
blk(sys, 'simulink/Sources/In1', 'theta', 30, 220, 30, 15);
blk(sys, 'simulink/Math Operations/Trigonometric Function', 'cos', ...
    110, 200, 45, 35, 'Operator', 'cos');
blk(sys, 'simulink/Math Operations/Trigonometric Function', 'sin', ...
    110, 260, 45, 35, 'Operator', 'sin');
blk(sys, 'simulink/Math Operations/Product', 'a cos', 220, 30, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'b sin', 220, 85, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'a sin', 220, 140, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'b cos', 220, 195, 40, 35);
blk(sys, 'simulink/Math Operations/Sum', 'sum d', 310, 55, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'sum q', 310, 155, 35, 45, ...
    'Inputs', '-+', 'IconShape', 'rectangular');
blk(sys, 'simulink/Sinks/Out1', 'd', 400, 70, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'q', 400, 170, 30, 15);
ln(sys, 'theta/1', 'cos/1');  ln(sys, 'theta/1', 'sin/1');
ln(sys, 'alpha/1', 'a cos/1');  ln(sys, 'cos/1', 'a cos/2');
ln(sys, 'beta/1', 'b sin/1');   ln(sys, 'sin/1', 'b sin/2');
ln(sys, 'alpha/1', 'a sin/1');  ln(sys, 'sin/1', 'a sin/2');
ln(sys, 'beta/1', 'b cos/1');   ln(sys, 'cos/1', 'b cos/2');
ln(sys, 'a cos/1', 'sum d/1');  ln(sys, 'b sin/1', 'sum d/2');
ln(sys, 'a sin/1', 'sum q/1');  ln(sys, 'b cos/1', 'sum q/2');
ln(sys, 'sum d/1', 'd/1');      ln(sys, 'sum q/1', 'q/1');
end

% ======================================================================
% Inverse Park:  alpha = d*cos - q*sin ;  beta = d*sin + q*cos
% ======================================================================
function build_invpark(sys)
blk(sys, 'simulink/Sources/In1', 'd', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'q', 30, 100, 30, 15);
blk(sys, 'simulink/Sources/In1', 'theta', 30, 220, 30, 15);
blk(sys, 'simulink/Math Operations/Trigonometric Function', 'cos', ...
    110, 200, 45, 35, 'Operator', 'cos');
blk(sys, 'simulink/Math Operations/Trigonometric Function', 'sin', ...
    110, 260, 45, 35, 'Operator', 'sin');
blk(sys, 'simulink/Math Operations/Product', 'd cos', 220, 30, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'q sin', 220, 85, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'd sin', 220, 140, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'q cos', 220, 195, 40, 35);
blk(sys, 'simulink/Math Operations/Sum', 'sum alpha', 310, 55, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'sum beta', 310, 155, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Sinks/Out1', 'alpha', 400, 70, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'beta', 400, 170, 30, 15);
ln(sys, 'theta/1', 'cos/1');  ln(sys, 'theta/1', 'sin/1');
ln(sys, 'd/1', 'd cos/1');  ln(sys, 'cos/1', 'd cos/2');
ln(sys, 'q/1', 'q sin/1');  ln(sys, 'sin/1', 'q sin/2');
ln(sys, 'd/1', 'd sin/1');  ln(sys, 'sin/1', 'd sin/2');
ln(sys, 'q/1', 'q cos/1');  ln(sys, 'cos/1', 'q cos/2');
ln(sys, 'd cos/1', 'sum alpha/1');  ln(sys, 'q sin/1', 'sum alpha/2');
ln(sys, 'd sin/1', 'sum beta/1');   ln(sys, 'q cos/1', 'sum beta/2');
ln(sys, 'sum alpha/1', 'alpha/1');  ln(sys, 'sum beta/1', 'beta/1');
end

% ======================================================================
% Flux estimator & slip:
%   psi_est  = Lm/(tau_r*s + 1) * id          (current-model estimator)
%   psi_safe = max(psi_est, psi_min)
%   omega_e  = p*wm + Lm*iq/(tau_r*psi_safe)
%   theta    = integral(omega_e)
%   out: 1 theta, 2 psi_safe, 3 omega_e, 4 psi_est
% ======================================================================
function build_fluxslip(sys)
blk(sys, 'simulink/Sources/In1', 'id', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'iq', 30, 140, 30, 15);
blk(sys, 'simulink/Sources/In1', 'wm', 30, 260, 30, 15);
blk(sys, 'simulink/Continuous/Transfer Fcn', 'rotor lag', 110, 25, 90, 45, ...
    'Numerator', '[P.Lm]', 'Denominator', '[P.tau_r 1]');
blk(sys, 'simulink/Sources/Constant', 'psi min', 110, 90, 60, 30, ...
    'Value', 'P.psi_min');
blk(sys, 'simulink/Math Operations/MinMax', 'guard', 240, 45, 55, 40, ...
    'Function', 'max', 'Inputs', '2');
blk(sys, 'simulink/Math Operations/Divide', 'iq over psi', 340, 130, 45, 40, ...
    'Inputs', '*/');
blk(sys, 'simulink/Math Operations/Gain', 'Lm over tau_r', 420, 135, 75, 30, ...
    'Gain', 'P.Lm/P.tau_r');
blk(sys, 'simulink/Math Operations/Gain', 'pole pairs', 340, 255, 50, 30, ...
    'Gain', 'P.p');
blk(sys, 'simulink/Math Operations/Sum', 'we sum', 540, 190, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Continuous/Integrator', 'theta int', 620, 197, 40, 35, ...
    'InitialCondition', '0');
blk(sys, 'simulink/Sinks/Out1', 'theta', 720, 207, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'psi_safe', 720, 58, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'omega_e', 720, 145, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'psi_est', 720, 20, 30, 15);
ln(sys, 'id/1', 'rotor lag/1');
ln(sys, 'rotor lag/1', 'guard/1');
ln(sys, 'psi min/1', 'guard/2');
ln(sys, 'iq/1', 'iq over psi/1');
ln(sys, 'guard/1', 'iq over psi/2');
ln(sys, 'iq over psi/1', 'Lm over tau_r/1');
ln(sys, 'wm/1', 'pole pairs/1');
ln(sys, 'Lm over tau_r/1', 'we sum/1');
ln(sys, 'pole pairs/1', 'we sum/2');
ln(sys, 'we sum/1', 'theta int/1');
ln(sys, 'theta int/1', 'theta/1');
ln(sys, 'guard/1', 'psi_safe/1');
ln(sys, 'we sum/1', 'omega_e/1');
ln(sys, 'rotor lag/1', 'psi_est/1');
end

% ======================================================================
% PI subsystem:  u = sat( Kp*err + int(Ki*err + Kb*(u - u_unsat)) [+ ff] )
%
% Anti-windup is by back-calculation: the saturation excess is fed back
% into the integrator with gain Kb = P.Kb_aw so the integral state cannot
% charge beyond what the actuator can deliver.  Integrator output clamping
% alone bounds only the integral term, leaving a windup overshoot after
% every torque-limited step.
% hasFF adds a feed-forward input summed before the output saturation.
%   in: 1 ref, 2 meas, (3 ff)     out: 1 u
% ======================================================================
function build_pi(sys, kp, ki, lim, hasFF)
blk(sys, 'simulink/Sources/In1', 'ref', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'meas', 30, 100, 30, 15);
blk(sys, 'simulink/Math Operations/Sum', 'err', 110, 55, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'Kp', 200, 30, 55, 30, 'Gain', kp);
blk(sys, 'simulink/Math Operations/Gain', 'Ki', 200, 110, 55, 30, 'Gain', ki);

blk(sys, 'simulink/Math Operations/Sum', 'int in', 290, 105, 35, 60, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Continuous/Integrator', 'int', 360, 118, 40, 35, ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', lim, 'LowerSaturationLimit', ['-' lim]);

if hasFF
    blk(sys, 'simulink/Sources/In1', 'ff', 30, 180, 30, 15);
    blk(sys, 'simulink/Math Operations/Sum', 'pi sum', 450, 60, 35, 90, ...
        'Inputs', '+++', 'IconShape', 'rectangular');
else
    blk(sys, 'simulink/Math Operations/Sum', 'pi sum', 450, 60, 35, 90, ...
        'Inputs', '++', 'IconShape', 'rectangular');
end
blk(sys, 'simulink/Discontinuities/Saturation', 'sat', 540, 85, 45, 35, ...
    'UpperLimit', lim, 'LowerLimit', ['-' lim]);
blk(sys, 'simulink/Sinks/Out1', 'u', 640, 95, 30, 15);

blk(sys, 'simulink/Math Operations/Sum', 'aw diff', 540, 220, 35, 50, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'Kb', 430, 225, 55, 30, ...
    'Gain', 'P.Kb_aw');

ln(sys, 'ref/1', 'err/1');
ln(sys, 'meas/1', 'err/2');
ln(sys, 'err/1', 'Kp/1');
ln(sys, 'err/1', 'Ki/1');
ln(sys, 'Ki/1', 'int in/1');
ln(sys, 'Kb/1', 'int in/2');
ln(sys, 'int in/1', 'int/1');
ln(sys, 'Kp/1', 'pi sum/1');
ln(sys, 'int/1', 'pi sum/2');
if hasFF, ln(sys, 'ff/1', 'pi sum/3'); end
ln(sys, 'pi sum/1', 'sat/1');
ln(sys, 'sat/1', 'u/1');
ln(sys, 'sat/1', 'aw diff/1');
ln(sys, 'pi sum/1', 'aw diff/2');
ln(sys, 'aw diff/1', 'Kb/1');
end

% ======================================================================
% Average-value SVPWM inverter:
%   abc refs from inverse Clarke -> min-max common-mode injection ->
%   pole-voltage saturation at +/-Vdc/2 -> Clarke back to alpha-beta.
%   in: 1 v_alpha_ref, 2 v_beta_ref     out: 1 v_alpha, 2 v_beta
% ======================================================================
function build_inverter(sys)
blk(sys, 'simulink/Sources/In1', 'va_ref', 30, 60, 30, 15);
blk(sys, 'simulink/Sources/In1', 'vb_ref', 30, 200, 30, 15);

% inverse Clarke
blk(sys, 'simulink/Math Operations/Gain', 'neg half', 110, 120, 50, 30, ...
    'Gain', '-0.5');
blk(sys, 'simulink/Math Operations/Gain', 'rt3 half', 110, 193, 50, 30, ...
    'Gain', 'sqrt(3)/2');
blk(sys, 'simulink/Math Operations/Sum', 'b ref', 200, 130, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'c ref', 200, 210, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');

% min-max common-mode injection
blk(sys, 'simulink/Math Operations/MinMax', 'vmax', 300, 30, 55, 50, ...
    'Function', 'max', 'Inputs', '3');
blk(sys, 'simulink/Math Operations/MinMax', 'vmin', 300, 100, 55, 50, ...
    'Function', 'min', 'Inputs', '3');
blk(sys, 'simulink/Math Operations/Sum', 'mm sum', 390, 55, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'neg half cm', 460, 62, 55, 30, ...
    'Gain', '-0.5');

% add common mode, clamp at the DC rails
names = {'a', 'b', 'c'};
for k = 1:3
    y = 140 + 75*(k-1);
    blk(sys, 'simulink/Math Operations/Sum', ['add cm ' names{k}], ...
        560, y, 35, 45, 'Inputs', '++', 'IconShape', 'rectangular');
    blk(sys, 'simulink/Discontinuities/Saturation', ['rail ' names{k}], ...
        640, y + 5, 45, 35, ...
        'UpperLimit', 'P.Vdc/2', 'LowerLimit', '-P.Vdc/2');
end

% Clarke back to alpha-beta (removes the zero sequence)
blk(sys, 'simulink/Math Operations/Gain', 'Ga', 740, 133, 50, 30, 'Gain', '2/3');
blk(sys, 'simulink/Math Operations/Gain', 'Gb', 740, 208, 50, 30, 'Gain', '1/3');
blk(sys, 'simulink/Math Operations/Gain', 'Gc', 740, 283, 50, 30, 'Gain', '1/3');
blk(sys, 'simulink/Math Operations/Sum', 'sum alpha', 830, 185, 35, 60, ...
    'Inputs', '+--', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'sum bc', 830, 290, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'inv sqrt3', 900, 297, 65, 30, ...
    'Gain', '1/sqrt(3)');
blk(sys, 'simulink/Sinks/Out1', 'v_alpha', 1010, 208, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'v_beta', 1010, 304, 30, 15);

% wiring
ln(sys, 'va_ref/1', 'neg half/1');
ln(sys, 'vb_ref/1', 'rt3 half/1');
ln(sys, 'neg half/1', 'b ref/1');
ln(sys, 'rt3 half/1', 'b ref/2');
ln(sys, 'neg half/1', 'c ref/1');
ln(sys, 'rt3 half/1', 'c ref/2');
for pin = 1:3
    src = {'va_ref/1', 'b ref/1', 'c ref/1'};
    ln(sys, src{pin}, sprintf('vmax/%d', pin));
    ln(sys, src{pin}, sprintf('vmin/%d', pin));
end
ln(sys, 'vmax/1', 'mm sum/1');
ln(sys, 'vmin/1', 'mm sum/2');
ln(sys, 'mm sum/1', 'neg half cm/1');
ln(sys, 'va_ref/1', 'add cm a/1');
ln(sys, 'b ref/1', 'add cm b/1');
ln(sys, 'c ref/1', 'add cm c/1');
for k = 1:3
    ln(sys, 'neg half cm/1', sprintf('add cm %s/2', names{k}));
    ln(sys, sprintf('add cm %s/1', names{k}), sprintf('rail %s/1', names{k}));
end
ln(sys, 'rail a/1', 'Ga/1');
ln(sys, 'rail b/1', 'Gb/1');
ln(sys, 'rail c/1', 'Gc/1');
ln(sys, 'Ga/1', 'sum alpha/1');
ln(sys, 'Gb/1', 'sum alpha/2');
ln(sys, 'Gc/1', 'sum alpha/3');
ln(sys, 'rail b/1', 'sum bc/1');
ln(sys, 'rail c/1', 'sum bc/2');
ln(sys, 'sum alpha/1', 'v_alpha/1');
ln(sys, 'sum bc/1', 'inv sqrt3/1');
ln(sys, 'inv sqrt3/1', 'v_beta/1');
end

% ======================================================================
% Induction motor, stationary-frame dq model (see CONTEXT.md sec. 2.1)
%   in: 1 v_alpha, 2 v_beta, 3 TL     out: 1 i_abc, 2 wm, 3 Te
% ======================================================================
function build_motor(sys)
blk(sys, 'simulink/Sources/In1', 'v_alpha', 30, 40, 30, 15);
blk(sys, 'simulink/Sources/In1', 'v_beta', 30, 330, 30, 15);
blk(sys, 'simulink/Sources/In1', 'TL', 30, 640, 30, 15);

% --- stator current alpha ----------------------------------------------
blk(sys, 'simulink/Math Operations/Gain', 'Rsig a', 350, 95, 55, 30, ...
    'Gain', 'P.Rsig');
blk(sys, 'simulink/Math Operations/Gain', 'psi a coup', 350, 145, 90, 30, ...
    'Gain', 'P.Lm/(P.Lr*P.tau_r)');
blk(sys, 'simulink/Math Operations/Product', 'wr x psib', 280, 195, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'kr a', 350, 197, 55, 30, ...
    'Gain', 'P.Lm/P.Lr');
blk(sys, 'simulink/Math Operations/Sum', 'sum dia', 460, 30, 40, 90, ...
    'Inputs', '+-++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'inv sigmaLs a', 540, 55, 85, 30, ...
    'Gain', '1/(P.sigma*P.Ls)');
blk(sys, 'simulink/Continuous/Integrator', 'int ia', 670, 52, 40, 35, ...
    'InitialCondition', '0');

% --- stator current beta ------------------------------------------------
blk(sys, 'simulink/Math Operations/Gain', 'Rsig b', 350, 385, 55, 30, ...
    'Gain', 'P.Rsig');
blk(sys, 'simulink/Math Operations/Gain', 'psi b coup', 350, 435, 90, 30, ...
    'Gain', 'P.Lm/(P.Lr*P.tau_r)');
blk(sys, 'simulink/Math Operations/Product', 'wr x psia', 280, 485, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'kr b', 350, 487, 55, 30, ...
    'Gain', 'P.Lm/P.Lr');
blk(sys, 'simulink/Math Operations/Sum', 'sum dib', 460, 320, 40, 90, ...
    'Inputs', '+-+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'inv sigmaLs b', 540, 345, 85, 30, ...
    'Gain', '1/(P.sigma*P.Ls)');
blk(sys, 'simulink/Continuous/Integrator', 'int ib', 670, 342, 40, 35, ...
    'InitialCondition', '0');

% --- rotor flux alpha ---------------------------------------------------
blk(sys, 'simulink/Math Operations/Gain', 'inv tau a', 350, 570, 60, 30, ...
    'Gain', '1/P.tau_r');
blk(sys, 'simulink/Math Operations/Product', 'wr x psib 2', 280, 620, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'Lm tau a', 350, 620, 60, 30, ...
    'Gain', 'P.Lm/P.tau_r');
blk(sys, 'simulink/Math Operations/Sum', 'sum dpa', 460, 555, 40, 80, ...
    'Inputs', '--+', 'IconShape', 'rectangular');
blk(sys, 'simulink/Continuous/Integrator', 'int psia', 540, 575, 40, 35, ...
    'InitialCondition', '0');

% --- rotor flux beta ----------------------------------------------------
blk(sys, 'simulink/Math Operations/Gain', 'inv tau b', 350, 720, 60, 30, ...
    'Gain', '1/P.tau_r');
blk(sys, 'simulink/Math Operations/Product', 'wr x psia 2', 280, 770, 40, 35);
blk(sys, 'simulink/Math Operations/Gain', 'Lm tau b', 350, 770, 60, 30, ...
    'Gain', 'P.Lm/P.tau_r');
blk(sys, 'simulink/Math Operations/Sum', 'sum dpb', 460, 705, 40, 80, ...
    'Inputs', '-++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Continuous/Integrator', 'int psib', 540, 725, 40, 35, ...
    'InitialCondition', '0');

% --- torque and mechanics ----------------------------------------------
blk(sys, 'simulink/Math Operations/Product', 'psia x ib', 700, 870, 40, 35);
blk(sys, 'simulink/Math Operations/Product', 'psib x ia', 700, 925, 40, 35);
blk(sys, 'simulink/Math Operations/Sum', 'sum te', 780, 885, 35, 50, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'kt', 850, 892, 90, 35, ...
    'Gain', '1.5*P.p*P.Lm/P.Lr');
blk(sys, 'simulink/Math Operations/Gain', 'friction', 850, 960, 55, 30, ...
    'Gain', 'P.B');
blk(sys, 'simulink/Math Operations/Sum', 'sum mech', 990, 880, 40, 80, ...
    'Inputs', '+--', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Gain', 'inv J', 1070, 905, 55, 30, ...
    'Gain', '1/P.J');
blk(sys, 'simulink/Continuous/Integrator', 'int wm', 1160, 902, 40, 35, ...
    'InitialCondition', '0');
blk(sys, 'simulink/Math Operations/Gain', 'pole pairs', 1160, 990, 50, 30, ...
    'Gain', 'P.p');

% --- inverse Clarke for phase currents ---------------------------------
blk(sys, 'simulink/Math Operations/Gain', 'neg half', 780, 120, 50, 30, ...
    'Gain', '-0.5');
blk(sys, 'simulink/Math Operations/Gain', 'rt3 half', 780, 400, 50, 30, ...
    'Gain', 'sqrt(3)/2');
blk(sys, 'simulink/Math Operations/Sum', 'ib phase', 880, 180, 35, 45, ...
    'Inputs', '++', 'IconShape', 'rectangular');
blk(sys, 'simulink/Math Operations/Sum', 'ic phase', 880, 260, 35, 45, ...
    'Inputs', '+-', 'IconShape', 'rectangular');
blk(sys, 'simulink/Signal Routing/Mux', 'mux abc', 970, 100, 8, 120, ...
    'Inputs', '3');

blk(sys, 'simulink/Sinks/Out1', 'i_abc', 1050, 152, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'wm', 1250, 912, 30, 15);
blk(sys, 'simulink/Sinks/Out1', 'Te', 1000, 830, 30, 15);

% --- wiring -------------------------------------------------------------
ln(sys, 'v_alpha/1', 'sum dia/1');
ln(sys, 'int ia/1', 'Rsig a/1');
ln(sys, 'Rsig a/1', 'sum dia/2');
ln(sys, 'int psia/1', 'psi a coup/1');
ln(sys, 'psi a coup/1', 'sum dia/3');
ln(sys, 'pole pairs/1', 'wr x psib/1');
ln(sys, 'int psib/1', 'wr x psib/2');
ln(sys, 'wr x psib/1', 'kr a/1');
ln(sys, 'kr a/1', 'sum dia/4');
ln(sys, 'sum dia/1', 'inv sigmaLs a/1');
ln(sys, 'inv sigmaLs a/1', 'int ia/1');

ln(sys, 'v_beta/1', 'sum dib/1');
ln(sys, 'int ib/1', 'Rsig b/1');
ln(sys, 'Rsig b/1', 'sum dib/2');
ln(sys, 'int psib/1', 'psi b coup/1');
ln(sys, 'psi b coup/1', 'sum dib/3');
ln(sys, 'pole pairs/1', 'wr x psia/1');
ln(sys, 'int psia/1', 'wr x psia/2');
ln(sys, 'wr x psia/1', 'kr b/1');
ln(sys, 'kr b/1', 'sum dib/4');
ln(sys, 'sum dib/1', 'inv sigmaLs b/1');
ln(sys, 'inv sigmaLs b/1', 'int ib/1');

ln(sys, 'int psia/1', 'inv tau a/1');
ln(sys, 'inv tau a/1', 'sum dpa/1');
ln(sys, 'pole pairs/1', 'wr x psib 2/1');
ln(sys, 'int psib/1', 'wr x psib 2/2');
ln(sys, 'wr x psib 2/1', 'sum dpa/2');
ln(sys, 'int ia/1', 'Lm tau a/1');
ln(sys, 'Lm tau a/1', 'sum dpa/3');
ln(sys, 'sum dpa/1', 'int psia/1');

ln(sys, 'int psib/1', 'inv tau b/1');
ln(sys, 'inv tau b/1', 'sum dpb/1');
ln(sys, 'pole pairs/1', 'wr x psia 2/1');
ln(sys, 'int psia/1', 'wr x psia 2/2');
ln(sys, 'wr x psia 2/1', 'sum dpb/2');
ln(sys, 'int ib/1', 'Lm tau b/1');
ln(sys, 'Lm tau b/1', 'sum dpb/3');
ln(sys, 'sum dpb/1', 'int psib/1');

ln(sys, 'int psia/1', 'psia x ib/1');
ln(sys, 'int ib/1', 'psia x ib/2');
ln(sys, 'int psib/1', 'psib x ia/1');
ln(sys, 'int ia/1', 'psib x ia/2');
ln(sys, 'psia x ib/1', 'sum te/1');
ln(sys, 'psib x ia/1', 'sum te/2');
ln(sys, 'sum te/1', 'kt/1');
ln(sys, 'kt/1', 'sum mech/1');
ln(sys, 'TL/1', 'sum mech/2');
ln(sys, 'int wm/1', 'friction/1');
ln(sys, 'friction/1', 'sum mech/3');
ln(sys, 'sum mech/1', 'inv J/1');
ln(sys, 'inv J/1', 'int wm/1');
ln(sys, 'int wm/1', 'pole pairs/1');

ln(sys, 'int ia/1', 'neg half/1');
ln(sys, 'int ib/1', 'rt3 half/1');
ln(sys, 'neg half/1', 'ib phase/1');
ln(sys, 'rt3 half/1', 'ib phase/2');
ln(sys, 'neg half/1', 'ic phase/1');
ln(sys, 'rt3 half/1', 'ic phase/2');
ln(sys, 'int ia/1', 'mux abc/1');
ln(sys, 'ib phase/1', 'mux abc/2');
ln(sys, 'ic phase/1', 'mux abc/3');
ln(sys, 'mux abc/1', 'i_abc/1');
ln(sys, 'int wm/1', 'wm/1');
ln(sys, 'kt/1', 'Te/1');
end

% ======================================================================
% Speed Reference generator (fundamental blocks only).
%
% A staircase of constant speed levels: four Step blocks, each carrying the
% increment between consecutive levels, summed together ->
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
% small helpers
% ======================================================================
function blk(sys, lib, name, x, y, w, h, varargin)
add_block(lib, [sys '/' name], 'Position', [x y x+w y+h], varargin{:});
end

function ln(sys, from, to)
add_line(sys, from, to, 'autorouting', 'smart');
end
