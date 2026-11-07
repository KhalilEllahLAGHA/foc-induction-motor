function configure_foc_dc_link(mdl)
%CONFIGURE_FOC_DC_LINK Add an R-L source and DC capacitor using basic blocks.
% The bridge remains averaged. Duty d couples the motor and the DC link:
% i_bridge = d'*i_abc and v_abc (relative to the negative rail) = v_dc*d.
sys = [mdl '/SVPWM Inverter (avg)'];
if getSimulinkBlockHandle([sys '/RLC DC link']) ~= -1
    return
end

add(sys, 'Sources/In1', 'i_abc', [60 410 90 425], 'Port', '3');
add(sys, 'Signal Routing/Mux', 'pole commands', [725 355 730 410], 'Inputs', '3');
add(sys, 'Math Operations/Gain', 'per unit', [775 365 825 400], 'Gain', '1/P.Vdc');
add(sys, 'Math Operations/Bias', 'duty abc', [865 365 925 400], 'Bias', '0.5');
link = [sys '/RLC DC link'];
add_block('built-in/Subsystem', link, 'Position', [965 355 1080 435]);
build_link(link);
add(sys, 'Math Operations/Gain', 'DC ratio', [1120 365 1190 400], 'Gain', '1/P.Vdc');
add(sys, 'Math Operations/Product', 'actual alpha', [1240 110 1280 145]);
add(sys, 'Math Operations/Product', 'actual beta', [1240 270 1280 305]);
set_param([sys '/v_alpha'], 'Position', [1330 120 1360 135]);
set_param([sys '/v_beta'], 'Position', [1330 280 1360 295]);
add(sys, 'Sinks/Out1', 'v_dc', [1330 417 1360 432], 'Port', '3');

delete_line(sys, 'sum alpha/1', 'v_alpha/1');
delete_line(sys, 'inv sqrt3/1', 'v_beta/1');
wire(sys, 'rail a/1', 'pole commands/1');
wire(sys, 'rail b/1', 'pole commands/2');
wire(sys, 'rail c/1', 'pole commands/3');
wire(sys, 'pole commands/1', 'per unit/1');
wire(sys, 'per unit/1', 'duty abc/1');
wire(sys, 'duty abc/1', 'RLC DC link/1');
wire(sys, 'i_abc/1', 'RLC DC link/2');
wire(sys, 'RLC DC link/1', 'DC ratio/1');
wire(sys, 'RLC DC link/1', 'v_dc/1');
wire(sys, 'sum alpha/1', 'actual alpha/1');
wire(sys, 'inv sqrt3/1', 'actual beta/1');
wire(sys, 'DC ratio/1', 'actual alpha/2');
wire(sys, 'DC ratio/1', 'actual beta/2');
wire(sys, 'actual alpha/1', 'v_alpha/1');
wire(sys, 'actual beta/1', 'v_beta/1');

wire(mdl, 'Induction Motor/1', 'SVPWM Inverter (avg)/3');
position = get_param([mdl '/log_psi_est'], 'Position');
position([2 4]) = position([2 4]) + 55;
add(mdl, 'Sinks/To Workspace', 'log_v_dc', position, ...
    'VariableName', 'v_dc', 'SaveFormat', 'Timeseries');
wire(mdl, 'SVPWM Inverter (avg)/3', 'log_v_dc/1');
end

function build_link(sys)
add(sys, 'Sources/In1', 'duty abc', [30 230 60 245], 'Port', '1');
add(sys, 'Sources/In1', 'i abc', [30 295 60 310], 'Port', '2');
add(sys, 'Sources/Constant', 'source voltage', [30 30 90 60], 'Value', 'P.Vdc');
add(sys, 'Math Operations/Sum', 'inductor voltage', [170 26 195 94], 'Inputs', '+--');
add(sys, 'Math Operations/Gain', 'inverse L', [240 45 300 75], 'Gain', '1/P.Ldc');
add(sys, 'Continuous/Integrator', 'source current', [345 44 395 76], ...
    'InitialCondition', '0');
add(sys, 'Math Operations/Gain', 'series R', [340 112 400 142], 'Gain', 'P.Rdc');
add(sys, 'Math Operations/Dot Product', 'bridge current', [145 247 195 293]);
add(sys, 'Math Operations/Sum', 'capacitor current', [470 229 495 289], 'Inputs', '+-');
add(sys, 'Math Operations/Gain', 'inverse C', [535 242 595 272], 'Gain', '1/P.Cdc');
add(sys, 'Continuous/Integrator', 'DC voltage', [640 241 690 273], ...
    'InitialCondition', 'P.Vdc');
add(sys, 'Sinks/Out1', 'v dc', [755 250 785 265]);

wire(sys, 'source voltage/1', 'inductor voltage/1');
wire(sys, 'series R/1', 'inductor voltage/2');
wire(sys, 'DC voltage/1', 'inductor voltage/3');
wire(sys, 'inductor voltage/1', 'inverse L/1');
wire(sys, 'inverse L/1', 'source current/1');
wire(sys, 'source current/1', 'series R/1');
wire(sys, 'source current/1', 'capacitor current/1');
wire(sys, 'duty abc/1', 'bridge current/1');
wire(sys, 'i abc/1', 'bridge current/2');
wire(sys, 'bridge current/1', 'capacitor current/2');
wire(sys, 'capacitor current/1', 'inverse C/1');
wire(sys, 'inverse C/1', 'DC voltage/1');
wire(sys, 'DC voltage/1', 'v dc/1');
end

function add(sys, lib, name, position, varargin)
add_block(['simulink/' lib], [sys '/' name], 'Position', position, varargin{:});
end

function wire(sys, source, destination)
add_line(sys, source, destination, 'autorouting', 'on');
end
