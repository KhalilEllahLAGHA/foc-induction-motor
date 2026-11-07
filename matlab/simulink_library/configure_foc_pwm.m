function configure_foc_pwm(mdl)
%CONFIGURE_FOC_PWM Install the custom modulator in a loaded library model.
sys = [mdl '/PWM Generator'];
old = getSimulinkBlockHandle(sys);
position = [470 400 560 460];
if old ~= -1
    position = get_param(old, 'Position');
    delete_line(mdl, 'FOC Controller/1', 'PWM Generator/1');
    delete_line(mdl, 'PWM Generator/1', 'Inverter/1');
    delete_block(old);
end

add_block('built-in/Subsystem', sys, 'Position', position);
add_block('simulink/Sources/In1', [sys '/Uref'], ...
    'Position', [30 55 60 70], 'PortDimensions', '3');
add_block('simulink/Discrete/Zero-Order Hold', [sys '/Hold per PWM period'], ...
    'Position', [110 45 225 80], 'SampleTime', '1/P.fsw');
add_block('simulink/Sources/Digital Clock', [sys '/Power step clock'], ...
    'Position', [120 105 215 135], 'SampleTime', 'P.Ts_power');
add_block('simulink/Sources/Constant', [sys '/Switching frequency'], ...
    'Position', [120 155 215 185], 'Value', 'P.fsw');
add_block('simulink/Sources/Constant', [sys '/Power step'], ...
    'Position', [120 205 215 235], 'Value', 'P.Ts_power');
add_block('simulink/Sources/Constant', [sys '/Enable SVPWM'], ...
    'Position', [120 255 215 285], 'Value', 'P.use_svpwm');
functionBlock = [sys '/Custom gate timing'];
add_block('simulink/User-Defined Functions/MATLAB Function', functionBlock, ...
    'Position', [300 45 500 285]);
chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', functionBlock);
chart.Script = fileread(fullfile(fileparts(mfilename('fullpath')), 'foc_pwm_gates.m'));
config = get_param(functionBlock, 'MATLABFunctionConfiguration');
config.UpdateMethod = 'Discrete';
config.SampleTime = 'P.Ts_power';
add_block('simulink/Sinks/Out1', [sys '/g'], ...
    'Position', [565 159 595 174]);

add_line(sys, 'Uref/1', 'Hold per PWM period/1');
sources = {'Hold per PWM period', 'Power step clock', 'Switching frequency', ...
           'Power step', 'Enable SVPWM'};
for k = 1:numel(sources)
    add_line(sys, [sources{k} '/1'], sprintf('Custom gate timing/%d', k), 'autorouting', 'on');
end
add_line(sys, 'Custom gate timing/1', 'g/1', 'autorouting', 'on');
add_line(mdl, 'FOC Controller/1', 'PWM Generator/1', 'autorouting', 'on');
add_line(mdl, 'PWM Generator/1', 'Inverter/1', 'autorouting', 'on');
end
