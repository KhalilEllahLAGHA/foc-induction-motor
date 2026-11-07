function metrics = compare_foc_pwm()
%COMPARE_FOC_PWM Compare the three graphical Simscape PWM variants.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'm_files'));
P = motor_params();
models = {'foc_im_spwm', 'foc_im_svpwm', 'foc_im_svpwm_4seg'};
modes = ["SPWM"; "Seven-segment SVPWM"; "Four-segment SVPWM"];
assert(~any(cellfun(@bdIsLoaded, models)), 'Close the graphical PWM models before running the comparison.');

start = 1.105;
periods = 6;
samplesPerPeriod = round(1/(P.fsw*P.Ts_power));
assert(abs(samplesPerPeriod*P.Ts_power - 1/P.fsw) < 1e-12, ...
    'The electrical step must divide the PWM period.');
samples = periods*samplesPerPeriod;
outDir = fullfile(here, '..', 'outputs', 'graphs');
if ~isfolder(outDir), mkdir(outDir); end
waveforms = table((0:samples)'*P.Ts_power*1e6, 'VariableNames', {'time_us'});
prefixes = ["spwm", "svpwm7", "svpwm4"];
torque = zeros(samples+1, numel(models));
firstMean = zeros(numel(models), 3);

for k = 1:numel(models)
    fprintf('Running %s\n', modes(k));
    [torque(:,k), voltage, state] = run_window(here, models{k}, P, start, samples);
    firstMean(k,:) = mean(voltage(1:samplesPerPeriod,:), 1);
    names = prefixes(k) + ["_torque_nm", "_va_v", "_vb_v", "_vc_v", "_state"];
    waveforms = [waveforms array2table([torque(:,k), voltage, state], 'VariableNames', names)]; %#ok<AGROW>
    edges = [1; find(diff(state(1:samplesPerPeriod)) ~= 0) + 1; samplesPerPeriod+1];
    fprintf('  First-period states %s, dwell times %s us\n', ...
        strjoin(string(dec2bin(state(edges(1:end-1)),3)), ' '), ...
        num2str(diff(edges)'*P.Ts_power*1e6));
end

window = torque(1:samples,:);
metrics = table(modes, mean(window,1)', min(window,[],1)', max(window,[],1)', ...
    (max(window,[],1)-min(window,[],1))', firstMean(:,1), firstMean(:,2), firstMean(:,3), ...
    'VariableNames', {'mode','mean_torque_nm','min_torque_nm','max_torque_nm','peak_to_peak_nm', ...
    'first_period_va_v','first_period_vb_v','first_period_vc_v'});
disp(metrics);
writetable(metrics, fullfile(outDir, 'library_pwm_metrics.csv'));
writetable(waveforms, fullfile(outDir, 'library_pwm_waveforms.csv'));
plot_torque(waveforms.time_us, torque, modes, start, periods, P, outDir);
plot_voltages(waveforms, modes(2), prefixes(2), start, periods, P, outDir);
plot_voltages(waveforms, modes(3), prefixes(3), start, periods, P, outDir);
end

function [torque, phaseVoltage, state] = run_window(here, model, P, start, samples)
file = fullfile(here, [model '.slx']);
assert(isfile(file), 'Missing %s. Run build_foc_pwm_variants first.', file);
load_system(file);
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
add_voltage_probes(model);
ports = get_param([model '/PWM Generator'], 'PortHandles');
set_param(ports.Outport, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'gates');
input = Simulink.SimulationInput(model);
input = input.setVariable('P', P);
input = input.setModelParameter('StopTime', sprintf('%.12g', start + samples*P.Ts_power), ...
    'SignalLogging', 'on');
out = sim(input);
ticks = round(start/P.Ts_power) + (0:samples)';
torque = read_signal(out.Te, ticks, P.Ts_power);
lineNames = {'v_ab', 'v_bc', 'v_ca'};
lineVoltage = zeros(samples+1, 3);
for k = 1:3
    lineVoltage(:,k) = read_signal(out.(lineNames{k}), ticks, P.Ts_power);
end
assert(max(abs(sum(lineVoltage,2))) < 1e-6, 'Measured line voltages must sum to zero.');
phaseVoltage = (lineVoltage - lineVoltage(:,[3 1 2]))/3;
gates = read_signal(out.logsout.getElement('gates').Values, ticks, P.Ts_power);
assert(all(gates(:,1:2:5)+gates(:,2:2:6) == 1, 'all'), 'Expected complementary gate pairs.');
state = gates(:,1:2:5) * [4;2;1];
end

function add_voltage_probes(mdl)
% Measure the three line voltages at the motor terminals for this run.
phases = get_param([mdl '/IM 5kW'], 'PortHandles');
labels = {'AB', 'BC', 'CA'};
for k = 1:3
    label = labels{k};
    probe = add_block('sps_lib/Sensors and Measurements/Voltage Measurement', ...
        [mdl '/V' label ' probe']);
    ports = get_param(probe, 'PortHandles');
    add_line(mdl, phases.LConn(k), ports.LConn(1));
    add_line(mdl, phases.LConn(mod(k,3)+1), ports.LConn(2));
    log = add_block('simulink/Sinks/To Workspace', [mdl '/Log V' label], ...
        'VariableName', ['v_' lower(label)], 'SaveFormat', 'Timeseries');
    logPorts = get_param(log, 'PortHandles');
    add_line(mdl, ports.Outport, logPorts.Inport);
end
end

function values = read_signal(ts, ticks, step)
time = ts.Time;
idx = find(time >= (ticks(1)-0.25)*step & time <= (ticks(end)+0.25)*step);
assert(numel(idx) == numel(ticks) && all(abs(time(idx)-ticks*step) < step/4), ...
    'Unexpected electrical simulation grid.');
values = squeeze(ts.Data);
if size(values,1) ~= numel(time), values = values.'; end
values = values(idx,:);
assert(all(isfinite(values), 'all'));
end

function plot_torque(t, torque, modes, start, periods, P, outDir)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 990 455]);
ax = axes(f);
hold(ax, 'on');
colors = [0.09 0.38 0.66; 0.73 0.27 0.09; 0.03 0.47 0.34];
styles = {'-', '--', '-.'};
for k = 1:numel(modes)
    plot(ax, t, torque(:,k), styles{k}, 'Color', colors(k,:), ...
        'DisplayName', modes(k), 'LineWidth', 1.7);
end
style_axes(ax, periods, P);
legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal');
title(ax, 'Electromagnetic torque over six PWM periods');
xlabel(ax, time_label(start));
ylabel(ax, 'Torque (N m)');
save_plot(f, outDir, 'library_pwm_comparison.png');
end

function plot_voltages(data, mode, prefix, start, periods, P, outDir)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 990 655]);
layout = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, "Motor phase voltages | SPWM and " + mode, 'FontWeight', 'bold', 'FontSize', 14);
xlabel(layout, time_label(start));
phases = ["A","B","C"];
for k = 1:3
    ax = nexttile(layout);
    hold(ax, 'on');
    suffix = "_v" + lower(phases(k)) + "_v";
    stairs(ax, data.time_us, data.("spwm" + suffix), ...
        'Color', [0.09 0.38 0.66], 'LineWidth', 1.7, 'DisplayName', 'SPWM');
    stairs(ax, data.time_us, data.(prefix + suffix), '--', ...
        'Color', [0.73 0.27 0.09], 'LineWidth', 1.35, 'DisplayName', mode);
    style_axes(ax, periods, P);
    ylim(ax, 0.75*P.Vdc*[-1 1]);
    yticks(ax, (-2:2)*P.Vdc/3);
    ylabel(ax, 'Voltage (V)');
    title(ax, "Phase " + phases(k));
    if k == 1, legend(ax, 'Location', 'southoutside', 'Orientation', 'horizontal'); end
    if k < 3, ax.XTickLabel = {}; end
end
save_plot(f, outDir, "library_pwm_voltage_" + prefix + ".png");
end

function style_axes(ax, periods, P)
ticks = (0:periods)*1e6/P.fsw;
grid(ax, 'on');
ax.GridAlpha = 0.18;
ax.FontSize = 11;
xlim(ax, [ticks(1) ticks(end)]);
xticks(ax, ticks);
end

function label = time_label(start)
label = sprintf('Time since %.4f s (\\mus)', start);
end

function save_plot(figureHandle, folder, file)
exportgraphics(figureHandle, fullfile(folder,file), 'Resolution', 160);
close(figureHandle);
end
