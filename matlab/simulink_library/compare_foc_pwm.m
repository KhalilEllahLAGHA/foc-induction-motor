function metrics = compare_foc_pwm()
%COMPARE_FOC_PWM Run the same Simscape IGBT drive with custom SPWM and SVPWM.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'm_files'));
P = motor_params();
mdl = 'foc_im_library';
if bdIsLoaded(mdl)
    error('Close %s before running the comparison to preserve any open edits.', mdl);
end
load_system(fullfile(here, [mdl '.slx']));
closeModel = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

% Observe gate commands without adding a logging block to the saved model.
ports = get_param([mdl '/PWM Generator'], 'PortHandles');
set_param(ports.Outport, 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'pwm_gates');

names = ["SPWM"; "SVPWM"];
zoomStart = 1.105;
for k = 1:numel(names)
    P.use_svpwm = k == 2;
    input = Simulink.SimulationInput(mdl);
    input = input.setVariable('P', P);
    input = input.setModelParameter('SignalLogging', 'on');
    fprintf('\n%s, %.0f Hz PWM, %.1f us power step\n', names(k), P.fsw, P.Ts_power*1e6);
    out = sim(input);
    result(k) = summarize_run(out, P, zoomStart); %#ok<AGROW>
    clear out
end

metrics = vertcat(result.metrics);
metrics = addvars(metrics, names, 'Before', 1, 'NewVariableNames', 'mode');
disp(metrics);
outDir = fullfile(here, '..', 'outputs', 'graphs');
if ~isfolder(outDir), mkdir(outDir); end
writetable(metrics, fullfile(outDir, 'library_pwm_metrics.csv'));
plot_comparison(result, P, zoomStart, outDir);
end

function result = summarize_run(out, P, start)
m = report_metrics(out, P);
[t, current] = signal(out.i_abc);
[tg, gates] = signal(out.logsout.get('pwm_gates').Values);
assert(all(isfinite(current(:))) && all(isfinite(out.wm.Data)), 'Nonfinite drive output');
assert(all(gates(:, 1:2:5) + gates(:, 2:2:6) == 1, 'all'), 'IGBT gate order mismatch');

% Analyse 200 ms of the 1400 rpm plateau on the same power-step grid.
samplesPerPeriod = round(1/(P.fsw*P.Ts_power));
plateau = grid_window(t, 1.05, 0.20, P.Ts_power);
currentsByPeriod = reshape(current(plateau, :), samplesPerPeriod, []);
% Remove a linear trend separately from every phase and PWM period.
residual = detrend(currentsByPeriod, 1);
rippleRms = sqrt(mean(residual(:).^2));
ripplePp = mean(max(residual, [], 1) - min(residual, [], 1));
result.metrics = table(m.settling_ms(1), m.settling_ms(2), m.settling_ms(3), ...
    m.settling_ms(4), m.plateau_max_rpm, m.plateau_rms_rpm, m.load_dip_rpm, ...
    m.final_flux_wb, max(abs(current(:))), rippleRms, ripplePp, ...
    'VariableNames', {'settle_800_ms', 'settle_1400_ms', 'settle_600_ms', ...
    'settle_reverse_ms', 'plateau_max_rpm', 'plateau_rms_rpm', 'load_dip_rpm', ...
    'final_flux_wb', 'peak_current_a', 'ripple_rms_a', 'mean_ripple_pp_a'});

[tw, w] = signal(out.wm);
[tt, torque] = signal(out.Te);
stride = round(P.Ts_ctrl/P.Ts_power);
result.speed = [tw(1:stride:end), w(1:stride:end)*60/(2*pi)];
result.torque = [tt(1:stride:end), torque(1:stride:end)];
[tr, wr] = signal(out.w_ref);
result.reference = [tr(1:stride:end), wr(1:stride:end)*60/(2*pi)];

duration = 6/P.fsw;
win = grid_window(t, start, duration, P.Ts_power);
gateWin = grid_window(tg, start, duration, P.Ts_power);
periods = round(duration*P.fsw);
zoomRipple = reshape(residual, samplesPerPeriod, [], 3);
firstPeriod = round((start - 1.05)*P.fsw) + 1;
phaseARipple = zoomRipple(:, firstPeriod:firstPeriod+periods-1, 1);
result.zoom = [(t(win) - start)*1e6, current(win, 1), phaseARipple(:)];
result.vab = [(tg(gateWin) - start)*1e6, ...
    P.Vdc*(gates(gateWin, 1) - gates(gateWin, 3))];
end

function idx = grid_window(time, start, duration, step)
idx = find(time >= start - step/4 & time < start + duration - step/4);
assert(numel(idx) == round(duration/step), 'Unexpected power-step sample grid');
end

function [t, y] = signal(ts)
t = ts.Time(:);
y = squeeze(ts.Data);
if size(y, 1) ~= numel(t), y = y.'; end
end

function plot_comparison(result, P, zoomStart, outDir)
colors = [0.13 0.40 0.70; 0.79 0.29 0.10];
labels = {'SPWM', 'SVPWM'};
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 560]);
layout = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout);
hold(ax, 'on');
plot(ax, result(1).reference(:, 1), result(1).reference(:, 2), ':k', ...
    'DisplayName', 'Reference', 'LineWidth', 1.1);
for k = 1:2
    plot(ax, result(k).speed(:, 1), result(k).speed(:, 2), ...
        'Color', colors(k, :), 'DisplayName', labels{k}, 'LineWidth', 1.2);
end
ylabel(ax, 'Speed (rpm)'); grid(ax, 'on'); legend(ax, 'Location', 'southwest');
title(ax, 'Same Simscape drive and speed reference');
ax = nexttile(layout); hold(ax, 'on');
for k = 1:2
    plot(ax, result(k).torque(:, 1), result(k).torque(:, 2), ...
        'Color', colors(k, :), 'DisplayName', labels{k}, 'LineWidth', 1.0);
end
ylabel(ax, 'Torque (N m)'); xlabel(ax, 'Time (s)'); grid(ax, 'on');
exportgraphics(f, fullfile(outDir, 'library_pwm_comparison.png'), 'Resolution', 170);
close(f);

f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 930 710]);
layout = tiledlayout(f, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
zoomLimits = [0 6e6/P.fsw];
lineVoltage = [result(1).vab(:, 2); result(2).vab(:, 2)];
voltageLimits = [min(lineVoltage) max(lineVoltage)] + 0.12*P.Vdc*[-1 1];
for k = 1:2
    ax = nexttile(layout);
    stairs(ax, result(k).vab(:, 1), result(k).vab(:, 2), 'Color', colors(k, :));
    xlim(ax, zoomLimits); ylim(ax, voltageLimits);
    ylabel(ax, 'v_{AB} (V)'); grid(ax, 'on');
    title(ax, [labels{k} ' ideal line voltage from IGBT gate states']);
end
names = {'i_A (A)', 'Ripple (A)'};
titles = {'Motor phase A current', 'Phase A ripple after removing each period''s linear trend'};
for n = 1:2
    ax = nexttile(layout); hold(ax, 'on');
    for k = 1:2
        plot(ax, result(k).zoom(:, 1), result(k).zoom(:, n+1), ...
            'Color', colors(k, :), 'DisplayName', labels{k}, 'LineWidth', 1.15);
    end
    xlim(ax, zoomLimits); ylabel(ax, names{n}); title(ax, titles{n}); grid(ax, 'on');
    legend(ax, 'Location', 'best');
end
xlabel(ax, sprintf('Time since %.3f s (\\mus)', zoomStart));
title(layout, '1400 rpm plateau, six 10 kHz switching periods');
exportgraphics(f, fullfile(outDir, 'library_pwm_zoom.png'), 'Resolution', 170);
close(f);
end
