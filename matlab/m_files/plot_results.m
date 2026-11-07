function paths = plot_results(out, variant, P)
%PLOT_RESULTS  Post-process a FOC simulation run and save all figures.
%
%   paths = PLOT_RESULTS(out, variant)
%
%   out     - Simulink.SimulationOutput from sim('foc_im_blocks') or
%             sim('foc_im_library').  Both models log the same To Workspace
%             variables: wm, w_ref, Te, TL, id, iq, iq_ref, psi_est, i_abc
%             (all timeseries).
%   variant - 'blocks' or 'library'; used as a filename prefix.
%   P       - parameters used for the run (defaults to the base workspace).
%
%   Figures (speed, torque, dq currents, rotor flux, phase currents) are
%   saved as PNG into  matlab/outputs/graphs/.

if nargin < 2, variant = 'sim'; end

% Base workspace P (for references/limits shown on the plots).
if nargin < 3, P = evalin('base', 'P'); end

out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                   'outputs', 'graphs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

% ---- palette -------------------------------------------------------------
c.blue    = [ 42 120 214]/255;
c.orange  = [235 104  52]/255;
c.aqua    = [ 27 175 122]/255;
c.ink2    = [ 82  81  78]/255;   % references / limits (dashed)
c.surface = [252 252 251]/255;
rpm = 60/(2*pi);

paths = {};

% ---- 1. speed ------------------------------------------------------------
f = new_fig(c);
[t, y] = sig(out, 'w_ref');  plot(t, y*rpm, '--', 'Color', c.ink2, ...
    'LineWidth', 1.2, 'DisplayName', 'Reference');
hold on;
[t, y] = sig(out, 'wm');     plot(t, y*rpm, 'Color', c.blue, ...
    'LineWidth', 1.6, 'DisplayName', 'Measured');
style_axes(gca, c, 'Time [s]', 'Speed [rpm]', ...
           'Rotor speed - reference vs. measured');
legend('Location', 'southeast');
paths{end+1} = save_fig(f, out_dir, variant, 'speed');

% ---- 1b. speed tracking error -------------------------------------------
f = new_fig(c);
[t, wr] = sig(out, 'w_ref');
[~, wm] = sig(out, 'wm');
plot(t, (wr - wm)*rpm, 'Color', c.blue, 'LineWidth', 1.6, ...
     'DisplayName', 'Tracking error');
hold on;
yline(0, 'Color', [195 194 183]/255, 'HandleVisibility', 'off');
style_axes(gca, c, 'Time [s]', 'Error [rpm]', ...
           'Speed tracking error (reference - measured)');
legend('Location', 'northwest');
paths{end+1} = save_fig(f, out_dir, variant, 'tracking_error');

% ---- 2. torque -----------------------------------------------------------
f = new_fig(c);
[t, y] = sig(out, 'TL');  plot(t, y, '--', 'Color', c.ink2, ...
    'LineWidth', 1.2, 'DisplayName', 'Load torque');
hold on;
[t, y] = sig(out, 'Te');  plot(t, y, 'Color', c.blue, ...
    'LineWidth', 1.6, 'DisplayName', 'Electromagnetic torque');
style_axes(gca, c, 'Time [s]', 'Torque [N m]', ...
           'Electromagnetic torque and load torque');
legend('Location', 'northwest');
paths{end+1} = save_fig(f, out_dir, variant, 'torque');

% ---- 3. dq currents ------------------------------------------------------
f = new_fig(c);
[t, y] = sig(out, 'id');
plot([t(1) t(end)], [P.id_ref P.id_ref], '--', 'Color', c.blue, ...
     'LineWidth', 1.1, 'DisplayName', 'i_d reference');
hold on;
plot(t, y, 'Color', c.blue, 'LineWidth', 1.6, 'DisplayName', 'i_d measured');
[t, y] = sig(out, 'iq_ref'); plot(t, y, '--', 'Color', c.orange, ...
    'LineWidth', 1.1, 'DisplayName', 'i_q reference');
[t, y] = sig(out, 'iq');     plot(t, y, 'Color', c.orange, ...
    'LineWidth', 1.6, 'DisplayName', 'i_q measured');
style_axes(gca, c, 'Time [s]', 'Current [A]', ...
           'Stator currents in the rotor-flux frame');
legend('Location', 'northwest', 'NumColumns', 2);
paths{end+1} = save_fig(f, out_dir, variant, 'currents_dq');

% ---- 4. rotor flux estimate ---------------------------------------------
f = new_fig(c);
plot([0 P.T_end], [P.psi_ref P.psi_ref], '--', 'Color', c.ink2, ...
     'LineWidth', 1.2, 'DisplayName', 'Flux reference');
hold on;
[t, y] = sig(out, 'psi_est'); plot(t, y, 'Color', c.blue, ...
    'LineWidth', 1.6, 'DisplayName', '\psi_r estimate (controller)');
style_axes(gca, c, 'Time [s]', 'Flux linkage [Wb]', 'Rotor flux linkage');
legend('Location', 'southeast');
paths{end+1} = save_fig(f, out_dir, variant, 'flux');

% ---- 5. phase currents (full + zoom) ------------------------------------
f = new_fig(c); f.Position(4) = 620;
tl = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
[t, y] = sig(out, 'i_abc');
names = {'i_a', 'i_b', 'i_c'};
cols  = {c.blue, c.orange, c.aqua};
ax1 = nexttile(tl); hold(ax1, 'on');
ax2 = nexttile(tl); hold(ax2, 'on');
for k = 1:3
    plot(ax1, t, y(:,k), 'Color', cols{k}, 'LineWidth', 1.0, ...
         'DisplayName', names{k});
    plot(ax2, t, y(:,k), 'Color', cols{k}, 'LineWidth', 1.4, ...
         'DisplayName', names{k});
end
xlim(ax2, [t(end)-0.06, t(end)]);
style_axes(ax1, c, '', 'Current [A]', 'Stator phase currents - full run');
style_axes(ax2, c, 'Time [s]', 'Current [A]', ...
           'Stator phase currents - zoom (last 60 ms)');
legend(ax1, 'Location', 'northwest', 'NumColumns', 3);
paths{end+1} = save_fig(f, out_dir, variant, 'currents_abc');

if strcmp(variant, 'library')
    cycle = phase_cycle(t, y(:,1), P);
    if ~isempty(cycle)
        f = new_fig(c);
        ax = axes(f);
        hold(ax, 'on');
        timeMs = (t(cycle) - t(cycle(1)))*1e3;
        for k = 1:3
            plot(ax, timeMs, y(cycle,k), 'Color', cols{k}, ...
                'LineWidth', 1.1, 'DisplayName', names{k});
        end
        xlim(ax, [0 timeMs(end)]);
        style_axes(ax, c, sprintf('Time since %.4f s (ms)', t(cycle(1))), ...
            'Current [A]', 'Stator phase currents over one electrical cycle');
        legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal');
        paths{end+1} = save_fig(f, out_dir, variant, 'currents_abc_cycle');
    end
end

if any(strcmp(who(out), 'v_dc'))
    f = new_fig(c);
    [t, vdc] = sig(out, 'v_dc');
    plot(t, vdc, 'Color', c.blue, 'LineWidth', 1.4, 'DisplayName', 'DC link');
    hold on;
    yline(P.Vdc, '--', 'Color', c.ink2, 'DisplayName', 'Ideal source');
    style_axes(gca, c, 'Time [s]', 'Voltage [V]', 'DC-link voltage with R-L source');
    legend('Location', 'best');
    paths{end+1} = save_fig(f, out_dir, variant, 'dc_link');
end

fprintf('saved %d figures to %s\n', numel(paths), out_dir);
end

% ==========================================================================
% helpers
% ==========================================================================

function [t, y] = sig(out, name)
% Extract a logged timeseries from the SimulationOutput by variable name.
ts = out.(name);
t = ts.Time;
y = squeeze(ts.Data);
% Row-vector signals logged as 1xNxM come back MxN after squeeze -> fix.
if size(y, 1) ~= numel(t), y = y.'; end
end

function idx = phase_cycle(t, phaseA, P)
% Locate two rising crossings of the PWM-period means on the settled plateau.
% The figure itself uses the original, unfiltered current samples.
idx = [];
window = find(t >= 1.05 & t < 1.25);
samples = round(1/(P.fsw*P.Ts_power));
window = window(1:floor(numel(window)/samples)*samples);
if numel(window) < 2*samples, return; end
periodTime = mean(reshape(t(window), samples, []), 1);
periodCurrent = mean(reshape(phaseA(window), samples, []), 1);
crossings = find(periodCurrent(1:end-1) < 0 & periodCurrent(2:end) >= 0 & periodTime(1:end-1) >= 1.09, 2);
if numel(crossings) < 2, return; end
crossingTime = periodTime(crossings) - periodCurrent(crossings).* ...
    (periodTime(crossings+1)-periodTime(crossings))./(periodCurrent(crossings+1)-periodCurrent(crossings));
idx = find(t >= crossingTime(1) & t <= crossingTime(2));
end

function f = new_fig(c)
f = figure('Color', c.surface, 'Visible', 'off', ...
           'Position', [80 80 900 460]);
end

function style_axes(ax, c, xlab, ylab, ttl)
grid(ax, 'on');
set(ax, 'Color', c.surface, 'GridColor', [225 224 217]/255, ...
        'GridAlpha', 1, 'Box', 'off', 'XColor', [137 135 129]/255, ...
        'YColor', [137 135 129]/255, 'FontSize', 9.5);
xlabel(ax, xlab, 'Color', c.ink2, 'FontSize', 10.5);
ylabel(ax, ylab, 'Color', c.ink2, 'FontSize', 10.5);
title(ax, ttl, 'Color', [11 11 11]/255, 'FontSize', 12, 'FontWeight', 'bold');
ax.TitleHorizontalAlignment = 'left';
end

function p = save_fig(f, out_dir, variant, name)
p = fullfile(out_dir, sprintf('%s_%s.png', variant, name));
exportgraphics(f, p, 'Resolution', 150);
close(f);
end
