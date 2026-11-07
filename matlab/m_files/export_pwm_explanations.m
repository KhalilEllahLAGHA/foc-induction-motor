function export_pwm_explanations()
%EXPORT_PWM_EXPLANATIONS Draw the two modulation methods for the report.
P = motor_params();
outDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'outputs', 'diagrams');
if ~isfolder(outDir), mkdir(outDir); end
blue = [0.10 0.35 0.63];
orange = [0.76 0.31 0.09];
teal = [0.08 0.47 0.45];
gray = [0.45 0.49 0.55];

% Ten carrier cycles per sine cycle make the comparisons easy to read.
n = 10;
x = linspace(0, 1, 6001);
phase = mod(n*x, 1);
carrier = 1 - 4*min(phase, 1 - phase);
sine = 0.82*cos(2*pi*x);
held = 0.82*cos(2*pi*floor(n*x)/n);
gate = held >= carrier;
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 930 500]);
layout = tiledlayout(f, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(layout); hold(ax, 'on');
plot(ax, x, carrier, 'Color', gray, 'DisplayName', 'Triangle carrier');
plot(ax, x, sine, ':', 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'Phase A sine command');
stairs(ax, x, held, 'Color', blue, 'LineWidth', 1.5, 'DisplayName', 'Command held per PWM period');
ylim(ax, [-1.1 1.1]); ylabel(ax, 'Normalized level'); grid(ax, 'on');
legend(ax, 'Location', 'southoutside', 'NumColumns', 3);
title(ax, 'SPWM compares the phase command with the triangle');
ax = nexttile(layout);
area(ax, x, double(gate), 'FaceColor', blue, 'EdgeColor', blue, 'FaceAlpha', 0.29);
yticks(ax, [0 1]); ylim(ax, [-0.08 1.12]); grid(ax, 'on');
ylabel(ax, 'Upper gate'); xlabel(ax, 'Fraction of one electrical cycle');
title(ax, 'The upper gate is high when the held command exceeds the triangle');
exportgraphics(f, fullfile(outDir, 'spwm_comparator.png'), 'Resolution', 190);
close(f);

angle = (0:6)*pi/3;
radius = 2*P.Vdc/3;
circle = linspace(0, 2*pi, 241);
states = {'100', '110', '010', '011', '001', '101'};
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 840 620]);
ax = axes(f); hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');
vx = radius*cos(angle); vy = radius*sin(angle);
patch(ax, [0 vx(1:2)], [0 vy(1:2)], [0.88 0.95 0.99], ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax, vx, vy, 'Color', blue, 'LineWidth', 1.8, 'DisplayName', 'Feasible PWM-period averages');
plot(ax, P.v_max*cos(circle), P.v_max*sin(circle), '--', ...
    'Color', teal, 'LineWidth', 1.5, 'DisplayName', 'Largest constant radius (311.8 V)');
for k = 1:6
    quiver(ax, 0, 0, vx(k), vy(k), 0, 'Color', gray, 'LineWidth', 1.05, ...
        'MaxHeadSize', 0.10, 'HandleVisibility', 'off');
    text(ax, 1.13*vx(k), 1.13*vy(k), states{k}, 'HorizontalAlignment', 'center', ...
        'FontWeight', 'bold', 'FontSize', 11, 'Color', blue);
end
quiver(ax, 0, 0, 200, 100, 0, 'Color', orange, 'LineWidth', 2.1, ...
    'MaxHeadSize', 0.22, 'DisplayName', 'Example command (200, 100) V');
xlim(ax, [-448 448]); ylim(ax, [-395 395]);
xlabel(ax, 'V_\alpha (V)'); ylabel(ax, 'V_\beta (V)');
title(ax, 'SVPWM uses the two states on either side of the requested vector');
legend(ax, 'Location', 'southoutside', 'NumColumns', 1);
exportgraphics(f, fullfile(outDir, 'svpwm_hexagon.png'), 'Resolution', 190);
close(f);
end
