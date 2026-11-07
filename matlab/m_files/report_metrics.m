function metrics = report_metrics(out, P)
%REPORT_METRICS  Print closed-loop performance metrics for a FOC run.
%
%   REPORT_METRICS(out, P) prints step settling times, plateau tracking
%   error, load-disturbance dip, final flux and peak current.
%
%   Signals are resampled onto a uniform grid first: the Version-1 model
%   runs a variable-step solver, so raw sample spacing would bias RMS
%   values toward the densely-sampled transients.

rpm = 60/(2*pi);
ts  = P.Ts_ctrl;
t   = (0:ts:P.T_end).';

wr = resample_sig(out, 'w_ref', t);
wm = resample_sig(out, 'wm',    t);
err = (wr - wm)*rpm;

% Measurement windows are bounded by the next scenario event and kept clear
% of it by a guard band: at a reference discontinuity the error equals the
% step size, which would otherwise be reported as a tracking figure.
events = sort([P.step_t, P.t_load]);
guard  = 5*ts;

fprintf('  settling times (+/-10 rpm):');
metrics.settling_ms = zeros(1, numel(P.step_t));
for k = 1:numel(P.step_t)
    t_end_k = next_event(events, P.step_t(k), P.T_end, 0.4) - guard;
    metrics.settling_ms(k) = 1e3*settling_time(t, err, P.step_t(k), t_end_k, 10);
    fprintf(' %5.0f ms', metrics.settling_ms(k));
end
fprintf('\n');

% Plateau quality: skip 150 ms after every event and the guard band before.
settled = t >= P.step_t(1) + 0.15;
for ev = events
    settled = settled & ~(t >= ev - guard & t < ev + 0.15);
end
metrics.plateau_max_rpm = max(abs(err(settled)));
metrics.plateau_rms_rpm = rms_of(err(settled));
fprintf('  plateau error   : max %6.2f rpm, rms %6.2f rpm\n', ...
        metrics.plateau_max_rpm, metrics.plateau_rms_rpm);

t_dip = next_event(events, P.t_load, P.T_end, 0.25) - guard;
win = t >= P.t_load & t <= t_dip;
metrics.load_dip_rpm = max(abs(err(win)));
fprintf('  load-step dip   : %6.2f rpm\n', metrics.load_dip_rpm);

% ---- common checks -------------------------------------------------------
psi = resample_sig(out, 'psi_est', t);
iabc = resample_sig(out, 'i_abc', t);
metrics.final_flux_wb = psi(end);
metrics.sampled_peak_current_a = max(abs(iabc(:)));
fprintf('  rotor flux (end): %6.3f Wb (ref %.2f)   peak current: %5.2f A\n', ...
        metrics.final_flux_wb, P.psi_ref, metrics.sampled_peak_current_a);
end

% ==========================================================================
% helpers
% ==========================================================================

function y = resample_sig(out, name, t)
% Logged timeseries -> values on the uniform grid t (columnwise).
ts = out.(name);
td = ts.Time;
yd = squeeze(ts.Data);
if size(yd, 1) ~= numel(td), yd = yd.'; end
y = interp1(td, yd, t, 'linear', 'extrap');
end

function r = rms_of(x)
r = sqrt(mean(x.^2));
end

function t_next = next_event(events, t_now, t_end, max_span)
% First scenario event strictly after t_now (or t_now + max_span / the end
% of the run, whichever comes first).
nxt = events(events > t_now + 1e-9);
t_next = min([nxt, t_now + max_span, t_end]);
end

function ts_out = settling_time(t, err, t_step, t_stop, band)
% Time from t_step until |err| stays inside +/-band [rpm], searching only
% up to t_stop so the next scenario event cannot contaminate the result.
win = t >= t_step & t <= t_stop;
tw = t(win);
ew = err(win);
last_bad = find(abs(ew) > band, 1, 'last');
if isempty(last_bad)
    ts_out = 0;
elseif last_bad == numel(ew)
    ts_out = NaN;                    % never settled inside the window
else
    ts_out = tw(last_bad + 1) - t_step;
end
end
