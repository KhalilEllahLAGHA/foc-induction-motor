function gates = foc_pwm_gates(uref, time, fsw, step, useSvpwm)
%FOC_PWM_GATES Custom SPWM or seven-segment SVPWM for the IGBT bridge.
% uref contains the phase-voltage commands divided by half the DC voltage.
% The model holds uref at the PWM rate and calls this function at each step.
% Evaluate at the centre of a power step to round gate edges to the grid.
samplesPerPeriod = round(1/(fsw*step));
sample = mod(round(time/step), samplesPerPeriod);
phase = (sample + 0.5)/samplesPerPeriod;
edgeDistance = min(phase, 1 - phase);

if useSvpwm
    upper = space_vector_state(uref, edgeDistance);
else
    carrier = 1 - 4*edgeDistance;
    upper = reshape(uref >= carrier, 1, 3);
end

% Universal Bridge IGBT gate order: A upper/lower, B upper/lower, C upper/lower.
gates = reshape(double([upper; ~upper]), 1, 6);
end

function upper = space_vector_state(uref, edgeDistance)
alpha = (2*uref(1) - uref(2) - uref(3))/3;
beta = (uref(2) - uref(3))/sqrt(3);
angle = mod(atan2(beta, alpha), 2*pi);
sectorWidth = pi/3;
sector = min(floor(angle/sectorWidth) + 1, 6);
localAngle = angle - (sector - 1)*sectorWidth;

% Dwell times as fractions of a PWM period. The command uses Vdc/2 units.
index = sqrt(3)/2*hypot(alpha, beta);
t1 = index*sin(sectorWidth - localAngle);
t2 = index*sin(localAngle);
scale = max(1, t1 + t2);
t1 = t1/scale;
t2 = t2/scale;
t0 = max(0, 1 - t1 - t2);

states = logical([1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1; 1 0 1]);
nextSector = mod(sector, 6) + 1;
if mod(sector, 2) == 1
    singleHigh = states(sector, :);
    doubleHigh = states(nextSector, :);
    singleTime = t1;
else
    singleHigh = states(nextSector, :);
    doubleHigh = states(sector, :);
    singleTime = t2;
end

% Evaluate the forward half of the sequence and mirror it in the second half.
if edgeDistance < t0/4
    upper = [false false false];
elseif edgeDistance < t0/4 + singleTime/2
    upper = singleHigh;
elseif edgeDistance < t0/4 + (t1 + t2)/2
    upper = doubleHigh;
else
    upper = [true true true];
end
end
