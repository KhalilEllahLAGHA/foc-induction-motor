%
%   Version 2 = FOC drive using ready-made library components (Simscape
%   Electrical Specialized Power Systems: Asynchronous Machine, Universal
%   Bridge, PWM Generator, transforms + Simulink PID blocks) in
%   foc_im_library.slx.
%
%   Usage:  open MATLAB in this folder (or addpath it) and run:
%       >> run_foc_library
%
%   Test scenario (same as Version 1): a staircase of constant speed levels
%   - 800, 1400, 600 and -800 rpm, each reached by a sudden reference step
%   - with a 10 N*m load step at 0.55 s.  Figures: library_*.png.
%
%   Requires Simscape Electrical (Specialized Power Systems) in R2024b.
%   The switching power stage runs at Ts_power = 2.5 us, so this simulation
%   is far slower than Version 1 - expect several minutes.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'm_files'));

foc_init;                                   % defines P in this workspace

mdl = 'foc_im_library';
slx = fullfile(here, [mdl '.slx']);
if ~isfile(slx)
    error('%s.slx not found in %s', mdl, here);
end
load_system(slx);

fprintf('simulating %s (0 .. %.2f s, switching model - be patient)...\n', ...
        mdl, P.T_end);
out = sim(mdl);

report_metrics(out, P);
plot_results(out, 'library');

close_system(mdl, 0);      % discard the dirty flag left by sim()
