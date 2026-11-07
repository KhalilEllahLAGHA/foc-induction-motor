% Run the saved Simscape FOC model with the modulation in motor_params.m.
% Use compare_foc_pwm to run both modes.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'm_files'));

foc_init;

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

close_system(mdl, 0);
