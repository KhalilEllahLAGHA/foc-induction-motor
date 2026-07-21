%
%   Version 1 = FOC drive assembled ONLY from fundamental Simulink blocks
%   (foc_im_blocks.slx).
%
%   Usage:  open MATLAB in this folder (or addpath it) and run:
%       >> run_foc_blocks
%
%   Test scenario (same as scenario A of python/main.py): a staircase of
%   constant speed levels - 800, 1400, 600 and -800 rpm, each reached by a
%   sudden reference step - with a 10 N*m load step at 0.55 s.
%   Figures are saved as blocks_*.png in matlab/outputs/graphs/.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'm_files'));

foc_init;                                   % defines P in this workspace

mdl = 'foc_im_blocks';
slx = fullfile(here, [mdl '.slx']);
if ~isfile(slx)
    error('%s.slx not found in %s', mdl, here);
end
load_system(slx);

fprintf('simulating %s (0 .. %.2f s)...\n', mdl, P.T_end);
out = sim(mdl);

report_metrics(out, P);
plot_results(out, 'blocks');

close_system(mdl, 0);      % discard the dirty flag left by sim()
