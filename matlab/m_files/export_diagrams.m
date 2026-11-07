function paths = export_diagrams()
%EXPORT_DIAGRAMS  Save PNG images of both Simulink models and subsystems.
%
%   paths = EXPORT_DIAGRAMS() builds the models if their .slx files are
%   missing, then writes one image per diagram into matlab/outputs/diagrams/
%   for use in the report.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'm_files'));
addpath(fullfile(root, 'simulink_blocks'));
addpath(fullfile(root, 'simulink_library'));

out_dir = fullfile(root, 'outputs', 'diagrams');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

if ~evalin('base', 'exist(''P'', ''var'')')
    evalin('base', 'foc_init;');
end

build_if_missing(fullfile(root, 'simulink_blocks', 'foc_im_blocks.slx'), ...
                 @build_foc_blocks_model);
build_if_missing(fullfile(root, 'simulink_library', 'foc_im_library.slx'), ...
                 @build_foc_library_model);

targets = {
    'foc_im_blocks',                        'blocks_top'
    'foc_im_blocks/Speed Reference',        'blocks_speed_reference'
    'foc_im_blocks/FOC Controller',         'blocks_controller'
    'foc_im_blocks/FOC Controller/Id PI',   'blocks_pi'
    'foc_im_blocks/Flux estimator & slip',  'blocks_flux_estimator'
    'foc_im_blocks/SVPWM Inverter (avg)',   'blocks_inverter'
    'foc_im_blocks/SVPWM Inverter (avg)/RLC DC link', 'blocks_dc_link'
    'foc_im_blocks/Induction Motor',        'blocks_motor'
    'foc_im_library',                       'library_top'
    'foc_im_library/FOC Controller',        'library_controller'
    'foc_im_library/PWM Generator',         'library_pwm'
    };

% the flux estimator lives inside the controller subsystem
targets{5, 1} = 'foc_im_blocks/FOC Controller/Flux estimator & slip';

paths = {};
for k = 1:size(targets, 1)
    sys = targets{k, 1};
    mdl = strtok(sys, '/');
    load_system(mdl);
    file = fullfile(out_dir, [targets{k, 2} '.png']);
    try
        print(['-s' sys], '-dpng', '-r150', file);
        paths{end+1} = file; %#ok<AGROW>
        fprintf('  %s\n', file);
    catch err
        fprintf('  SKIPPED %s (%s)\n', sys, err.message);
    end
end

bdclose('all');
fprintf('exported %d diagrams to %s\n', numel(paths), out_dir);
end

function build_if_missing(slx, builder)
if ~isfile(slx)
    fprintf('building %s ...\n', slx);
    builder();
    bdclose('all');
end
end
