function paths = build_foc_pwm_variants(variant)
%BUILD_FOC_PWM_VARIANTS Save all graphical PWM models or one named variant.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'm_files'));
template = fullfile(here, 'foc_im_library.slx');
names = {'foc_im_spwm', 'foc_im_svpwm', 'foc_im_svpwm_4seg'};
if nargin > 0
    names = {validatestring(variant, names)};
end
assert(~any(cellfun(@bdIsLoaded, names)), 'Close the requested PWM models before rebuilding.');
staging = tempname;
mkdir(staging);
previousFolder = pwd;
cleanup = onCleanup(@() remove_staging(staging, names, previousFolder)); %#ok<NASGU>
cd(staging);

for k = 1:numel(names)
    model = names{k};
    file = fullfile(staging, [model '.slx']);
    copyfile(template, file);
    load_system(file);
    sys = [model '/PWM Generator'];
    lines = find_system(sys, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
    roots = arrayfun(@(line) get_param(line, 'LineParent') == -1, lines);
    for line = lines(roots).'
        delete_line(line);
    end
    blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    keep = {sys, [sys '/Uref'], [sys '/g']};
    obsolete = setdiff(blocks, keep);
    for n = 1:numel(obsolete)
        delete_block(obsolete{n});
    end
    if strcmp(model, 'foc_im_spwm')
        build_spwm(sys);
    else
        build_svpwm(sys, strcmp(model, 'foc_im_svpwm'));
    end
    straighten_foc_wires(model);
    save_system(model);
    close_system(model, 0);
end

paths = cell(size(names));
for k = 1:numel(names)
    name = [names{k} '.slx'];
    paths{k} = fullfile(here, name);
    copyfile(fullfile(staging, name), paths{k}, 'f');
end
end

function remove_staging(folder, names, previousFolder)
cd(previousFolder);
for k = 1:numel(names)
    if bdIsLoaded(names{k}), close_system(names{k}, 0); end
    file = fullfile(folder, [names{k} '.slx']);
    if isfile(file), delete(file); end
end
rmdir(folder);
end

function build_spwm(sys)
set_param([sys '/Uref'], 'Position', [25 96 55 110]);
blk(sys, 'Discrete/Zero-Order Hold', 'Hold per PWM period', [105 80 215 126], 'SampleTime', '1/P.fsw');
blk(sys, 'Signal Routing/Demux', 'Phase references', [265 61 270 219], 'Outputs', '3');
blk(sys, 'built-in/Subsystem', 'PWM clock', [105 265 215 310]);
build_clock([sys '/PWM clock'], true);
gain(sys, 'Negative slope', '-4', [265 269 335 305]);
blk(sys, 'Math Operations/Bias', 'Triangle carrier', [380 269 460 305], 'Bias', '1');
phases = 'ABC';
for k = 1:numel(phases)
    y = 56 + (k-1)*65;
    name = ['Compare phase ' phases(k)];
    blk(sys, 'Logic and Bit Operations/Relational Operator', name, ...
        [510 y 575 y+40], 'Operator', '>=');
    ln(sys, sprintf('Phase references/%d', k), [name '/1']);
    ln(sys, 'Triangle carrier/1', [name '/2']);
end
blk(sys, 'Signal Routing/Mux', 'Upper gates', [625 60 630 222], 'Inputs', '3');
blk(sys, 'built-in/Subsystem', 'IGBT gate order', [675 107 780 170]);
build_gate_order([sys '/IGBT gate order']);
set_param([sys '/g'], 'Position', [825 131 855 145]);
wire(sys, {
    'Uref/1', 'Hold per PWM period/1';
    'Hold per PWM period/1', 'Phase references/1';
    'PWM clock/1', 'Negative slope/1';
    'Negative slope/1', 'Triangle carrier/1';
    'Compare phase A/1', 'Upper gates/1';
    'Compare phase B/1', 'Upper gates/2';
    'Compare phase C/1', 'Upper gates/3';
    'Upper gates/1', 'IGBT gate order/1';
    'IGBT gate order/1', 'g/1'});
end

function build_svpwm(sys, centered)
set_param([sys '/Uref'], 'Position', [25 121 55 135]);
blk(sys, 'Discrete/Zero-Order Hold', 'Hold per PWM period', [95 105 205 151], 'SampleTime', '1/P.fsw');
blk(sys, 'built-in/Subsystem', 'Sector and dwell times', [245 55 405 200]);
build_dwell([sys '/Sector and dwell times']);
blk(sys, 'built-in/Subsystem', 'PWM clock', [95 232 205 282]);
build_clock([sys '/PWM clock'], centered);
gain(sys, 'Time in seconds', '1/P.fsw', [265 237 360 277]);
sequence = 'Seven segment sequence';
if ~centered, sequence = 'Four segment sequence'; end
blk(sys, 'built-in/Subsystem', sequence, [465 48 625 225]);
build_sequence([sys '/' sequence], centered);
blk(sys, 'built-in/Subsystem', 'IGBT gate order', [680 108 795 167]);
build_gate_order([sys '/IGBT gate order']);
set_param([sys '/g'], 'Position', [840 130 870 144]);
wire(sys, {
    'Uref/1', 'Hold per PWM period/1';
    'Hold per PWM period/1', 'Sector and dwell times/1';
    'Sector and dwell times/1', [sequence '/1'];
    'Sector and dwell times/2', [sequence '/2'];
    'Sector and dwell times/3', [sequence '/3'];
    'Sector and dwell times/4', [sequence '/4'];
    'PWM clock/1', 'Time in seconds/1';
    'Time in seconds/1', [sequence '/5'];
    [sequence '/1'], 'IGBT gate order/1';
    'IGBT gate order/1', 'g/1'});
end

function build_clock(sys, mirrored)
% Sample the midpoint of each electrical step.
blk(sys, 'Sources/Digital Clock', 'Power step clock', [25 59 115 95], 'SampleTime', 'P.Ts_power');
gain(sys, 'Seconds to samples', '1/P.Ts_power', [155 59 245 95]);
math(sys, 'Nearest sample', 'Rounding Function', 'round', [280 59 350 95]);
constant(sys, 'Samples per period', 'round(1/(P.fsw*P.Ts_power))', [270 125 355 159]);
math(sys, 'Sample in period', 'Math Function', 'mod', [390 52 455 102]);
blk(sys, 'Math Operations/Bias', 'Midpoint of step', [490 60 555 94], 'Bias', '0.5');
gain(sys, 'Fraction of period', '1/round(1/(P.fsw*P.Ts_power))', [595 60 675 94]);
wire(sys, {
    'Power step clock/1', 'Seconds to samples/1';
    'Seconds to samples/1', 'Nearest sample/1';
    'Nearest sample/1', 'Sample in period/1';
    'Samples per period/1', 'Sample in period/2';
    'Sample in period/1', 'Midpoint of step/1';
    'Midpoint of step/1', 'Fraction of period/1'});
if ~mirrored
    outports(sys, {'Elapsed fraction'}, 715, 70);
    ln(sys, 'Fraction of period/1', 'Elapsed fraction/1');
    return
end
constant(sys, 'One period', '1', [590 125 675 154]);
sumblk(sys, 'Remaining fraction', '+-', [715 107 760 157]);
blk(sys, 'Math Operations/MinMax', 'Distance to nearest edge', [810 53 875 103], 'Function', 'min', 'Inputs', '2');
blk(sys, 'Sinks/Out1', 'Edge fraction', [920 71 950 85]);
wire(sys, {
    'One period/1', 'Remaining fraction/1';
    'Fraction of period/1', 'Remaining fraction/2';
    'Fraction of period/1', 'Distance to nearest edge/1';
    'Remaining fraction/1', 'Distance to nearest edge/2';
    'Distance to nearest edge/1', 'Edge fraction/1'});
end

function build_dwell(sys)
blk(sys, 'Sources/In1', 'Uref', [25 77 55 91], 'PortDimensions', '3');
blk(sys, 'Math Operations/Gain', 'Clarke transform', [90 65 185 103], ...
    'Gain', '[2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)]', 'Multiplication', 'Matrix(K*u) (u vector)');
blk(sys, 'Signal Routing/Demux', 'Alpha and beta', [225 53 230 115], 'Outputs', '2');
blk(sys, 'built-in/Subsystem', 'Sector and local angle', [275 40 415 130]);
build_sector([sys '/Sector and local angle']);
math(sys, 'Vector magnitude', 'Math Function', 'hypot', [310 164 380 214]);
gain(sys, 'Modulation index', 'sqrt(3)/2', [420 169 500 209]);
blk(sys, 'built-in/Subsystem', 'Active and zero dwells', [550 118 715 245]);
build_times([sys '/Active and zero dwells']);
outports(sys, {'Sector', 'T1 seconds', 'T2 seconds', 'T0 seconds'}, 780, [63 139 182 223]);
wire(sys, {
    'Uref/1', 'Clarke transform/1';
    'Clarke transform/1', 'Alpha and beta/1';
    'Alpha and beta/1', 'Sector and local angle/1';
    'Alpha and beta/2', 'Sector and local angle/2';
    'Alpha and beta/1', 'Vector magnitude/1';
    'Alpha and beta/2', 'Vector magnitude/2';
    'Sector and local angle/1', 'Sector/1';
    'Sector and local angle/2', 'Active and zero dwells/1';
    'Vector magnitude/1', 'Modulation index/1';
    'Modulation index/1', 'Active and zero dwells/2';
    'Active and zero dwells/1', 'T1 seconds/1';
    'Active and zero dwells/2', 'T2 seconds/1';
    'Active and zero dwells/3', 'T0 seconds/1'});
end

function build_sector(sys)
inports(sys, {'Alpha', 'Beta'}, 25, [70 32]);
math(sys, 'Vector angle', 'Trigonometric Function', 'atan2', [105 35 165 79]);
constant(sys, 'Full revolution', '2*pi', [95 114 170 145]);
math(sys, 'Wrap angle', 'Math Function', 'mod', [210 48 260 98]);
gain(sys, 'Angle in sectors', '3/pi', [300 55 380 90]);
math(sys, 'Sector index', 'Rounding Function', 'floor', [420 55 490 90]);
blk(sys, 'Discontinuities/Saturation', 'Valid sector index', [525 55 605 90], 'LowerLimit', '0', 'UpperLimit', '5');
blk(sys, 'Math Operations/Bias', 'Sector number', [655 55 725 90], 'Bias', '1');
gain(sys, 'Sector start angle', 'pi/3', [640 125 725 162]);
sumblk(sys, 'Angle within sector', '+-', [765 112 810 162]);
outports(sys, {'Sector', 'Local angle'}, 860, [65 130]);
wire(sys, {
    'Beta/1', 'Vector angle/1';
    'Alpha/1', 'Vector angle/2';
    'Vector angle/1', 'Wrap angle/1';
    'Full revolution/1', 'Wrap angle/2';
    'Wrap angle/1', 'Angle in sectors/1';
    'Angle in sectors/1', 'Sector index/1';
    'Sector index/1', 'Valid sector index/1';
    'Valid sector index/1', 'Sector number/1';
    'Sector number/1', 'Sector/1';
    'Valid sector index/1', 'Sector start angle/1';
    'Wrap angle/1', 'Angle within sector/1';
    'Sector start angle/1', 'Angle within sector/2';
    'Angle within sector/1', 'Local angle/1'});
end

function build_times(sys)
inports(sys, {'Local angle', 'Modulation index'}, 25, [103 227]);
constant(sys, 'Sector width', 'pi/3', [80 26 145 54]);
sumblk(sys, 'Angle to next', '+-', [183 44 220 93]);
math(sys, 'Sine for T1', 'Trigonometric Function', 'sin', [260 45 322 88]);
math(sys, 'Sine for T2', 'Trigonometric Function', 'sin', [260 141 322 184]);
blk(sys, 'Math Operations/Product', 'Raw T1 fraction', [365 48 425 91]);
blk(sys, 'Math Operations/Product', 'Raw T2 fraction', [365 144 425 187]);
sumblk(sys, 'Raw active fraction', '++', [475 235 520 283]);
constant(sys, 'One PWM period', '1', [461 321 525 351]);
blk(sys, 'Math Operations/MinMax', 'Scale for overmodulation', [568 252 641 311], 'Function', 'max', 'Inputs', '2');
blk(sys, 'Math Operations/Product', 'T1 fraction', [677 45 733 95], 'Inputs', '*/');
blk(sys, 'Math Operations/Product', 'T2 fraction', [677 141 733 191], 'Inputs', '*/');
sumblk(sys, 'Zero vector fraction', '+--', [780 233 828 291]);
blk(sys, 'Discontinuities/Saturation', 'Clamp negative roundoff', [865 245 940 279], 'LowerLimit', '0', 'UpperLimit', 'inf');
dwellIds = [1 2 0];
y = [53 149 245];
for k = 1:numel(dwellIds)
    gain(sys, sprintf('T%d in seconds', dwellIds(k)), '1/P.fsw', [978 y(k) 1058 y(k)+34]);
end
outports(sys, {'T1 seconds', 'T2 seconds', 'T0 seconds'}, 1110, [63 159 255]);
wire(sys, {
    'Sector width/1', 'Angle to next/1';
    'Local angle/1', 'Angle to next/2';
    'Angle to next/1', 'Sine for T1/1';
    'Local angle/1', 'Sine for T2/1';
    'Sine for T1/1', 'Raw T1 fraction/1';
    'Modulation index/1', 'Raw T1 fraction/2';
    'Sine for T2/1', 'Raw T2 fraction/1';
    'Modulation index/1', 'Raw T2 fraction/2';
    'Raw T1 fraction/1', 'Raw active fraction/1';
    'Raw T2 fraction/1', 'Raw active fraction/2';
    'Raw active fraction/1', 'Scale for overmodulation/1';
    'One PWM period/1', 'Scale for overmodulation/2';
    'Raw T1 fraction/1', 'T1 fraction/1';
    'Scale for overmodulation/1', 'T1 fraction/2';
    'Raw T2 fraction/1', 'T2 fraction/1';
    'Scale for overmodulation/1', 'T2 fraction/2';
    'One PWM period/1', 'Zero vector fraction/1';
    'T1 fraction/1', 'Zero vector fraction/2';
    'T2 fraction/1', 'Zero vector fraction/3';
    'Zero vector fraction/1', 'Clamp negative roundoff/1';
    'T1 fraction/1', 'T1 in seconds/1';
    'T2 fraction/1', 'T2 in seconds/1';
    'Clamp negative roundoff/1', 'T0 in seconds/1';
    'T1 in seconds/1', 'T1 seconds/1';
    'T2 in seconds/1', 'T2 seconds/1';
    'T0 in seconds/1', 'T0 seconds/1'});
end

function build_sequence(sys, centered)
% The turn-on instants build 000, one high, two high, then 111.
clockPort = 'Edge time seconds';
if ~centered, clockPort = 'Elapsed time seconds'; end
inports(sys, {'Sector', 'T1 seconds', 'T2 seconds', 'T0 seconds', clockPort}, ...
    25, [65 126 176 225 383]);
blk(sys, 'built-in/Subsystem', 'Turn-on instants', [180 45 315 255]);
build_edges([sys '/Turn-on instants'], centered);
blk(sys, 'built-in/Subsystem', 'Leg order by sector', [180 286 315 338]);
build_leg_order([sys '/Leg order by sector']);
blk(sys, 'Signal Routing/Selector', 'Turn-on instants for A B C', [390 154 510 222], ...
    'InputPortWidth', '3', 'IndexOptionArray', {'Index vector (port)'});
blk(sys, 'Logic and Bit Operations/Relational Operator', 'Turn on each upper IGBT', ...
    [570 171 665 237], 'Operator', '<=');
outports(sys, {'Upper gates'}, 720, 197);
wire(sys, {
    'Sector/1', 'Turn-on instants/1';
    'T1 seconds/1', 'Turn-on instants/2';
    'T2 seconds/1', 'Turn-on instants/3';
    'T0 seconds/1', 'Turn-on instants/4';
    'Sector/1', 'Leg order by sector/1';
    'Turn-on instants/1', 'Turn-on instants for A B C/1';
    'Leg order by sector/1', 'Turn-on instants for A B C/2';
    'Turn-on instants for A B C/1', 'Turn on each upper IGBT/1';
    [clockPort '/1'], 'Turn on each upper IGBT/2';
    'Turn on each upper IGBT/1', 'Upper gates/1'});
end

function build_edges(sys, centered)
inports(sys, {'Sector', 'T1 seconds', 'T2 seconds', 'T0 seconds'}, 25, [138 60 232 350]);
constant(sys, 'Two sectors', '2', [82 177 147 206]);
math(sys, 'Odd sector', 'Math Function', 'mod', [183 126 235 169]);
blk(sys, 'Signal Routing/Switch', 'First active dwell', [285 58 365 185], ...
    'Criteria', 'u2 ~= 0');
sumblk(sys, 'Both active dwells', '++', [298 224 349 273]);
activeScale = '0.5';
zeroScale = '0.25';
if ~centered
    activeScale = '1';
    zeroScale = '0.5';
end
gain(sys, 'First active interval', activeScale, [411 101 488 141]);
gain(sys, 'Total active interval', activeScale, [411 232 488 272]);
gain(sys, 'Initial zero interval', zeroScale, [410 333 490 381]);
sumblk(sys, 'Second edge', '++', [550 108 600 162]);
sumblk(sys, 'Third edge', '++', [550 227 600 281]);
blk(sys, 'Signal Routing/Mux', 'First second third', [665 117 670 338], 'Inputs', '3');
outports(sys, {'Edge times'}, 730, 220);
wire(sys, {
    'Sector/1', 'Odd sector/1';
    'Two sectors/1', 'Odd sector/2';
    'T1 seconds/1', 'First active dwell/1';
    'Odd sector/1', 'First active dwell/2';
    'T2 seconds/1', 'First active dwell/3';
    'T1 seconds/1', 'Both active dwells/1';
    'T2 seconds/1', 'Both active dwells/2';
    'First active dwell/1', 'First active interval/1';
    'Both active dwells/1', 'Total active interval/1';
    'T0 seconds/1', 'Initial zero interval/1';
    'First active interval/1', 'Second edge/1';
    'Initial zero interval/1', 'Second edge/2';
    'Total active interval/1', 'Third edge/1';
    'Initial zero interval/1', 'Third edge/2';
    'Initial zero interval/1', 'First second third/1';
    'Second edge/1', 'First second third/2';
    'Third edge/1', 'First second third/3';
    'First second third/1', 'Edge times/1'});
end

function build_leg_order(sys)
% Each vector indexes the first, second and third edges for legs A, B and C.
inports(sys, {'Sector'}, 25, 28);
blk(sys, 'Signal Routing/Multiport Switch', 'Choose edge rank for A B C', [250 21 342 357], ...
    'DataPortOrder', 'One-based contiguous', 'Inputs', '6');
ranks = {'[1 2 3]', '[2 1 3]', '[3 1 2]', '[3 2 1]', '[2 3 1]', '[1 3 2]'};
for k = 1:numel(ranks)
    name = sprintf('Sector %d', k);
    y = 61 + (k-1)*49;
    constant(sys, name, ranks{k}, [95 y 171 y+32]);
    ln(sys, [name '/1'], sprintf('Choose edge rank for A B C/%d', k+1));
end
outports(sys, {'A B C edge ranks'}, 395, 181);
ln(sys, 'Sector/1', 'Choose edge rank for A B C/1');
ln(sys, 'Choose edge rank for A B C/1', 'A B C edge ranks/1');
end

function build_gate_order(sys)
inports(sys, {'Upper gates'}, 25, 83);
blk(sys, 'Logic and Bit Operations/Logical Operator', 'Complement', [105 135 175 174], 'Operator', 'NOT');
blk(sys, 'Signal Attributes/Data Type Conversion', 'Upper as double', [215 70 295 110], 'OutDataTypeStr', 'double');
blk(sys, 'Signal Attributes/Data Type Conversion', 'Lower as double', [215 135 295 174], 'OutDataTypeStr', 'double');
blk(sys, 'Signal Routing/Mux', 'Upper then lower', [345 72 350 173], 'Inputs', '2');
blk(sys, 'Signal Routing/Selector', 'Interleave A B C pairs', [405 101 520 145], ...
    'InputPortWidth', '6', 'IndexOptionArray', {'Index vector (dialog)'}, 'IndexParamArray', {'[1 4 2 5 3 6]'});
outports(sys, {'g'}, 575, 116);
wire(sys, {
    'Upper gates/1', 'Complement/1';
    'Upper gates/1', 'Upper as double/1';
    'Complement/1', 'Lower as double/1';
    'Upper as double/1', 'Upper then lower/1';
    'Lower as double/1', 'Upper then lower/2';
    'Upper then lower/1', 'Interleave A B C pairs/1';
    'Interleave A B C pairs/1', 'g/1'});
end

function blk(sys, library, name, position, varargin)
if ~startsWith(library, 'built-in/')
    library = ['simulink/' library];
end
add_block(library, [sys '/' name], 'Position', position, varargin{:});
end

function gain(sys, name, value, position)
blk(sys, 'Math Operations/Gain', name, position, 'Gain', value);
end

function constant(sys, name, value, position)
blk(sys, 'Sources/Constant', name, position, 'Value', value);
end

function math(sys, name, kind, op, position)
blk(sys, ['Math Operations/' kind], name, position, 'Operator', op);
end

function sumblk(sys, name, signs, position)
blk(sys, 'Math Operations/Sum', name, position, 'Inputs', signs, 'IconShape', 'rectangular');
end

function inports(sys, names, x, y)
for k = 1:numel(names)
    blk(sys, 'Sources/In1', names{k}, [x y(k) x+30 y(k)+14], 'Port', num2str(k));
end
end

function outports(sys, names, x, y)
for k = 1:numel(names)
    blk(sys, 'Sinks/Out1', names{k}, [x y(k) x+30 y(k)+14], 'Port', num2str(k));
end
end

function ln(sys, from, to)
add_line(sys, from, to, 'autorouting', 'on');
end

function wire(sys, pairs)
for k = 1:size(pairs,1)
    ln(sys, pairs{k,1}, pairs{k,2});
end
end
