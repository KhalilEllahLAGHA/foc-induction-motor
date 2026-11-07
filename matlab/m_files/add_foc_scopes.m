function add_foc_scopes(mdl)
%ADD_FOC_SCOPES Group drive readouts and workspace loggers in one panel.
sys = [mdl '/Scopes'];
if getSimulinkBlockHandle(sys) ~= -1, return; end
isSwitching = strcmp(mdl, 'foc_im_library');
signals = {'w_ref','wm','Te','TL','i_abc','id','iq_ref','iq','psi_est'};
ports = {'w ref','w rotor','Te','TL','i abc','id','iq ref','iq','psi rotor'};
if isSwitching
    signals{end+1} = 'gates'; ports{end+1} = 'IGBT gates';
else
    signals{end+1} = 'v_dc'; ports{end+1} = 'v dc';
end

source = zeros(size(signals));
logger = zeros(size(signals));
sourceY = zeros(size(signals));
for k = 1:numel(signals)
    if strcmp(signals{k}, 'gates')
        gatePorts = get_param([mdl '/PWM Generator'], 'PortHandles');
        source(k) = gatePorts.Outport;
    else
        logger(k) = getSimulinkBlockHandle([mdl '/log_' signals{k}]);
        logPorts = get_param(logger(k), 'PortHandles');
        source(k) = get_param(get_param(logPorts.Inport, 'Line'), 'SrcPortHandle');
    end
    xy = get_param(source(k), 'Position');
    sourceY(k) = xy(2);
end

top = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Block');
positions = cell2mat(get_param(setdiff(top, logger), 'Position'));
left = max(positions(:,3)) + 200;
topY = min(sourceY) - 20;
bottomY = max(max(sourceY) + 20, topY + 32*numel(signals));
add_block('built-in/Subsystem', sys, 'Position', [left topY left+135 bottomY]);

% Port order follows the source heights to keep the top-level branches short.
[~, order] = sort(sourceY);
for port = 1:numel(order)
    k = order(port);
    row = 50 + 62*(k-1);
    block(sys, 'Sources/In1', ports{k}, [25 row 55 row+14], 'Port', num2str(port));
    if logger(k) > 0
        name = ['log_' signals{k}];
        add_block(getfullname(logger(k)), [sys '/' name], ...
            'Position', [160 row-8 245 row+22], 'ShowName', 'off');
        logPorts = get_param(logger(k), 'PortHandles');
        delete_line(mdl, source(k), logPorts.Inport);
        delete_block(logger(k));
        wire(sys, [ports{k} '/1'], [name '/1']);
    end
end

inputs = get_param(sys, 'PortHandles');
monitorLines = zeros(size(order));
for port = 1:numel(order)
    monitorLines(port) = add_line(mdl, source(order(port)), inputs.Inport(port), 'autorouting', 'on');
end
Simulink.BlockDiagram.routeLine(monitorLines);

% Speed and torque are display-decimated in the fixed-step switching model.
decimation = 1;
if isSwitching, decimation = 100; end
block(sys, 'Math Operations/Gain', 'Reference to rpm', [302 28 367 56], 'Gain', '60/(2*pi)');
block(sys, 'Math Operations/Gain', 'Speed to rpm', [302 81 367 109], 'Gain', '60/(2*pi)');
wire(sys, 'w ref/1', 'Reference to rpm/1');
wire(sys, 'w rotor/1', 'Speed to rpm/1');
scope(sys, 'Speed', [565 25 655 108], 'Speed (rpm)', ...
    {'Reference to rpm/1','Speed to rpm/1'}, {'Reference','Rotor speed'}, decimation, 'Auto');
scope(sys, 'Torque', [565 144 655 225], 'Torque (N m)', ...
    {'Te/1','TL/1'}, {'Electromagnetic torque','Load torque'}, decimation, 'Auto');

block(sys, 'Signal Routing/Demux', 'Split phase currents', [350 247 356 328], 'Outputs', '3');
wire(sys, 'i abc/1', 'Split phase currents/1');
span = 'Auto';
if isSwitching, span = '0.04'; end
scope(sys, 'Phase currents', [565 247 655 318], 'Current (A)', ...
    {'Split phase currents/1','Split phase currents/2','Split phase currents/3'}, ...
    {'Phase A','Phase B','Phase C'}, 1, span);

block(sys, 'Sources/Constant', 'd current reference', [315 348 382 378], ...
    'Value', 'P.id_ref', 'ShowName', 'off');
scope(sys, 'd current', [565 346 655 407], 'Current (A)', ...
    {'d current reference/1','id/1'}, {'d reference','d measured'}, 1, 'Auto');
scope(sys, 'q current', [565 432 655 493], 'Current (A)', ...
    {'iq ref/1','iq/1'}, {'q reference','q measured'}, 1, 'Auto');
block(sys, 'Sources/Constant', 'Flux reference', [315 516 382 546], 'Value', 'P.psi_ref');
scope(sys, 'Rotor flux', [565 518 655 577], 'Flux linkage (Wb)', ...
    {'Flux reference/1','psi rotor/1'}, {'Reference','Estimate'}, 1, 'Auto');

if isSwitching
    block(sys, 'Math Operations/Reshape', 'Gate vector', [185 612 250 644], ...
        'OutputDimensionality', '1-D array');
    block(sys, 'Math Operations/Gain', 'Upper gates', [300 612 381 644], ...
        'Gain', '[1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0]', 'Multiplication', 'Matrix(K*u)');
    block(sys, 'Signal Routing/Demux', 'Split upper gates', [431 601 437 668], 'Outputs', '3');
    wire(sys, 'IGBT gates/1', 'Gate vector/1');
    wire(sys, 'Gate vector/1', 'Upper gates/1');
    wire(sys, 'Upper gates/1', 'Split upper gates/1');
    gateView = scope(sys, 'IGBT upper gates', [565 604 655 675], 'Gate state', ...
        {'Split upper gates/1','Split upper gates/2','Split upper gates/3'}, ...
        {'Upper A','Upper B','Upper C'}, 1, '0.0006');
    gateView.LayoutDimensions = [3 1];
    for k = 1:3
        gateView.ActiveDisplay = k;
        gateView.Title = ['Upper gate ' char('A'+k-1)];
        gateView.YLimits = [-0.2 1.2];
    end
    gateView.ActiveDisplay = 1;
    gateView.ShowLegend = false;
else
    scope(sys, 'DC link', [565 608 655 677], 'Voltage (V)', ...
        {'v dc/1'}, {'DC link'}, 1, 'Auto');
end
end

function cfg = scope(sys, name, position, label, sources, legends, decimation, span)
block(sys, 'Sinks/Scope', name, position);
cfg = get_param([sys '/' name], 'ScopeConfiguration');
cfg.NumInputPorts = num2str(numel(sources));
cfg.Position = [140 110 980 600];
cfg.AxesScaling = 'Auto';
cfg.Title = name;
cfg.YLabel = label;
cfg.ShowLegend = true;
cfg.ShowTimeAxisLabel = true;
cfg.TimeSpan = span;
cfg.TimeSpanOverrunAction = 'Scroll';
cfg.DataLoggingLimitDataPoints = true;
cfg.DataLoggingMaxPoints = '150000';
if decimation > 1
    cfg.DataLoggingDecimateData = true;
    cfg.DataLoggingDecimation = num2str(decimation);
end
for k = 1:numel(sources)
    line = wire(sys, sources{k}, sprintf('%s/%d', name, k));
    if get_param(line, 'LineParent') == -1
        set_param(line, 'Name', legends{k});
    end
end
end

function block(sys, library, name, position, varargin)
add_block(['simulink/' library], [sys '/' name], 'Position', position, varargin{:});
end

function line = wire(sys, source, target)
line = add_line(sys, source, target, 'autorouting', 'on');
end
