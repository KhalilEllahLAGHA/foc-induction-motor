function count = straighten_foc_wires(mdl)
%STRAIGHTEN_FOC_WIRES Replace diagonal wire segments with right-angle bends.
ground = getSimulinkBlockHandle([mdl '/DC ground']);
if ground ~= -1
    ports = get_param(ground, 'PortHandles');
    port = [ports.LConn ports.RConn];
    line = get_param(port, 'Line');
    if line ~= -1
        groundPoints = get_param(line, 'Points');
        xy = get_param(port, 'Position');
        shift = groundPoints(1,1) - xy(1);
        if shift ~= 0 && abs(shift) <= 10
            set_param(ground, 'Position', get_param(ground, 'Position') + [shift 0 shift 0]);
        end
    end
end

lines = find_system(mdl, 'FindAll', 'on', 'FollowLinks', 'off', ...
    'LookUnderMasks', 'all', 'Type', 'line');
count = 0;
for line = lines(:).'
    if startsWith(get_param(line, 'Parent'), [mdl '/powergui/EquivalentModel']), continue; end
    points = get_param(line, 'Points');
    if ~any(all(diff(points) ~= 0, 2)), continue; end
    set_param(line, 'Points', orthogonal_points(points));
    count = count + 1;
end
end

function route = orthogonal_points(points)
steps = diff(points);
route = points(1,:);
for k = 1:size(steps,1)
    a = points(k,:);
    b = points(k+1,:);
    if all(steps(k,:) ~= 0)
        before = 0;
        after = 0;
        if k > 1, before = axis_of(steps(k-1,:)); end
        if k < size(steps,1), after = axis_of(steps(k+1,:)); end
        if before == 1 && after == 1 && abs(b(1)-a(1)) > 1
            middle = round((a(1)+b(1))/2);
            route(end+1:end+2,:) = [middle a(2); middle b(2)]; %#ok<AGROW>
        elseif before == 2 && after == 2 && abs(b(2)-a(2)) > 1
            middle = round((a(2)+b(2))/2);
            route(end+1:end+2,:) = [a(1) middle; b(1) middle]; %#ok<AGROW>
        elseif before == 2 || (before == 0 && after == 1)
            route(end+1,:) = [a(1) b(2)]; %#ok<AGROW>
        else
            route(end+1,:) = [b(1) a(2)]; %#ok<AGROW>
        end
    end
    route(end+1,:) = b; %#ok<AGROW>
end

% Remove redundant bends along a straight run without moving either endpoint.
for k = size(route,1)-1:-1:2
    if any(all(diff(route(k-1:k+1,:)) == 0, 1))
        route(k,:) = [];
    end
end
end

function axis = axis_of(step)
axis = 0;
if step(2) == 0 && step(1) ~= 0, axis = 1; end
if step(1) == 0 && step(2) ~= 0, axis = 2; end
end
