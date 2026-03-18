clc
clear
close all

%% =========================
% 1) BUILDING COORDINATES
%% =========================
lat_buildings = [
    51.540828   % 1 Sadler's Wells
    51.541422   % 2 LCF
    51.542319   % 3 HERA
    51.542202   % 4 Templar
    51.541987   % 5 itsu
    51.541708   % 6 PRET
    51.541687   % 7 Bread Street Kitchen
];
lon_buildings = [
   -0.012031
   -0.013204
   -0.012331
   -0.010269
   -0.009258
   -0.009466
   -0.010185
];
buildingNames = {'Sadlers Wells','LCF','HERA','Templar','itsu','PRET','Bread St Kitchen'};

%% =========================
% 2) SIGNAL POINTS
%% =========================
lat_signal = [
    51.5412   % S1
    51.5411   % S2
    51.5407   % S3
    51.5414   % S4
    51.5420   % S5
    51.5422   % S6
    51.5419   % S7
    51.5422   % S8
];
lon_signal = [
   -0.0101
   -0.0107
   -0.0118
   -0.0127
   -0.0128
   -0.0120
   -0.0109
   -0.0107
];
signalNames = arrayfun(@(k) sprintf('S%d',k), 1:numel(lat_signal), 'UniformOutput', false);

%% =========================
% 3) WAITING AREAS
%% =========================
lat_wait = [
    51.5417   % W1
    51.5416   % W2
];
lon_wait = [
   -0.0097
   -0.0134
];
waitNames = {'W1','W2'};

%% =========================
% 4) LOAD GPS TRACK
%% =========================
S = load("signalpoints.mat");
T = S.Position;
vars   = lower(string(T.Properties.VariableNames));
latIdx = find(contains(vars,"lat"),1);
lonIdx = find(contains(vars,"lon"),1);
lat_track = T{:,latIdx};
lon_track = T{:,lonIdx};
valid     = ~isnan(lat_track) & ~isnan(lon_track);
lat_track = lat_track(valid);
lon_track = lon_track(valid);

%% =========================
% 5) DEFINE ROUTE
%% =========================
lat_main = [
    lat_signal(1)
    lat_signal(2)
    lat_signal(3)
    lat_buildings(1)
    lat_signal(4)
    lat_buildings(2)
    lat_wait(2)
    lat_signal(5)
    lat_buildings(3)
    lat_signal(6)
    lat_signal(7)
    lat_signal(8)
    lat_buildings(4)
    lat_wait(1)
    lat_signal(1)
];
lon_main = [
    lon_signal(1)
    lon_signal(2)
    lon_signal(3)
    lon_buildings(1)
    lon_signal(4)
    lon_buildings(2)
    lon_wait(2)
    lon_signal(5)
    lon_buildings(3)
    lon_signal(6)
    lon_signal(7)
    lon_signal(8)
    lon_buildings(4)
    lon_wait(1)
    lon_signal(1)
];

lat_spurs = [
    lat_wait(1); lat_buildings(5); lat_wait(1)
    lat_wait(1); lat_buildings(6); lat_wait(1)
    lat_wait(1); lat_buildings(7); lat_wait(1)
];
lon_spurs = [
    lon_wait(1); lon_buildings(5); lon_wait(1)
    lon_wait(1); lon_buildings(6); lon_wait(1)
    lon_wait(1); lon_buildings(7); lon_wait(1)
];

mainNodeOrder = {'S1','S2','S3','Sadlers Wells','S4','LCF','W2', ...
                 'S5','HERA','S6','S7','S8','Templar','W1','S1'};
spurNodeOrders = {
    {'W1','itsu','W1'}
    {'W1','PRET','W1'}
    {'W1','Bread St Kitchen','W1'}
};

%% =========================
% 6) MERGE FOR MAP EXTENT
%% =========================
lat_all = [lat_buildings; lat_signal(:); lat_wait(:); lat_track(:)];
lon_all = [lon_buildings; lon_signal(:); lon_wait(:); lon_track(:)];

%% =========================
% 7) GPS -> LOCAL XY (metres)
%% =========================
lat0 = lat_all(1);
lon0 = lon_all(1);
R    = 6371000;
to_xy = @(lat,lon) deal( ...
    deg2rad(lon - lon0) .* R .* cos(deg2rad(lat0)), ...
    deg2rad(lat - lat0) .* R );

[x_all,       y_all      ] = to_xy(lat_all,       lon_all);
[x_buildings, y_buildings] = to_xy(lat_buildings, lon_buildings);
[x_signal,    y_signal   ] = to_xy(lat_signal,    lon_signal);
[x_wait,      y_wait     ] = to_xy(lat_wait,      lon_wait);
[x_track,     y_track    ] = to_xy(lat_track,     lon_track);
[x_main,      y_main     ] = to_xy(lat_main,      lon_main);
[x_spurs,     y_spurs    ] = to_xy(lat_spurs,     lon_spurs);

%% =========================
% 8) SHIFT TO POSITIVE MAP FRAME
%% =========================
padding = 20;
xMin = min(x_all);
yMin = min(y_all);
shift = @(x,y) deal(x - xMin + padding, y - yMin + padding);

[x_buildings, y_buildings] = shift(x_buildings, y_buildings);
[x_signal,    y_signal   ] = shift(x_signal,    y_signal);
[x_wait,      y_wait     ] = shift(x_wait,      y_wait);
[x_track,     y_track    ] = shift(x_track,     y_track);
[x_main,      y_main     ] = shift(x_main,      y_main);
[x_spurs,     y_spurs    ] = shift(x_spurs,     y_spurs);

mapWidthMeters  = ceil(max(x_all - xMin + padding) + padding);
mapHeightMeters = ceil(max(y_all - yMin + padding) + padding);

%% =========================
% 9) GRID SETUP
%% =========================
cellSize = 2;
numCols = ceil(mapWidthMeters  / cellSize);
numRows = ceil(mapHeightMeters / cellSize);
occupancyGrid = ones(numRows, numCols);

%% =========================
% 10) ROAD HALF-WIDTH
%% =========================
roadHalfWidth = 5.0;

%% =========================
% 11) CARVE MAIN LOOP + SPURS
%% =========================
interpSpacing = cellSize / 2;
occupancyGrid = carve_path(occupancyGrid, x_main, y_main, roadHalfWidth, ...
    cellSize, numRows, numCols, interpSpacing);
occupancyGrid = carve_path(occupancyGrid, x_spurs, y_spurs, roadHalfWidth, ...
    cellSize, numRows, numCols, interpSpacing);

%% =========================
% 12) POST-PROCESSING
%% =========================
smoothR = round(3.0 / cellSize);
[se_xx, se_yy] = meshgrid(-smoothR:smoothR, -smoothR:smoothR);
SE_disk = double((se_xx.^2 + se_yy.^2) <= smoothR^2);
SE_sum  = sum(SE_disk(:));

freeMap        = double(~logical(occupancyGrid));
freeMap_dil    = conv2(freeMap,             SE_disk, 'same') > 0;
freeMap_closed = conv2(double(freeMap_dil), SE_disk, 'same') >= SE_sum;
occupancyGrid  = double(~freeMap_closed);

recarveR = roadHalfWidth * 0.7;
all_rx = [x_main(:); x_spurs(:); x_signal(:); x_wait(:)];
all_ry = [y_main(:); y_spurs(:); y_signal(:); y_wait(:)];
for i = 1:length(all_rx)
    occupancyGrid = carve_circle(occupancyGrid, all_rx(i), all_ry(i), ...
        recarveR, cellSize, numRows, numCols);
end

%% =========================
% 13) BORDER WALL
%% =========================
borderCells = ceil(padding / 2 / cellSize);
occupancyGrid(1:borderCells,       :) = 1;
occupancyGrid(end-borderCells:end, :) = 1;
occupancyGrid(:, 1:borderCells      ) = 1;
occupancyGrid(:, end-borderCells:end) = 1;

%% =========================
% 14) BUILD NODE DATA STRUCTURES
%% =========================
nodes = struct('id',{},'name',{},'type',{},'x',{},'y',{}, ...
               'lat',{},'lon',{},'row',{},'col',{});
id = 0;

for i = 1:numel(buildingNames)
    id = id + 1;
    [r,c] = xy_to_rc(x_buildings(i), y_buildings(i), cellSize, numRows, numCols);
    [r,c] = snap_to_free(occupancyGrid, r, c);
    nodes(id) = make_node(id, buildingNames{i}, 'building', ...
        x_buildings(i), y_buildings(i), lat_buildings(i), lon_buildings(i), r, c);
end

for i = 1:numel(signalNames)
    id = id + 1;
    [r,c] = xy_to_rc(x_signal(i), y_signal(i), cellSize, numRows, numCols);
    [r,c] = snap_to_free(occupancyGrid, r, c);
    nodes(id) = make_node(id, signalNames{i}, 'signal', ...
        x_signal(i), y_signal(i), lat_signal(i), lon_signal(i), r, c);
end

for i = 1:numel(waitNames)
    id = id + 1;
    [r,c] = xy_to_rc(x_wait(i), y_wait(i), cellSize, numRows, numCols);
    [r,c] = snap_to_free(occupancyGrid, r, c);
    nodes(id) = make_node(id, waitNames{i}, 'waiting', ...
        x_wait(i), y_wait(i), lat_wait(i), lon_wait(i), r, c);
end

nameToId = containers.Map('KeyType','char','ValueType','double');
for i = 1:numel(nodes)
    nameToId(nodes(i).name) = nodes(i).id;
end

%% =========================
% 15) LINKED LISTS
%% =========================
% Simple singly linked lists using node indices.
linkedLists.landmarks = build_linked_list(nodes(strcmp({nodes.type}, 'building')));
linkedLists.signals   = build_linked_list(nodes(strcmp({nodes.type}, 'signal') | strcmp({nodes.type}, 'waiting')));

%% =========================
% 16) KD-TREE FOR LOOKUP
%% =========================
landmarkIdx = find(strcmp({nodes.type}, 'building'));
signalIdx   = find(strcmp({nodes.type}, 'signal') | strcmp({nodes.type}, 'waiting'));

landmarkPts = [[nodes(landmarkIdx).x].' [nodes(landmarkIdx).y].'];
signalPts   = [[nodes(signalIdx).x].'   [nodes(signalIdx).y].'];

landmarkKD = build_kdtree(landmarkPts, landmarkIdx, 1);
signalKD   = build_kdtree(signalPts, signalIdx, 1);

%% =========================
% 17) ABSTRACT GRAPH (ADJACENCY LIST)
%% =========================
graph.N = numel(nodes);
graph.adj = cell(graph.N,1);

% Add the main loop edges
for k = 1:numel(mainNodeOrder)-1
    a = nameToId(mainNodeOrder{k});
    b = nameToId(mainNodeOrder{k+1});
    w = euclid_nodes(nodes(a), nodes(b));
    graph = add_undirected_edge(graph, a, b, w);
end

% Add the spur edges
for s = 1:numel(spurNodeOrders)
    seq = spurNodeOrders{s};
    for k = 1:numel(seq)-1
        a = nameToId(seq{k});
        b = nameToId(seq{k+1});
        w = euclid_nodes(nodes(a), nodes(b));
        graph = add_undirected_edge(graph, a, b, w);
    end
end

% connect waiting points directly to their closest signal if absent(just in case)
graph = ensure_wait_connections(graph, nodes, nameToId);

%% =========================
% 18) VISUALIZE GRID + LANDMARKS
%% =========================
figure('Name','Occupancy Grid + Landmarks')
imagesc([0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid))
axis equal tight
clim([0 1])
colormap(gray)
hold on

plot(x_main,  mapHeightMeters - y_main,  'm-',  'LineWidth',1.5)
plot(x_spurs, mapHeightMeters - y_spurs, 'm--', 'LineWidth',1.2)

scatter([nodes(strcmp({nodes.type},'building')).x], ...
        mapHeightMeters - [nodes(strcmp({nodes.type},'building')).y], ...
        90, 'r', 'filled', 'MarkerEdgeColor','k');
scatter([nodes(strcmp({nodes.type},'signal')).x], ...
        mapHeightMeters - [nodes(strcmp({nodes.type},'signal')).y], ...
        90, 'b', 'd', 'filled', 'MarkerEdgeColor','k');
scatter([nodes(strcmp({nodes.type},'waiting')).x], ...
        mapHeightMeters - [nodes(strcmp({nodes.type},'waiting')).y], ...
        120, 'g', 's', 'filled', 'MarkerEdgeColor','k');

for i = 1:numel(nodes)
    if strcmp(nodes(i).type,'building')
        c = [0.85 0 0];
    elseif strcmp(nodes(i).type,'signal')
        c = [0 0 1];
    else
        c = [0 0.55 0];
    end
    text(nodes(i).x + 1.5, mapHeightMeters - nodes(i).y, nodes(i).name, ...
        'FontSize',7, 'FontWeight','bold', 'Color',c, 'Interpreter','none');
end

title('Occupancy Grid with Named Nodes')
xlabel('X (m)')
ylabel('Y (m)')
grid on
hold off
function out = answer_hri_query(q, nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize)

    out = struct();
    out.type = q.type;
    out.summary = '';
    out.nodeSequence = {};
    out.graphCost = [];
    out.gridDistance = [];
    out.extra = struct();

    switch q.type

        case 'route'
            route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, false, "dijkstra_heap");
            walk  = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

            out.summary = sprintf('Shortest route from %s to %s.', q.start, q.goal);
            out.nodeSequence = route.nodeNames;
            out.graphCost = route.totalCost;
            out.gridDistance = walk.totalDistanceMeters;

        case 'guide_landmark'
            if ~isfield(q, 'returnToClosestWait')
                q.returnToClosestWait = true;
            end

            route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, q.returnToClosestWait, "dijkstra_heap");
            walk  = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

            out.summary = sprintf('Guide request from %s to %s (return=%d).', ...
                q.start, q.goal, q.returnToClosestWait);
            out.nodeSequence = route.nodeNames;
            out.graphCost = route.totalCost;
            out.gridDistance = walk.totalDistanceMeters;

        case 'distance'
            route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, false, "dijkstra_heap");
            walk  = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

            out.summary = sprintf('Distance query from %s to %s.', q.start, q.goal);
            out.nodeSequence = route.nodeNames;
            out.graphCost = route.totalCost;
            out.gridDistance = walk.totalDistanceMeters;

        case 'nearest_landmark'
            queryXY = resolve_query_xy(q, nodes, nameToId);

            [idxList, distList] = linked_list_nearest(linkedLists.landmarks, queryXY, nodes);
            [idxKD, distKD]     = kd_nearest(landmarkKD, queryXY, nodes, inf, -1);

            out.summary = 'Nearest landmark query using linked list and KD-tree.';
            out.extra.queryXY = queryXY;
            out.extra.linkedListName = nodes(idxList).name;
            out.extra.linkedListDist = distList;
            out.extra.kdName = nodes(idxKD).name;
            out.extra.kdDist = distKD;

        case 'k_nearest_landmarks'
            queryXY = resolve_query_xy(q, nodes, nameToId);
            if ~isfield(q, 'k')
                q.k = 3;
            end

            landmarkIds = find(strcmp({nodes.type}, 'building'));
            [idsSorted, distsSorted] = sort_candidates_by_distance(queryXY, landmarkIds, nodes);

            k = min(q.k, numel(idsSorted));
            idsSorted = idsSorted(1:k);
            distsSorted = distsSorted(1:k);

            out.summary = sprintf('%d nearest landmarks to query point.', k);
            out.extra.queryXY = queryXY;
            out.extra.nearestNames = {nodes(idsSorted).name};
            out.extra.nearestDists = distsSorted;

        case 'nearest_signal'
            queryXY = resolve_query_xy(q, nodes, nameToId);

            signalOnlyIds = find(strcmp({nodes.type}, 'signal'));
            [idsSorted, distsSorted] = sort_candidates_by_distance(queryXY, signalOnlyIds, nodes);

            out.summary = 'Nearest signal point query.';
            out.extra.queryXY = queryXY;
            out.extra.signalName = nodes(idsSorted(1)).name;
            out.extra.signalDist = distsSorted(1);

        case 'constrained_route'
            if ~isfield(q, 'requiredSignals')
                q.requiredSignals = {};
            end
            if ~isfield(q, 'returnToClosestWait')
                q.returnToClosestWait = true;
            end

            route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, q.requiredSignals, q.returnToClosestWait, "dijkstra_heap");
            walk  = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

            out.summary = sprintf('Constrained route from %s to %s via required signals.', q.start, q.goal);
            out.nodeSequence = route.nodeNames;
            out.graphCost = route.totalCost;
            out.gridDistance = walk.totalDistanceMeters;
            out.extra.requiredSignals = q.requiredSignals;

        case 'full_tour'
            tour = full_tourist_guide_mode(graph, nodes, nameToId, q.startWait, "dijkstra_heap");
            walk = realize_graph_route_on_grid(tour.fullRoute, graph, nodes, occupancyGrid, cellSize);

            out.summary = sprintf('Full tourist-guide mode starting at %s.', q.startWait);
            out.nodeSequence = tour.fullRoute.nodeNames;
            out.graphCost = tour.totalCost;
            out.gridDistance = walk.totalDistanceMeters;
            out.extra.visitOrder = tour.visitNames;

        case 'scheduler_demo'
            queue = [];
            serialCounter = 0;

            serialCounter = serialCounter + 1;
            queue = scheduler_push(queue, make_task('Guide HERA', 3, serialCounter, ...
                struct('type','guide_landmark','start','W1','goal','HERA','returnToClosestWait',true)));

            serialCounter = serialCounter + 1;
            queue = scheduler_push(queue, make_task('Distance to LCF', 4, serialCounter, ...
                struct('type','distance','start','W2','goal','LCF')));

            serialCounter = serialCounter + 1;
            queue = scheduler_push(queue, make_task('Emergency return to W2', 1, serialCounter, ...
                struct('type','route','start','Templar','goal','W2')));

            processed = {};
            while ~isempty(queue)
                [task, queue] = scheduler_pop(queue);
                result = answer_hri_query(task.payload, nodes, graph, nameToId, ...
                    linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize);

                processed{end+1} = sprintf('[Priority %d] %s -> %s', ...
                    task.priority, task.name, result.summary); %#ok<AGROW>
            end

            out.summary = 'Task scheduling demo with interrupt priorities.';
            out.extra.processedOrder = processed;

        otherwise
            error('Unknown query type: %s', q.type);
    end
end

function print_hri_answer(out)
    fprintf('%s\n', out.summary);

    if ~isempty(out.nodeSequence)
        fprintf('Node sequence: %s\n', strjoin(out.nodeSequence, ' -> '));
    end

    if ~isempty(out.graphCost)
        fprintf('Graph cost: %.2f m\n', out.graphCost);
    end

    if ~isempty(out.gridDistance)
        fprintf('Grid walking distance: %.2f m\n', out.gridDistance);
    end

    switch out.type
        case 'nearest_landmark'
            fprintf('Query XY: (%.2f, %.2f)\n', out.extra.queryXY(1), out.extra.queryXY(2));
            fprintf('Linked list nearest landmark: %s (%.2f m)\n', ...
                out.extra.linkedListName, out.extra.linkedListDist);
            fprintf('KD-tree nearest landmark    : %s (%.2f m)\n', ...
                out.extra.kdName, out.extra.kdDist);

        case 'k_nearest_landmarks'
            fprintf('Query XY: (%.2f, %.2f)\n', out.extra.queryXY(1), out.extra.queryXY(2));
            for i = 1:numel(out.extra.nearestNames)
                fprintf('  %d) %s (%.2f m)\n', i, out.extra.nearestNames{i}, out.extra.nearestDists(i));
            end

        case 'nearest_signal'
            fprintf('Query XY: (%.2f, %.2f)\n', out.extra.queryXY(1), out.extra.queryXY(2));
            fprintf('Nearest signal: %s (%.2f m)\n', ...
                out.extra.signalName, out.extra.signalDist);

        case 'constrained_route'
            if isfield(out.extra, 'requiredSignals')
                fprintf('Required signals: %s\n', strjoin(out.extra.requiredSignals, ', '));
            end

        case 'full_tour'
            fprintf('Visit order: %s\n', strjoin(out.extra.visitOrder, ' -> '));

        case 'scheduler_demo'
            fprintf('Processed task order:\n');
            for i = 1:numel(out.extra.processedOrder)
                fprintf('  %s\n', out.extra.processedOrder{i});
            end
    end
end

function queryXY = resolve_query_xy(q, nodes, nameToId)
    if isfield(q, 'queryMode') && strcmp(q.queryMode, 'xy')
        queryXY = q.xy;
        return
    end

    if isfield(q, 'pointName')
        nid = nameToId(q.pointName);
        queryXY = [nodes(nid).x, nodes(nid).y];
        return
    end

    error('Could not resolve query position.');
end

function [idsSorted, distsSorted] = sort_candidates_by_distance(queryXY, candidateIds, nodes)
    d = zeros(1, numel(candidateIds));
    for i = 1:numel(candidateIds)
        nid = candidateIds(i);
        d(i) = hypot(nodes(nid).x - queryXY(1), nodes(nid).y - queryXY(2));
    end
    [distsSorted, ord] = sort(d, 'ascend');
    idsSorted = candidateIds(ord);
end

function task = make_task(name, priority, serialNo, payload)
    task.name = name;
    task.priority = priority;
    task.serialNo = serialNo;
    task.payload = payload;
end

function queue = scheduler_push(queue, task)
    if isempty(queue)
        queue = task;
        return
    end
    queue(end+1) = task;
    queue = scheduler_sort(queue);
end

function [task, queue] = scheduler_pop(queue)
    queue = scheduler_sort(queue);
    task = queue(1);
    if numel(queue) == 1
        queue = [];
    else
        queue = queue(2:end);
    end
end

function queue = scheduler_sort(queue)
    pri = [queue.priority];
    ser = [queue.serialNo];
    mat = [(1:numel(queue))' pri(:) ser(:)];
    mat = sortrows(mat, [2 3]);   % lower priority number first, then FIFO
    queue = queue(mat(:,1));
end

function out = speculative_dialogue_demo(dialogue, defaultStartName, nodes, graph, nameToId, occupancyGrid, cellSize)
    out = struct();
    out.startName = defaultStartName;
    out.timeline = {};

    currentStart = defaultStartName;
    currentGoal  = '';

    candidateNames = {nodes.name};

    for i = 1:size(dialogue,1)
        t = dialogue{i,1};
        utt = dialogue{i,2};

        [detectedStart, detectedGoal] = parse_dialogue_intent(utt, candidateNames);

        if ~isempty(detectedStart)
            currentStart = detectedStart;
        end
        if ~isempty(detectedGoal)
            currentGoal = detectedGoal;
        end

        entry.time = t;
        entry.utterance = utt;
        entry.predictedStart = currentStart;
        entry.predictedGoal = currentGoal;

        if ~isempty(currentGoal) && isKey(nameToId, currentStart) && isKey(nameToId, currentGoal)
            route = plan_graph_route(graph, nodes, nameToId, currentStart, currentGoal, {}, false, "dijkstra_heap");
            walk  = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

            entry.routeNames = route.nodeNames;
            entry.graphCost = route.totalCost;
            entry.gridDistance = walk.totalDistanceMeters;
        else
            entry.routeNames = {};
            entry.graphCost = [];
            entry.gridDistance = [];
        end

        out.timeline{end+1} = entry; %#ok<AGROW>
    end
end

function [startName, goalName] = parse_dialogue_intent(sentence, candidateNames)
    s = lower(sentence);
    startName = '';
    goalName  = '';

    foundNames = {};
    foundPos   = [];

    for i = 1:numel(candidateNames)
        cand = candidateNames{i};
        pos = strfind(s, lower(cand));
        if ~isempty(pos)
            foundNames{end+1} = cand; %#ok<AGROW>
            foundPos(end+1) = pos(1); %#ok<AGROW>
        end
    end

    if isempty(foundNames)
        return
    end

    [~, ord] = sort(foundPos);
    foundNames = foundNames(ord);
    foundPos   = foundPos(ord);

    goalKeywords = {'go to', 'head to', 'move to', 'visit', 'to '};
    startKeywords = {'from ', 'start from'};

    for i = 1:numel(foundNames)
        nm = foundNames{i};
        pos = foundPos(i);
        leftContext = s(max(1,pos-20):pos);

        for k = 1:numel(startKeywords)
            if contains(leftContext, startKeywords{k})
                startName = nm;
            end
        end

        for k = 1:numel(goalKeywords)
            if contains(leftContext, goalKeywords{k})
                goalName = nm;
            end
        end
    end

    if isempty(goalName)
        if contains(s, 'go') || contains(s, 'head') || contains(s, 'visit') || contains(s, 'move')
            goalName = foundNames{end};
        end
    end
end

function names = extract_named_nodes_from_sentence(sentence, candidateNames)
    s = lower(sentence);
    names = {};
    for i = 1:numel(candidateNames)
        cand = candidateNames{i};
        if contains(s, lower(cand))
            names{end+1} = cand; %#ok<AGROW>
        end
    end
end

function print_speculative_answer(out)
    fprintf('Speculative dialogue start node: %s\n', out.startName);
    for i = 1:numel(out.timeline)
        e = out.timeline{i};
        fprintf('\n[t = %.1fs] "%s"\n', e.time, e.utterance);
        if isempty(e.predictedGoal)
            fprintf('Predicted goal: <none>\n');
        else
            fprintf('Predicted goal: %s\n', e.predictedGoal);
        end

        if ~isempty(e.routeNames)
            fprintf('Predicted route: %s\n', strjoin(e.routeNames, ' -> '));
            fprintf('Predicted graph cost: %.2f m\n', e.graphCost);
            fprintf('Predicted grid distance: %.2f m\n', e.gridDistance);
        else
            fprintf('No valid route prediction yet.\n');
        end
    end
end
%% =========================
% 19) PART 3 — GUI INTERFACE
%% =========================
fprintf('\nLaunching Task 3 GUI...\n');

create_task3_gui(nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, ...
    occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters);

function node = make_node(id, name, type, x, y, lat, lon, row, col)
    node.id   = id;
    node.name = name;
    node.type = type;
    node.x    = x;
    node.y    = y;
    node.lat  = lat;
    node.lon  = lon;
    node.row  = row;
    node.col  = col;
end

function occupancyGrid = carve_circle(occupancyGrid, cx, cy, r, cellSize, numRows, numCols)
    colC = round(cx / cellSize) + 1;
    rowC = round(cy / cellSize) + 1;
    rC   = ceil(r / cellSize);
    for rr = max(1,rowC-rC) : min(numRows,rowC+rC)
        for cc = max(1,colC-rC) : min(numCols,colC+rC)
            xCell = (cc-1)*cellSize;
            yCell = (rr-1)*cellSize;
            if (xCell-cx)^2 + (yCell-cy)^2 <= r^2
                occupancyGrid(rr,cc) = 0;
            end
        end
    end
end

function occupancyGrid = carve_path(occupancyGrid, xs_path, ys_path, r, cellSize, numRows, numCols, interpSpacing)
    for seg = 1:length(xs_path)-1
        x1 = xs_path(seg);   y1 = ys_path(seg);
        x2 = xs_path(seg+1); y2 = ys_path(seg+1);
        segLen = hypot(x2-x1, y2-y1);
        nPts   = max(2, ceil(segLen/interpSpacing));
        xs = linspace(x1, x2, nPts);
        ys = linspace(y1, y2, nPts);
        for k = 1:nPts
            occupancyGrid = carve_circle(occupancyGrid, xs(k), ys(k), r, cellSize, numRows, numCols);
        end
    end
end

function [row,col] = xy_to_rc(x, y, cellSize, numRows, numCols)
    col = min(max(round(x / cellSize) + 1, 1), numCols);
    row = min(max(round(y / cellSize) + 1, 1), numRows);
end

function [row,col] = snap_to_free(occ, row, col)
    if occ(row,col) == 0
        return
    end

    [numRows, numCols] = size(occ);
    maxR = max(numRows, numCols);
    for rad = 1:maxR
        r1 = max(1, row-rad); r2 = min(numRows, row+rad);
        c1 = max(1, col-rad); c2 = min(numCols, col+rad);
        for rr = r1:r2
            for cc = c1:c2
                if occ(rr,cc)==0
                    row = rr; col = cc;
                    return
                end
            end
        end
    end
    error('Could not find a free cell near the requested node.');
end

function LL = build_linked_list(nodeSubset)
    LL.head = 1;
    LL.data = struct('nodeId',{},'next',{});
    for i = 1:numel(nodeSubset)
        LL.data(i).nodeId = nodeSubset(i).id;
        if i < numel(nodeSubset)
            LL.data(i).next = i + 1;
        else
            LL.data(i).next = 0;
        end
    end
end

function [bestId, bestDist] = linked_list_nearest(LL, queryXY, allNodes)
    ptr = LL.head;
    bestId = -1;
    bestDist = inf;
    while ptr ~= 0
        nid = LL.data(ptr).nodeId;
        d = hypot(allNodes(nid).x - queryXY(1), allNodes(nid).y - queryXY(2));
        if d < bestDist
            bestDist = d;
            bestId = nid;
        end
        ptr = LL.data(ptr).next;
    end
end

function tree = build_kdtree(pointsXY, pointIds, depth)
    if isempty(pointIds)
        tree = [];
        return
    end
    axis = mod(depth-1, 2) + 1;
    [~, ord] = sort(pointsXY(:,axis));
    pointsXY = pointsXY(ord,:);
    pointIds = pointIds(ord);
    mid = ceil(size(pointsXY,1)/2);

    tree.axis = axis;
    tree.point = pointsXY(mid,:);
    tree.nodeId = pointIds(mid);
    tree.left = build_kdtree(pointsXY(1:mid-1,:), pointIds(1:mid-1), depth+1);
    tree.right = build_kdtree(pointsXY(mid+1:end,:), pointIds(mid+1:end), depth+1);
end

function [bestId, bestDist] = kd_nearest(tree, queryXY, allNodes, bestDist, bestId)
    if isempty(tree)
        return
    end

    currDist = hypot(allNodes(tree.nodeId).x - queryXY(1), allNodes(tree.nodeId).y - queryXY(2));
    if currDist < bestDist
        bestDist = currDist;
        bestId = tree.nodeId;
    end

    axis = tree.axis;
    if queryXY(axis) < tree.point(axis)
        nearBranch = tree.left;
        farBranch  = tree.right;
    else
        nearBranch = tree.right;
        farBranch  = tree.left;
    end

    [bestId, bestDist] = kd_nearest(nearBranch, queryXY, allNodes, bestDist, bestId);

    if abs(queryXY(axis) - tree.point(axis)) < bestDist
        [bestId, bestDist] = kd_nearest(farBranch, queryXY, allNodes, bestDist, bestId);
    end
end

function d = euclid_nodes(a, b)
    d = hypot(a.x - b.x, a.y - b.y);
end

function graph = add_undirected_edge(graph, u, v, w)
    if ~has_edge(graph, u, v)
        graph.adj{u}(end+1).to = v;
        graph.adj{u}(end).weight = w;
    end
    if ~has_edge(graph, v, u)
        graph.adj{v}(end+1).to = u;
        graph.adj{v}(end).weight = w;
    end
end

function yes = has_edge(graph, u, v)
    yes = false;
    for k = 1:numel(graph.adj{u})
        if graph.adj{u}(k).to == v
            yes = true;
            return
        end
    end
end

function graph = ensure_wait_connections(graph, nodes, nameToId)
    waitIds = [nameToId('W1'), nameToId('W2')];
    sigIds = find(strcmp({nodes.type}, 'signal'));

    for i = 1:numel(waitIds)
        w = waitIds(i);

        hasSignalEdge = false;
        for k = 1:numel(graph.adj{w})
            v = graph.adj{w}(k).to;
            if strcmp(nodes(v).type, 'signal')
                hasSignalEdge = true;
                break
            end
        end

        if ~hasSignalEdge
            best = -1;
            bestD = inf;
            for j = 1:numel(sigIds)
                s = sigIds(j);
                d = euclid_nodes(nodes(w), nodes(s));
                if d < bestD
                    bestD = d;
                    best = s;
                end
            end
            graph = add_undirected_edge(graph, w, best, bestD);
        end
    end
end

function out = plan_graph_route(graph, nodes, nameToId, startName, goalName, requiredSignals, returnToClosestWait, method)
    if nargin < 7
        returnToClosestWait = false;
    end

    startId = nameToId(char(startName));
    goalId  = nameToId(char(goalName));

    waypoints = startId;
    for i = 1:numel(requiredSignals)
        waypoints(end+1) = nameToId(char(requiredSignals{i})); 
    end
    waypoints(end+1) = goalId;

    if returnToClosestWait
        waitIds = find(strcmp({nodes.type}, 'waiting'));
        bestWait = waitIds(1);
        bestDist = inf;
        for i = 1:numel(waitIds)
            switch method
                case "dijkstra_plain"
                    r = dijkstra_plain(graph, goalId, waitIds(i));
                otherwise
                    r = dijkstra_heap(graph, goalId, waitIds(i));
            end
            if r.dist(waitIds(i)) < bestDist
                bestDist = r.dist(waitIds(i));
                bestWait = waitIds(i);
            end
        end
        waypoints(end+1) = bestWait;
    end

    fullIds = [];
    totalCost = 0;
    for k = 1:numel(waypoints)-1
        src = waypoints(k);
        dst = waypoints(k+1);
        switch method
            case "dijkstra_plain"
                r = dijkstra_plain(graph, src, dst);
            otherwise
                r = dijkstra_heap(graph, src, dst);
        end
        segIds = reconstruct_path(r.parent, src, dst);
        if isempty(segIds)
            error('No abstract-graph route from %s to %s.', nodes(src).name, nodes(dst).name);
        end
        if isempty(fullIds)
            fullIds = segIds;
        else
            fullIds = [fullIds, segIds(2:end)]; 
        end
        totalCost = totalCost + r.dist(dst);
    end

    out.nodeIds = fullIds;
    out.nodeNames = {nodes(fullIds).name};
    out.totalCost = totalCost;
end

function result = dijkstra_plain(graph, src, dst)
    N = graph.N;
    dist = inf(1,N);
    parent = zeros(1,N);
    visited = false(1,N);
    dist(src) = 0;

    for iter = 1:N
        u = -1;
        best = inf;
        for i = 1:N
            if ~visited(i) && dist(i) < best
                best = dist(i);
                u = i;
            end
        end

        if u == -1
            break
        end
        if nargin >= 3 && u == dst
            break
        end

        visited(u) = true;
        nbrs = graph.adj{u};
        for k = 1:numel(nbrs)
            v = nbrs(k).to;
            alt = dist(u) + nbrs(k).weight;
            if alt < dist(v)
                dist(v) = alt;
                parent(v) = u;
            end
        end
    end

    result.dist = dist;
    result.parent = parent;
end

function result = dijkstra_heap(graph, src, dst)
    N = graph.N;
    dist = inf(1,N);
    parent = zeros(1,N);
    dist(src) = 0;

    heap = heap_init();
    heap = heap_push(heap, src, 0);

    while heap.size > 0
        [heap, u, du] = heap_pop(heap);
        if du > dist(u)
            continue
        end
        if nargin >= 3 && u == dst
            break
        end

        nbrs = graph.adj{u};
        for k = 1:numel(nbrs)
            v = nbrs(k).to;
            alt = dist(u) + nbrs(k).weight;
            if alt < dist(v)
                dist(v) = alt;
                parent(v) = u;
                heap = heap_push(heap, v, alt);
            end
        end
    end

    result.dist = dist;
    result.parent = parent;
end

function path = reconstruct_path(parent, src, dst)
    if src == dst
        path = src;
        return
    end
    if parent(dst) == 0
        path = [];
        return
    end

    path = dst;
    curr = dst;
    while curr ~= src
        curr = parent(curr);
        if curr == 0
            path = [];
            return
        end
        path = [curr, path]; 
    end
end

function heap = heap_init()
    heap.nodes = zeros(1,128);
    heap.keys  = zeros(1,128);
    heap.size  = 0;
end

function heap = heap_push(heap, node, key)
    if heap.size + 1 > numel(heap.nodes)
        heap.nodes = [heap.nodes zeros(1,numel(heap.nodes))];
        heap.keys  = [heap.keys  zeros(1,numel(heap.keys))];
    end

    heap.size = heap.size + 1;
    i = heap.size;
    heap.nodes(i) = node;
    heap.keys(i)  = key;

    while i > 1
        p = floor(i/2);
        if heap.keys(p) <= heap.keys(i)
            break
        end
        [heap.nodes(p), heap.nodes(i)] = deal(heap.nodes(i), heap.nodes(p));
        [heap.keys(p),  heap.keys(i)]  = deal(heap.keys(i),  heap.keys(p));
        i = p;
    end
end

function [heap, node, key] = heap_pop(heap)
    if heap.size == 0
        error('Cannot pop from empty heap.')
    end

    node = heap.nodes(1);
    key  = heap.keys(1);

    heap.nodes(1) = heap.nodes(heap.size);
    heap.keys(1)  = heap.keys(heap.size);
    heap.size = heap.size - 1;

    i = 1;
    while true
        l = 2*i;
        r = 2*i + 1;
        smallest = i;

        if l <= heap.size && heap.keys(l) < heap.keys(smallest)
            smallest = l;
        end
        if r <= heap.size && heap.keys(r) < heap.keys(smallest)
            smallest = r;
        end
        if smallest == i
            break
        end

        [heap.nodes(i), heap.nodes(smallest)] = deal(heap.nodes(smallest), heap.nodes(i));
        [heap.keys(i),  heap.keys(smallest)]  = deal(heap.keys(smallest),  heap.keys(i));
        i = smallest;
    end
end

function walkPlan = realize_graph_route_on_grid(route, graph, nodes, occ, cellSize)
    nodeIds = route.nodeIds;
    legPaths = cell(0,1);
    fullRC = [];
    totalMeters = 0;

    for i = 1:numel(nodeIds)-1
        src = nodeIds(i);
        dst = nodeIds(i+1);

        [legRC, legDist] = bfs_grid(occ, [nodes(src).row nodes(src).col], [nodes(dst).row nodes(dst).col], cellSize);
        if isempty(legRC)
            error('BFS failed on occupancy grid between %s and %s.', nodes(src).name, nodes(dst).name);
        end

        legPaths{end+1} = legRC; 
        if isempty(fullRC)
            fullRC = legRC;
        else
            fullRC = [fullRC; legRC(2:end,:)]; 
        end
        totalMeters = totalMeters + legDist;
    end

    xyPath = rc_path_to_xy(fullRC, cellSize);
    walkPlan.graphNodeIds = nodeIds;
    walkPlan.legPaths = legPaths;
    walkPlan.rcPath = fullRC;
    walkPlan.xyPath = xyPath;
    walkPlan.totalDistanceMeters = totalMeters;
end

function [pathRC, distMeters] = bfs_grid(occ, startRC, goalRC, cellSize)
    [numRows, numCols] = size(occ);
    sr = startRC(1); sc = startRC(2);
    gr = goalRC(1);  gc = goalRC(2);

    if occ(sr,sc) ~= 0 || occ(gr,gc) ~= 0
        pathRC = [];
        distMeters = inf;
        return
    end

    N = numRows * numCols;
    visited = false(numRows, numCols);
    parent = zeros(N,1);

    q = zeros(N,1);
    head = 1;
    tail = 1;

    sidx = sub2ind([numRows, numCols], sr, sc);
    gidx = sub2ind([numRows, numCols], gr, gc);

    q(tail) = sidx;
    visited(sr,sc) = true;

    moves = [
        -1  0
         1  0
         0 -1
         0  1
        -1 -1
        -1  1
         1 -1
         1  1
    ];

    found = false;
    while head <= tail
        u = q(head);
        head = head + 1;

        if u == gidx
            found = true;
            break
        end

        [ur, uc] = ind2sub([numRows, numCols], u);

        for k = 1:size(moves,1)
            vr = ur + moves(k,1);
            vc = uc + moves(k,2);

            if vr < 1 || vr > numRows || vc < 1 || vc > numCols
                continue
            end
            if visited(vr,vc) || occ(vr,vc) ~= 0
                continue
            end

            if abs(moves(k,1)) == 1 && abs(moves(k,2)) == 1 
                if occ(ur,vc) ~= 0 || occ(vr,uc) ~= 0
                    continue
                end
            end

            vidx = sub2ind([numRows, numCols], vr, vc);
            visited(vr,vc) = true;
            parent(vidx) = u;

            tail = tail + 1;
            q(tail) = vidx;
        end
    end

    if ~found
        pathRC = [];
        distMeters = inf;
        return
    end

    rev = gidx;
    curr = gidx;
    while curr ~= sidx
        curr = parent(curr);
        rev(end+1) = curr; 
    end
    rev = fliplr(rev);

    pathRC = zeros(numel(rev), 2);
    for i = 1:numel(rev)
        [r,c] = ind2sub([numRows, numCols], rev(i));
        pathRC(i,:) = [r,c];
    end

    distMeters = 0;
    for i = 1:size(pathRC,1)-1
        dr = abs(pathRC(i+1,1) - pathRC(i,1));
        dc = abs(pathRC(i+1,2) - pathRC(i,2));
        if dr == 1 && dc == 1
            distMeters = distMeters + sqrt(2) * cellSize;
        else
            distMeters = distMeters + cellSize;
        end
    end
end

function xy = rc_path_to_xy(pathRC, cellSize)
    if isempty(pathRC)
        xy = zeros(0,2);
        return
    end
    xy = [(pathRC(:,2)-1)*cellSize, (pathRC(:,1)-1)*cellSize];
end

function plot_graph_edges(nodes, graph, mapHeightMeters)
    for u = 1:graph.N
        for k = 1:numel(graph.adj{u})
            v = graph.adj{u}(k).to;
            if v > u
                plot([nodes(u).x nodes(v).x], ...
                     mapHeightMeters - [nodes(u).y nodes(v).y], ...
                     'y--', 'LineWidth', 1.0);
            end
        end
    end
end

function plot_grid_path(xyPath, mapHeightMeters, styleStr, lineW)
    if isempty(xyPath)
        return
    end
    plot(xyPath(:,1), mapHeightMeters - xyPath(:,2), styleStr, 'LineWidth', lineW);
end

function tour = full_tourist_guide_mode(graph, nodes, nameToId, startWaitName, method)
    waitId = nameToId(char(startWaitName));
    buildingIds = find(strcmp({nodes.type}, 'building'));

    relevant = [waitId, buildingIds]; 
    M = numel(relevant);
    costMat = inf(M,M); 
    pathCell = cell(M,M); 


    for i = 1:M
        for j = 1:M
            if i == j
                costMat(i,j) = 0;
                pathCell{i,j} = relevant(i);
            else
                switch method
                    case "dijkstra_plain"
                        r = dijkstra_plain(graph, relevant(i), relevant(j));
                    otherwise
                        r = dijkstra_heap(graph, relevant(i), relevant(j));
                end
                costMat(i,j) = r.dist(relevant(j));
                pathCell{i,j} = reconstruct_path(r.parent, relevant(i), relevant(j));
            end
        end
    end

    permsIdx = perms(2:M);  
    bestCost = inf;
    bestOrder = [];

    for r = 1:size(permsIdx,1)
        order = [1, permsIdx(r,:), 1];
        c = 0;
        feasible = true;
        for k = 1:numel(order)-1
            cc = costMat(order(k), order(k+1));
            if isinf(cc)
                feasible = false;
                break
            end
            c = c + cc;
        end
        if feasible && c < bestCost
            bestCost = c;
            bestOrder = order;
        end
    end

    if isempty(bestOrder)
        error('No feasible full-tour route found.');
    end

    fullRouteIds = [];
    visitNodeIds = relevant(bestOrder);
    for k = 1:numel(bestOrder)-1
        seg = pathCell{bestOrder(k), bestOrder(k+1)};
        if isempty(seg)
            error('A segment in tourist-guide mode is infeasible.');
        end
        if isempty(fullRouteIds)
            fullRouteIds = seg;
        else
            fullRouteIds = [fullRouteIds, seg(2:end)]; 
        end
    end

    tour.visitNodeIds = visitNodeIds;
    tour.visitNames = {nodes(visitNodeIds).name};
    tour.totalCost = bestCost;
    tour.fullRoute.nodeIds = fullRouteIds;
    tour.fullRoute.nodeNames = {nodes(fullRouteIds).name};
    tour.fullRoute.totalCost = bestCost;
end
function create_task3_gui(nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, ...
    occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters)

    allNames = {nodes.name};

    fig = uifigure('Name','Task 3 - Human Robot Interaction GUI', ...
        'Position',[80 80 1450 820]);

    gl = uigridlayout(fig,[1 2]);
    gl.ColumnWidth = {360, '1x'};

    leftPanel = uipanel(gl,'Title','Controls');
    leftGrid = uigridlayout(leftPanel,[17 2]);
    leftGrid.RowHeight = {22,22,22,22,22,22,22,22,22,22,28,28,28,28,120,'1x',30};
    leftGrid.ColumnWidth = {120,'1x'};

    uilabel(leftGrid,'Text','Query type:');
    ddType = uidropdown(leftGrid, ...
        'Items',{'route','guide_landmark','distance','nearest_landmark', ...
                 'k_nearest_landmarks','nearest_signal','constrained_route', ...
                 'full_tour','scheduler_demo','speculative_dialogue_demo'}, ...
        'Value','route');

    uilabel(leftGrid,'Text','Start point:');
    ddStart = uidropdown(leftGrid,'Items',allNames,'Value','W1');

    uilabel(leftGrid,'Text','Goal point:');
    ddGoal = uidropdown(leftGrid,'Items',allNames,'Value','LCF');

    uilabel(leftGrid,'Text','Query point:');
    ddPoint = uidropdown(leftGrid,'Items',allNames,'Value','W1');

    uilabel(leftGrid,'Text','k value:');
    efK = uieditfield(leftGrid,'numeric','Value',3,'Limits',[1 20],'RoundFractionalValues','on');

    uilabel(leftGrid,'Text','Required signals:');
    efSignals = uieditfield(leftGrid,'text','Value','S5,S6');

    uilabel(leftGrid,'Text','Return to wait:');
    cbReturn = uicheckbox(leftGrid,'Value',true,'Text','');

    uilabel(leftGrid,'Text','Preset dialogue:');
    taDialogue = uitextarea(leftGrid, ...
        'Value', {'Guide me from W2 to Templar via S5 and S6'});

    btnRun = uibutton(leftGrid,'Text','Run Query');
    btnRun.Layout.Row = 16;
    btnRun.Layout.Column = [1 2];

    btnClear = uibutton(leftGrid,'Text','Clear Output');
    btnClear.Layout.Row = 17;
    btnClear.Layout.Column = [1 2];

    rightPanel = uipanel(gl,'Title','Result & Map');
    rightGrid = uigridlayout(rightPanel,[2 1]);
    rightGrid.RowHeight = {220,'1x'};

    taResult = uitextarea(rightGrid,'Editable','off');
    ax = uiaxes(rightGrid);
    title(ax,'Map View');
    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');

    gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);

    btnRun.ButtonPushedFcn = @(src,event) gui_run_query( ...
        ddType, ddStart, ddGoal, ddPoint, efK, efSignals, cbReturn, taDialogue, ...
        taResult, ax, nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, ...
        occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters);

    btnClear.ButtonPushedFcn = @(src,event) gui_clear_output( ...
        taResult, ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);

    fig.WindowKeyReleaseFcn = @(src,event) gui_live_nlp_update( ...
        ddType, ddStart, taDialogue, taResult, ax, nodes, graph, nameToId, ...
        linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize, ...
        mapWidthMeters, mapHeightMeters);

    ddType.ValueChangedFcn = @(src,event) gui_live_nlp_update( ...
        ddType, ddStart, taDialogue, taResult, ax, nodes, graph, nameToId, ...
        linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize, ...
        mapWidthMeters, mapHeightMeters);

    ddStart.ValueChangedFcn = @(src,event) gui_live_nlp_update( ...
        ddType, ddStart, taDialogue, taResult, ax, nodes, graph, nameToId, ...
        linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize, ...
        mapWidthMeters, mapHeightMeters);
end

function gui_run_query(ddType, ddStart, ddGoal, ddPoint, efK, efSignals, cbReturn, taDialogue, ...
    taResult, ax, nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, ...
    occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters)

    try
        q = struct();
        q.type = ddType.Value;

        switch q.type
            case 'route'
                q.start = ddStart.Value;
                q.goal  = ddGoal.Value;

            case 'guide_landmark'
                q.start = ddStart.Value;
                q.goal  = ddGoal.Value;
                q.returnToClosestWait = cbReturn.Value;

            case 'distance'
                q.start = ddStart.Value;
                q.goal  = ddGoal.Value;

            case 'nearest_landmark'
                q.queryMode = 'pointName';
                q.pointName = ddPoint.Value;

            case 'k_nearest_landmarks'
                q.queryMode = 'pointName';
                q.pointName = ddPoint.Value;
                q.k = max(1, round(efK.Value));

            case 'nearest_signal'
                q.queryMode = 'pointName';
                q.pointName = ddPoint.Value;

            case 'constrained_route'
                q.start = ddStart.Value;
                q.goal  = ddGoal.Value;
                q.requiredSignals = parse_signal_list(efSignals.Value);
                q.returnToClosestWait = cbReturn.Value;

            case 'full_tour'
                q.startWait = ddStart.Value;

            case 'scheduler_demo'

            case 'speculative_dialogue_demo'
                latestText = get_latest_nonempty_line(taDialogue.Value);
                [q, isReady, msg] = parse_natural_language_live(latestText, nodes, ddStart.Value);

                if ~isReady
                    taResult.Value = {msg};
                    gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
                    return
                end

            otherwise
                error('Unsupported query type.');
        end

        gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);

        ansStruct = answer_hri_query(q, nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize);
        taResult.Value = format_hri_output(ansStruct);

        gui_plot_query_result(ax, q, ansStruct, nodes, graph, nameToId, occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters);

    catch ME
        taResult.Value = {['Error: ' ME.message]};
        gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
    end
end

function gui_clear_output(taResult, ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters)
    taResult.Value = {''};
    gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
end

function gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters)
    cla(ax);
    imagesc(ax,[0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid));
    axis(ax,'equal');
    axis(ax,'tight');
    colormap(ax,gray);
    hold(ax,'on');

    for u = 1:graph.N
        for k = 1:numel(graph.adj{u})
            v = graph.adj{u}(k).to;
            if v > u
                plot(ax,[nodes(u).x nodes(v).x], ...
                    mapHeightMeters - [nodes(u).y nodes(v).y], ...
                    'y--','LineWidth',1.0);
            end
        end
    end

    bIdx = strcmp({nodes.type},'building');
    scatter(ax,[nodes(bIdx).x], mapHeightMeters - [nodes(bIdx).y], ...
        65,'r','filled','MarkerEdgeColor','k');

    sIdx = strcmp({nodes.type},'signal');
    scatter(ax,[nodes(sIdx).x], mapHeightMeters - [nodes(sIdx).y], ...
        70,'b','d','filled','MarkerEdgeColor','k');

    wIdx = strcmp({nodes.type},'waiting');
    scatter(ax,[nodes(wIdx).x], mapHeightMeters - [nodes(wIdx).y], ...
        95,'g','s','filled','MarkerEdgeColor','k');

    for i = 1:numel(nodes)
        if strcmp(nodes(i).type,'building')
            c = [0.85 0 0];
        elseif strcmp(nodes(i).type,'signal')
            c = [0 0 1];
        else
            c = [0 0.55 0];
        end
        text(ax, nodes(i).x + 1.2, mapHeightMeters - nodes(i).y + 1.2, nodes(i).name, ...
            'Color',c,'FontSize',7,'FontWeight','bold','Interpreter','none');
    end

    title(ax,'Map View');
    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');
    grid(ax,'on');
    hold(ax,'off');
end

function gui_plot_query_result(ax, q, ansStruct, nodes, graph, nameToId, occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters)
    gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
    hold(ax,'on');

    switch q.type
        case {'route','guide_landmark','distance','constrained_route'}
            if strcmp(q.type,'route')
                route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, false, "dijkstra_heap");
            elseif strcmp(q.type,'guide_landmark')
                route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, q.returnToClosestWait, "dijkstra_heap");
            elseif strcmp(q.type,'distance')
                route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, {}, false, "dijkstra_heap");
            else
                route = plan_graph_route(graph, nodes, nameToId, q.start, q.goal, q.requiredSignals, q.returnToClosestWait, "dijkstra_heap");
            end

            walk = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);
            if ~isempty(walk.xyPath)
                plot(ax, walk.xyPath(:,1), mapHeightMeters - walk.xyPath(:,2), 'c-', 'LineWidth', 2.5);
            end

            ids = route.nodeIds;
            scatter(ax, [nodes(ids).x], mapHeightMeters - [nodes(ids).y], 80, 'm', 'filled');
            for i = 1:numel(ids)
                text(ax, nodes(ids(i)).x + 0.8, mapHeightMeters - nodes(ids(i)).y - 1.5, ...
                    nodes(ids(i)).name, 'Color','m','FontWeight','bold','FontSize',8,'Interpreter','none');
            end

        case 'full_tour'
            tour = full_tourist_guide_mode(graph, nodes, nameToId, q.startWait, "dijkstra_heap");
            walk = realize_graph_route_on_grid(tour.fullRoute, graph, nodes, occupancyGrid, cellSize);

            if ~isempty(walk.xyPath)
                plot(ax, walk.xyPath(:,1), mapHeightMeters - walk.xyPath(:,2), 'g-', 'LineWidth', 2.5);
            end

            ids = tour.visitNodeIds;
            scatter(ax, [nodes(ids).x], mapHeightMeters - [nodes(ids).y], 90, 'm', 'filled');
            for i = 1:numel(ids)
                text(ax, nodes(ids(i)).x + 0.8, mapHeightMeters - nodes(ids(i)).y - 1.5, ...
                    sprintf('%d:%s', i-1, nodes(ids(i)).name), ...
                    'Color','m','FontWeight','bold','FontSize',8,'Interpreter','none');
            end

        case 'nearest_landmark'
            nid = nameToId(q.pointName);
            qxy = [nodes(nid).x, nodes(nid).y];

            scatter(ax, qxy(1), mapHeightMeters - qxy(2), 110, 'k', 'filled');
            kdName = ansStruct.extra.kdName;
            hid = nameToId(kdName);
            scatter(ax, nodes(hid).x, mapHeightMeters - nodes(hid).y, 110, 'c', 'filled');
            plot(ax, [qxy(1) nodes(hid).x], mapHeightMeters - [qxy(2) nodes(hid).y], 'c-', 'LineWidth', 2.0);

        case 'k_nearest_landmarks'
            nid = nameToId(q.pointName);
            qxy = [nodes(nid).x, nodes(nid).y];

            scatter(ax, qxy(1), mapHeightMeters - qxy(2), 110, 'k', 'filled');
            for i = 1:numel(ansStruct.extra.nearestNames)
                hid = nameToId(ansStruct.extra.nearestNames{i});
                scatter(ax, nodes(hid).x, mapHeightMeters - nodes(hid).y, 90, 'c', 'filled');
                plot(ax, [qxy(1) nodes(hid).x], mapHeightMeters - [qxy(2) nodes(hid).y], 'c-', 'LineWidth', 1.5);
            end

        case 'nearest_signal'
            nid = nameToId(q.pointName);
            qxy = [nodes(nid).x, nodes(nid).y];

            scatter(ax, qxy(1), mapHeightMeters - qxy(2), 110, 'k', 'filled');
            hid = nameToId(ansStruct.extra.signalName);
            scatter(ax, nodes(hid).x, mapHeightMeters - nodes(hid).y, 110, 'c', 'filled');
            plot(ax, [qxy(1) nodes(hid).x], mapHeightMeters - [qxy(2) nodes(hid).y], 'c-', 'LineWidth', 2.0);

        case 'scheduler_demo'
            title(ax,'Scheduler demo: text output only');
    end

    hold(ax,'off');
end

function gui_plot_speculative(ax, specOut, nodes, nameToId, graph, occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters)
    gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
    hold(ax,'on');

    if isempty(specOut.timeline)
        hold(ax,'off');
        return
    end

    lastEntry = specOut.timeline{end};
    if isfield(lastEntry,'predictedGoal') && ~isempty(lastEntry.predictedGoal) ...
            && isfield(lastEntry,'predictedStart') && ~isempty(lastEntry.predictedStart) ...
            && isKey(nameToId,lastEntry.predictedStart) && isKey(nameToId,lastEntry.predictedGoal)

        route = plan_graph_route(graph, nodes, nameToId, lastEntry.predictedStart, lastEntry.predictedGoal, {}, false, "dijkstra_heap");
        walk = realize_graph_route_on_grid(route, graph, nodes, occupancyGrid, cellSize);

        if ~isempty(walk.xyPath)
            plot(ax, walk.xyPath(:,1), mapHeightMeters - walk.xyPath(:,2), 'm-', 'LineWidth', 2.5);
        end

        ids = route.nodeIds;
        scatter(ax, [nodes(ids).x], mapHeightMeters - [nodes(ids).y], 85, 'm', 'filled');
    end

    title(ax,'Speculative dialogue: latest predicted route');
    hold(ax,'off');
end

function sigList = parse_signal_list(txt)
    if isempty(strtrim(txt))
        sigList = {};
        return
    end
    parts = strsplit(txt, ',');
    sigList = cellfun(@strtrim, parts, 'UniformOutput', false);
    sigList = sigList(~cellfun(@isempty, sigList));
end

function dialogue = textarea_to_dialogue(textLines)
    if ischar(textLines)
        textLines = cellstr(textLines);
    end
    if isstring(textLines)
        textLines = cellstr(textLines);
    end

    dialogue = cell(numel(textLines),2);
    for i = 1:numel(textLines)
        dialogue{i,1} = (i-1) * 1.5;
        dialogue{i,2} = textLines{i};
    end
end

function outLines = format_hri_output(out)
    outLines = {};
    outLines{end+1} = out.summary;

    if ~isempty(out.nodeSequence)
        outLines{end+1} = ['Node sequence: ' strjoin(out.nodeSequence, ' -> ')];
    end

    if ~isempty(out.graphCost)
        outLines{end+1} = sprintf('Graph cost: %.2f m', out.graphCost);
    end

    if ~isempty(out.gridDistance)
        outLines{end+1} = sprintf('Grid walking distance: %.2f m', out.gridDistance);
    end

    switch out.type
        case 'nearest_landmark'
            outLines{end+1} = sprintf('Query XY: (%.2f, %.2f)', out.extra.queryXY(1), out.extra.queryXY(2));
            outLines{end+1} = sprintf('Linked list nearest landmark: %s (%.2f m)', ...
                out.extra.linkedListName, out.extra.linkedListDist);
            outLines{end+1} = sprintf('KD-tree nearest landmark    : %s (%.2f m)', ...
                out.extra.kdName, out.extra.kdDist);

        case 'k_nearest_landmarks'
            outLines{end+1} = sprintf('Query XY: (%.2f, %.2f)', out.extra.queryXY(1), out.extra.queryXY(2));
            for i = 1:numel(out.extra.nearestNames)
                outLines{end+1} = sprintf('%d) %s (%.2f m)', i, ...
                    out.extra.nearestNames{i}, out.extra.nearestDists(i));
            end

        case 'nearest_signal'
            outLines{end+1} = sprintf('Query XY: (%.2f, %.2f)', out.extra.queryXY(1), out.extra.queryXY(2));
            outLines{end+1} = sprintf('Nearest signal: %s (%.2f m)', ...
                out.extra.signalName, out.extra.signalDist);

        case 'constrained_route'
            if isfield(out.extra,'requiredSignals')
                outLines{end+1} = ['Required signals: ' strjoin(out.extra.requiredSignals, ', ')];
            end

        case 'full_tour'
            outLines{end+1} = ['Visit order: ' strjoin(out.extra.visitOrder, ' -> ')];

        case 'scheduler_demo'
            outLines{end+1} = 'Processed task order:';
            for i = 1:numel(out.extra.processedOrder)
                outLines{end+1} = ['  ' out.extra.processedOrder{i}];
            end
    end
end

function outLines = format_speculative_output(out)
    outLines = {};
    outLines{end+1} = ['Speculative dialogue default start node: ' out.startName];

    for i = 1:numel(out.timeline)
        e = out.timeline{i};
        outLines{end+1} = ' ';
        outLines{end+1} = sprintf('[t = %.1fs] "%s"', e.time, e.utterance);

        if isfield(e,'predictedStart') && ~isempty(e.predictedStart)
            outLines{end+1} = ['Predicted start: ' e.predictedStart];
        end

        if isempty(e.predictedGoal)
            outLines{end+1} = 'Predicted goal : <none>';
        else
            outLines{end+1} = ['Predicted goal : ' e.predictedGoal];
        end

        if ~isempty(e.routeNames)
            outLines{end+1} = ['Predicted route: ' strjoin(e.routeNames, ' -> ')];
            outLines{end+1} = sprintf('Predicted graph cost: %.2f m', e.graphCost);
            outLines{end+1} = sprintf('Predicted grid distance: %.2f m', e.gridDistance);
        else
            outLines{end+1} = 'No valid route prediction yet.';
        end
    end
end
function gui_live_nlp_update(ddType, ddStart, taDialogue, taResult, ax, ...
    nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, ...
    occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters)

    if ~strcmp(ddType.Value, 'speculative_dialogue_demo')
        return
    end

    try
        latestText = get_latest_nonempty_line(taDialogue.Value);

        if isempty(strtrim(latestText))
            taResult.Value = {'Live NLP mode: type a navigation sentence in the text box.'};
            gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
            return
        end

        [q, isReady, msg] = parse_natural_language_live(latestText, nodes, ddStart.Value);

        if ~isReady
            taResult.Value = { ...
                'Live NLP mode', ...
                ['Input: ' latestText], ...
                msg};
            gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
            return
        end

        ansStruct = answer_hri_query(q, nodes, graph, nameToId, linkedLists, landmarkKD, signalKD, occupancyGrid, cellSize);

        outLines = {
            'Live NLP mode'
            ['Input: ' latestText]
            ['Detected type: ' q.type]
            '--------------------------------'
            };
        outLines = [outLines; format_hri_output(ansStruct(:)).']; 

        taResult.Value = outLines;
        gui_plot_query_result(ax, q, ansStruct, nodes, graph, nameToId, occupancyGrid, cellSize, mapWidthMeters, mapHeightMeters);

    catch
        taResult.Value = { ...
            'Live NLP mode', ...
            ['Input: ' get_latest_nonempty_line(taDialogue.Value)], ...
            'Waiting for enough information to build a valid route...'};
        gui_plot_base_map(ax, occupancyGrid, nodes, graph, mapWidthMeters, mapHeightMeters);
    end
end

function [q, isReady, msg] = parse_natural_language_live(userText, nodes, defaultStartName)
    originalText = strtrim(userText);
    s = lower(originalText);

    q = struct();
    q.type = '';
    isReady = false;
    msg = 'Waiting for enough information...';

    allNames = {nodes.name};

    mentionedNames = {};
    mentionedPos = [];
    for i = 1:numel(allNames)
        nm = allNames{i};
        pos = strfind(lower(originalText), lower(nm));
        if ~isempty(pos)
            mentionedNames{end+1} = nm; 
            mentionedPos(end+1) = pos(1); 
        end
    end

    if ~isempty(mentionedNames)
        [~, ord] = sort(mentionedPos);
        mentionedNames = mentionedNames(ord);
    end

    if contains(s, 'full tour') || contains(s, 'tourist guide') || contains(s, 'visit all')
        q.type = 'full_tour';
        q.startWait = find_name_after_keyword(originalText, {'from'}, allNames);
        if isempty(q.startWait)
            q.startWait = defaultStartName;
        end
        isReady = true;
        msg = 'Full tour command detected.';
        return
    end

    if contains(s, 'nearest signal') || contains(s, 'closest signal')
        q.type = 'nearest_signal';
        q.queryMode = 'pointName';
        q.pointName = find_name_after_keyword(originalText, {'to','from','nearest to','closest to'}, allNames);

        if isempty(q.pointName) && ~isempty(mentionedNames)
            q.pointName = mentionedNames{end};
        end

        if isempty(q.pointName)
            msg = 'Detected nearest-signal query. Waiting for a reference point...';
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'closest landmark') || contains(s, 'nearest landmark') || ...
       contains(s, 'closest key point') || contains(s, 'nearest key point')

        k = extract_first_integer(s);

        if ~isempty(k) && k > 1
            q.type = 'k_nearest_landmarks';
            q.k = k;
        else
            q.type = 'nearest_landmark';
        end

        q.queryMode = 'pointName';
        q.pointName = find_name_after_keyword(originalText, {'to','from','nearest to','closest to'}, allNames);

        if isempty(q.pointName) && ~isempty(mentionedNames)
            q.pointName = mentionedNames{end};
        end

        if isempty(q.pointName)
            msg = 'Detected nearest-landmark query. Waiting for a reference point...';
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'distance') || contains(s, 'how far')
        q.type = 'distance';
        q.start = find_name_after_keyword(originalText, {'from'}, allNames);
        q.goal  = find_name_after_keyword(originalText, {'to'}, allNames);

        if isempty(q.start)
            q.start = defaultStartName;
        end

        if isempty(q.goal)
            if numel(mentionedNames) >= 1
                lastName = mentionedNames{end};
                if ~strcmp(lastName, q.start)
                    q.goal = lastName;
                end
            end
        end

        if isempty(q.goal)
            msg = sprintf('Detected distance query. Start = %s. Waiting for destination...', q.start);
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'via') || contains(s, 'through')
        q.type = 'constrained_route';
        q.start = find_name_after_keyword(originalText, {'from'}, allNames);
        q.goal  = find_name_after_keyword(originalText, {'to'}, allNames);

        if isempty(q.start)
            q.start = defaultStartName;
        end

        if isempty(q.goal)
            if numel(mentionedNames) >= 1
                lastName = mentionedNames{end};
                if ~strcmp(lastName, q.start)
                    q.goal = lastName;
                end
            end
        end

        q.requiredSignals = extract_signal_names(mentionedNames);
        q.returnToClosestWait = true;

        if isempty(q.goal)
            msg = sprintf('Detected constrained route. Start = %s. Waiting for destination...', q.start);
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'guide me') || contains(s, 'take me to') || contains(s, 'bring me to')
        q.type = 'guide_landmark';
        q.start = find_name_after_keyword(originalText, {'from'}, allNames);
        q.goal  = find_name_after_keyword(originalText, {'to'}, allNames);

        if isempty(q.start)
            q.start = defaultStartName;
        end

        if isempty(q.goal) && ~isempty(mentionedNames)
            lastName = mentionedNames{end};
            if ~strcmp(lastName, q.start)
                q.goal = lastName;
            end
        end

        q.returnToClosestWait = contains(s, 'return');

        if isempty(q.goal)
            msg = sprintf('Detected guide query. Start = %s. Waiting for destination...', q.start);
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'emergency') || contains(s, 'urgent') || contains(s, 'immediately')
        q.type = 'route';
        q.start = find_name_after_keyword(originalText, {'from'}, allNames);
        q.goal  = find_name_after_keyword(originalText, {'to'}, allNames);

        if isempty(q.start)
            q.start = defaultStartName;
        end

        if isempty(q.goal) && ~isempty(mentionedNames)
            lastName = mentionedNames{end};
            if ~strcmp(lastName, q.start)
                q.goal = lastName;
            end
        end

        if isempty(q.goal)
            msg = sprintf('Detected emergency route. Start = %s. Waiting for destination...', q.start);
            return
        end

        isReady = true;
        return
    end

    if contains(s, 'how do i get') || contains(s, 'route') || contains(s, 'path') || ...
       contains(s, 'go from') || contains(s, 'go to') || contains(s, 'head to')

        q.type = 'route';
        q.start = find_name_after_keyword(originalText, {'from'}, allNames);
        q.goal  = find_name_after_keyword(originalText, {'to'}, allNames);

        if isempty(q.start)
            q.start = defaultStartName;
        end

        if isempty(q.goal) && ~isempty(mentionedNames)
            lastName = mentionedNames{end};
            if ~strcmp(lastName, q.start)
                q.goal = lastName;
            end
        end

        if isempty(q.goal)
            msg = sprintf('Detected route query. Start = %s. Waiting for destination...', q.start);
            return
        end

        isReady = true;
        return
    end

    msg = 'No clear intent detected yet. Keep typing...';
end

function lineText = get_latest_nonempty_line(textLines)
    if ischar(textLines)
        textLines = cellstr(textLines);
    end
    if isstring(textLines)
        textLines = cellstr(textLines);
    end

    lineText = '';
    for i = numel(textLines):-1:1
        t = strtrim(textLines{i});
        if ~isempty(t)
            lineText = t;
            return
        end
    end
end

function nameOut = find_name_after_keyword(textIn, keywordList, allNames)
    lowerText = lower(textIn);
    nameOut = '';

    bestPos = inf;
    bestName = '';

    for k = 1:numel(keywordList)
        kw = lower(keywordList{k});
        kwPos = strfind(lowerText, kw);
        if isempty(kwPos)
            continue
        end

        for i = 1:numel(allNames)
            nm = allNames{i};
            nmPos = strfind(lowerText, lower(nm));
            if isempty(nmPos)
                continue
            end

            p = nmPos(1);
            if p > kwPos(1) && p < bestPos
                bestPos = p;
                bestName = nm;
            end
        end
    end

    nameOut = bestName;
end

function k = extract_first_integer(s)
    tok = regexp(s, '\d+', 'match');
    if isempty(tok)
        k = [];
    else
        k = str2double(tok{1});
    end
end

function sigs = extract_signal_names(nameList)
    sigs = {};
    for i = 1:numel(nameList)
        nm = nameList{i};
        if ~isempty(regexp(nm, '^S\d+$', 'once'))
            sigs{end+1} = nm; 
        end
    end
end