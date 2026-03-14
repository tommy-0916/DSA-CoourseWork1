clc
clear
close all

%% =========================================================
% TASK 2 IMPLEMENTATION
% - Linked list for landmarks / signal points
% - Adjacency-list graph
% - BFS on occupancy grid
% - Dijkstra without priority queue
% - Dijkstra with binary min-heap
% - Full tourist-guide mode (exact TSP over 7 landmarks)
%
% IMPORTANT DESIGN DECISION:
%   BFS is used on the OCCUPANCY GRID to obtain the walkable cell-by-cell route.
%   Dijkstra is used on the ABSTRACT GRAPH to choose which landmarks/signal points
%   the robot should traverse, where graph edge weights are Euclidean distances.
%
%   Full navigation therefore has two levels:
%   1) topological planning on graph  -> sequence of named nodes
%   2) metric walking on occupancy grid -> BFS path for each graph edge
%% =========================================================

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

%% =========================
% 19) DEMO 1: ABSTRACT GRAPH SEARCH
%% =========================
startName = 'W2';
goalName  = 'Templar';
requiredSignals = {'S5','S6'};   % Example of signal-constrained route

fprintf('\n===== DEMO 1: ABSTRACT GRAPH ROUTING =====\n');
routeNoHeap = plan_graph_route(graph, nodes, nameToId, startName, goalName, requiredSignals, true, "dijkstra_plain");
routeHeap   = plan_graph_route(graph, nodes, nameToId, startName, goalName, requiredSignals, true, "dijkstra_heap");

fprintf('Dijkstra (no heap): total graph cost = %.2f m\n', routeNoHeap.totalCost);
fprintf('Dijkstra (heap)   : total graph cost = %.2f m\n', routeHeap.totalCost);
disp('Node sequence (heap version):')
disp(strjoin(routeHeap.nodeNames, ' -> '))

%% =========================
% 20) DEMO 2: BFS WALKING ROUTE ON OCCUPANCY GRID
%% =========================
walkPlan = realize_graph_route_on_grid(routeHeap, graph, nodes, occupancyGrid, cellSize);

fprintf('\n===== DEMO 2: BFS REALIZATION ON GRID =====\n');
fprintf('Number of graph legs realized by BFS: %d\n', numel(walkPlan.legPaths));
fprintf('Approx grid walking distance: %.2f m\n', walkPlan.totalDistanceMeters);

figure('Name','Full Navigation: Graph + Grid BFS')
imagesc([0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid))
axis equal tight
clim([0 1])
colormap(gray)
hold on

plot_graph_edges(nodes, graph, mapHeightMeters);
plot_grid_path(walkPlan.xyPath, mapHeightMeters, 'c-', 2.2);

scatter([nodes.x], mapHeightMeters - [nodes.y], 25, 'y', 'filled', 'MarkerEdgeColor','k');
for i = 1:numel(routeHeap.nodeIds)
    nid = routeHeap.nodeIds(i);
    text(nodes(nid).x + 1, mapHeightMeters - nodes(nid).y + 1, nodes(nid).name, ...
        'Color','y','FontWeight','bold','FontSize',8,'Interpreter','none');
end

title(sprintf('Signal-Constrained Navigation: %s -> %s -> return', startName, goalName))
xlabel('X (m)')
ylabel('Y (m)')
grid on
hold off

%% =========================
% 21) DEMO 3: FULL TOURIST-GUIDE MODE (TSP)
%% =========================
fprintf('\n===== DEMO 3: FULL TOURIST-GUIDE MODE =====\n');
tour = full_tourist_guide_mode(graph, nodes, nameToId, 'W1', "dijkstra_heap");
fprintf('Best landmark visit order from %s:\n', 'W1');
disp(strjoin(tour.visitNames, ' -> '))
fprintf('Total graph cost of full tour: %.2f m\n', tour.totalCost);

tourWalk = realize_graph_route_on_grid(tour.fullRoute, graph, nodes, occupancyGrid, cellSize);
fprintf('Approx full-tour grid walking distance: %.2f m\n', tourWalk.totalDistanceMeters);

figure('Name','Full Tourist Guide Mode')
imagesc([0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid))
axis equal tight
clim([0 1])
colormap(gray)
hold on
plot_graph_edges(nodes, graph, mapHeightMeters);
plot_grid_path(tourWalk.xyPath, mapHeightMeters, 'g-', 2.5);
scatter([nodes.x], mapHeightMeters - [nodes.y], 30, 'r', 'filled', 'MarkerEdgeColor','k');
for i = 1:numel(tour.visitNodeIds)
    nid = tour.visitNodeIds(i);
    text(nodes(nid).x + 0.8, mapHeightMeters - nodes(nid).y + 0.8, sprintf('%d',i-1), ...
        'Color','w', 'FontWeight','bold', 'FontSize',8);
end
title('Full Tourist-Guide Mode: exact TSP over 7 landmarks')
xlabel('X (m)')
ylabel('Y (m)')
grid on
hold off

%% =========================
% 22) BENCHMARKING EXAMPLES
%% =========================
fprintf('\n===== BENCHMARKS =====\n');

scenarioPairs = {
    'W2','LCF',{'S4'}
    'W1','HERA',{'S1','S5','S6'}
    'W1','Bread St Kitchen',{'S1'}
};

for s = 1:size(scenarioPairs,1)
    sStart = scenarioPairs{s,1};
    sGoal  = scenarioPairs{s,2};
    sReq   = scenarioPairs{s,3};

    t1 = tic;
    r1 = plan_graph_route(graph, nodes, nameToId, sStart, sGoal, sReq, true, "dijkstra_plain");
    tPlain = toc(t1);

    t2 = tic;
    r2 = plan_graph_route(graph, nodes, nameToId, sStart, sGoal, sReq, true, "dijkstra_heap");
    tHeap = toc(t2);

    fprintf('Scenario %d: %s -> %s\n', s, sStart, sGoal);
    fprintf('  Plain Dijkstra: %.6f s, cost %.2f m\n', tPlain, r1.totalCost);
    fprintf('  Heap  Dijkstra: %.6f s, cost %.2f m\n', tHeap,  r2.totalCost);
end

t3 = tic;
tourTmp = full_tourist_guide_mode(graph, nodes, nameToId, 'W1', "dijkstra_heap");
tTour = toc(t3);
fprintf('Full tourist-guide mode (exact TSP over 7 landmarks): %.6f s\n', tTour);
fprintf('Tour cost: %.2f m\n', tourTmp.totalCost);

%% =========================
% 23) EXAMPLE LOOKUPS USING LINKED LIST / KD-TREE
%% =========================
queryXY = [nodes(nameToId('W1')).x + 3, nodes(nameToId('W1')).y + 3];

[idxList, distList] = linked_list_nearest(linkedLists.landmarks, queryXY, nodes);
[idxKD, distKD]     = kd_nearest(landmarkKD, queryXY, nodes, inf, -1);

fprintf('\n===== LOOKUP EXAMPLE =====\n');
fprintf('Nearest landmark by linked list: %s (%.2f m)\n', nodes(idxList).name, distList);
fprintf('Nearest landmark by KD-tree    : %s (%.2f m)\n', nodes(idxKD).name, distKD);

%% =========================================================
% LOCAL FUNCTIONS
%% =========================================================

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
        waypoints(end+1) = nameToId(char(requiredSignals{i})); %#ok<AGROW>
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
            fullIds = [fullIds, segIds(2:end)]; %#ok<AGROW>
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
        path = [curr, path]; %#ok<AGROW>
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

    % Expand graph route into graph edges, then BFS each consecutive node pair on grid.
    for i = 1:numel(nodeIds)-1
        src = nodeIds(i);
        dst = nodeIds(i+1);

        [legRC, legDist] = bfs_grid(occ, [nodes(src).row nodes(src).col], [nodes(dst).row nodes(dst).col], cellSize);
        if isempty(legRC)
            error('BFS failed on occupancy grid between %s and %s.', nodes(src).name, nodes(dst).name);
        end

        legPaths{end+1} = legRC; %#ok<AGROW>
        if isempty(fullRC)
            fullRC = legRC;
        else
            fullRC = [fullRC; legRC(2:end,:)]; %#ok<AGROW>
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

    % check if start point and goal point are available
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

    % 8-neighbour motion
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

            % avoiding "crossing the cornor"
            if abs(moves(k,1)) == 1 && abs(moves(k,2)) == 1 % this if checks if it is moving tilt
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

    % get the recorded path
    rev = gidx;
    curr = gidx;
    while curr ~= sidx
        curr = parent(curr);
        rev(end+1) = curr; %#ok<AGROW>
    end
    rev = fliplr(rev);

    pathRC = zeros(numel(rev), 2);
    for i = 1:numel(rev)
        [r,c] = ind2sub([numRows, numCols], rev(i));
        pathRC(i,:) = [r,c];
    end

    % calculate the actual length moved
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

    % Precompute shortest graph-path cost and actual graph path between all relevant nodes.
    relevant = [waitId, buildingIds]; % all nodes needed to be visited
    M = numel(relevant);
    costMat = inf(M,M); % cost from i to j
    pathCell = cell(M,M); % graph path from i to j

    % pre-calclulating all pairs
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

    % Exact TSP because there are only 7 landmarks -> 7! = 5040 tours, which is feasible.
    permsIdx = perms(2:M);  % indices into 'relevant'; 1 is the start waiting point
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
            fullRouteIds = [fullRouteIds, seg(2:end)]; %#ok<AGROW>
        end
    end

    tour.visitNodeIds = visitNodeIds;
    tour.visitNames = {nodes(visitNodeIds).name};
    tour.totalCost = bestCost;
    tour.fullRoute.nodeIds = fullRouteIds;
    tour.fullRoute.nodeNames = {nodes(fullRouteIds).name};
    tour.fullRoute.totalCost = bestCost;
end
