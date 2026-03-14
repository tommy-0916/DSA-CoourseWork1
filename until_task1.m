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
% Main loop:
%   S1 -> S2 -> S3 -> SadlersWells -> S4 -> LCF -> W2 ->
%   S5 -> HERA -> S6 -> S7 -> S8 -> Templar -> W1 -> S1
%
% Spurs from W1:
%   W1 -> itsu -> W1
%   W1 -> PRET -> W1
%   W1 -> BreadSt -> W1

% Main loop path
lat_main = [
    lat_signal(1)       % S1
    lat_signal(2)       % S2
    lat_signal(3)       % S3
    lat_buildings(1)    % Sadlers Wells
    lat_signal(4)       % S4
    lat_buildings(2)    % LCF
    lat_wait(2)         % W2
    lat_signal(5)       % S5
    lat_buildings(3)    % HERA
    lat_signal(6)       % S6
    lat_signal(7)       % S7
    lat_signal(8)       % S8
    lat_buildings(4)    % Templar
    lat_wait(1)         % W1
    lat_signal(1)       % S1 (close loop)
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

% Spurs from W1
lat_spurs = [
    lat_wait(1); lat_buildings(5); lat_wait(1)   % W1->itsu->W1
    lat_wait(1); lat_buildings(6); lat_wait(1)   % W1->PRET->W1
    lat_wait(1); lat_buildings(7); lat_wait(1)   % W1->BreadSt->W1
];
lon_spurs = [
    lon_wait(1); lon_buildings(5); lon_wait(1)
    lon_wait(1); lon_buildings(6); lon_wait(1)
    lon_wait(1); lon_buildings(7); lon_wait(1)
];

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
roadHalfWidth = 5.0;   % metres — tune this

%% =========================
% 11) CARVE HELPER
%% =========================
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

%% =========================
% 12) CARVE MAIN LOOP
%% =========================
interpSpacing = cellSize / 2;

function occupancyGrid = carve_path(occupancyGrid, xs_path, ys_path, r, cellSize, numRows, numCols, interpSpacing)
    for seg = 1:length(xs_path)-1
        x1 = xs_path(seg);   y1 = ys_path(seg);
        x2 = xs_path(seg+1); y2 = ys_path(seg+1);
        segLen = sqrt((x2-x1)^2+(y2-y1)^2);
        nPts   = max(2, ceil(segLen/interpSpacing));
        xs = linspace(x1,x2,nPts);
        ys = linspace(y1,y2,nPts);
        for k = 1:nPts
            occupancyGrid = carve_circle(occupancyGrid, xs(k), ys(k), r, cellSize, numRows, numCols);
        end
    end
end

% Carve main loop
occupancyGrid = carve_path(occupancyGrid, x_main, y_main, ...
    roadHalfWidth, cellSize, numRows, numCols, interpSpacing);

% Carve spurs (W1 <-> itsu/PRET/BreadSt)
occupancyGrid = carve_path(occupancyGrid, x_spurs, y_spurs, ...
    roadHalfWidth, cellSize, numRows, numCols, interpSpacing);

%% =========================
% 13) POST-PROCESSING
%% =========================

% ---------------------------------------------------------
% A) SMOOTH — manual morphological closing (no toolbox needed)
% ---------------------------------------------------------
smoothR = round(3.0 / cellSize);
[se_xx, se_yy] = meshgrid(-smoothR:smoothR, -smoothR:smoothR);
SE_disk = double((se_xx.^2 + se_yy.^2) <= smoothR^2);
SE_sum  = sum(SE_disk(:));

freeMap        = double(~logical(occupancyGrid));
freeMap_dil    = conv2(freeMap,            SE_disk, 'same') > 0;
freeMap_closed = conv2(double(freeMap_dil),SE_disk, 'same') >= SE_sum;
occupancyGrid  = double(~freeMap_closed);

% ---------------------------------------------------------
% B) RE-CARVE all route nodes to guarantee they stay free
% ---------------------------------------------------------
recarveR = roadHalfWidth * 0.7;
all_rx = [x_main(:); x_spurs(:); x_signal(:); x_wait(:)];
all_ry = [y_main(:); y_spurs(:); y_signal(:); y_wait(:)];
for i = 1:length(all_rx)
    occupancyGrid = carve_circle(occupancyGrid, all_rx(i), all_ry(i), ...
        recarveR, cellSize, numRows, numCols);
end

%% =========================
% 14) BORDER WALL
%% =========================
borderCells = ceil(padding / 2 / cellSize);
occupancyGrid(1:borderCells,       :) = 1;
occupancyGrid(end-borderCells:end, :) = 1;
occupancyGrid(:, 1:borderCells      ) = 1;
occupancyGrid(:, end-borderCells:end) = 1;

%% =========================
% 15) FIGURE 1 — CLEAN GRID
%% =========================
figure('Name','Occupancy Grid')
imagesc([0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid))
axis equal tight
clim([0 1])
colormap(gray)
cb = colorbar;
cb.Ticks = [0 1]; cb.TickLabels = {'Free (0)','Occupied (1)'};
title(sprintf('Occupancy Grid  |  road half-width = %.1fm', roadHalfWidth))
xlabel('X (m)'); ylabel('Y (m)')
grid on

%% =========================
% 16) FIGURE 2 — GRID + LANDMARKS
%% =========================
figure('Name','Occupancy Grid + Landmarks')
imagesc([0 mapWidthMeters],[0 mapHeightMeters], flipud(occupancyGrid))
axis equal tight; clim([0 1]); colormap(gray)
hold on

h = []; labels = {};

% Main route
h1 = plot(x_main, mapHeightMeters - y_main, 'm-', 'LineWidth', 1.5);
h(end+1) = h1; labels{end+1} = 'Main Route';

% Spurs
h2 = plot(x_spurs, mapHeightMeters - y_spurs, 'm--', 'LineWidth', 1.2);
h(end+1) = h2; labels{end+1} = 'Spurs (W1)';

% Buildings
h3 = scatter(x_buildings, mapHeightMeters - y_buildings, ...
    80, 'r', 'filled', 'MarkerEdgeColor','k');
h(end+1) = h3; labels{end+1} = 'Buildings';
for i = 1:length(x_buildings)
    text(x_buildings(i)+2, mapHeightMeters-y_buildings(i), buildingNames{i}, ...
        'Color',[0.85 0 0],'FontSize',7,'FontWeight','bold','Interpreter','none');
end

% Signal points
h4 = scatter(x_signal, mapHeightMeters - y_signal, ...
    90, 'b', 'd', 'filled', 'MarkerEdgeColor','k');
h(end+1) = h4; labels{end+1} = 'Signal Points';
for i = 1:length(x_signal)
    text(x_signal(i)+1.5, mapHeightMeters-y_signal(i), sprintf('S%d',i), ...
        'Color','b','FontSize',7);
end

% Waiting areas
h5 = scatter(x_wait, mapHeightMeters - y_wait, ...
    120, 'g', 's', 'filled', 'MarkerEdgeColor','k');
h(end+1) = h5; labels{end+1} = 'Waiting Areas';
for i = 1:length(x_wait)
    text(x_wait(i)+1.5, mapHeightMeters-y_wait(i), sprintf('W%d',i), ...
        'Color',[0 0.55 0],'FontSize',7,'FontWeight','bold');
end

legend(h, labels, 'Location','best', 'FontSize',8)
title('Occupancy Grid — Route Dilation')
xlabel('X (m)'); ylabel('Y (m)')
grid on; hold off

%% =========================
% 17) OCCUPANCY MAP OBJECT (optional)
%% =========================
% omap = occupancyMap(occupancyGrid, 1/cellSize);
% figure; show(omap)