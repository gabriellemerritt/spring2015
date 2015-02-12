close all;
% clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map1.txt', 0.1, 1, 0.25);
start = {[0.0  -4.9 0.2]};
stop  = {[6.0  18.0-6 3.0]};
% start = {[2.5  2.5 0.2]};
% stop  = {[19.25  3.0 5.0]};
% start = {[5  -3 4]};
% stop  = {[6.0  18.0 3.0]};
nquad = length(start);
for qn = 1:nquad
%     path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
 path{qn} = dijkstra(map, start{qn}, stop{qn},false);

end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
