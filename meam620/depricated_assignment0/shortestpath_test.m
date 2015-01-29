function [test_suite] = shortestpath_test
close all
clear all
clc

% the files you want to test
testEmptyGraph
testGraph1
end

function assertPathValid(returnedPath, returnedCost, start, goal, G)
% check dimensions
assert(size(returnedPath, 2) == 1);

if ~isempty(returnedPath);
    % check start+end
    assert(returnedPath(1) == start);
    assert(returnedPath(end) == goal);

    % check graph
    pathEdges = zeros(length(returnedPath)-1, 2);
    for i = 1:length(returnedPath)-1,
        pathEdges(i, :) = [returnedPath(i) returnedPath(i+1)];
    end
    modifiedGraph = [G; G(:, 2) G(:, 1) G(:, 3)];

    [~, loc1]  = ismember(modifiedGraph(:, 1), pathEdges(:, 1));
    [~, loc2] = ismember(modifiedGraph(:, 2), pathEdges(:, 2));
    locations = (loc1 == loc2) & (loc2~= 0);
    allCosts = modifiedGraph(locations, 3);
    assert(returnedCost == sum(allCosts)); % check weight
    assert(length(returnedPath)-1 == sum(locations)); % check edges
end

end

function testGraph1
load('sample_graph.mat');
start = 1;
goal  = 5;
[returnedPath, returnedCost] = working(G, start, goal)
assertPathValid(returnedPath, returnedCost, start, goal, G);
end

function testEmptyGraph
load('empty_graph.mat');
start = 1;
goal = 4;
[returnedPath, returnedCost] = working(G, start, goal)
assertPathValid(returnedPath, returnedCost, start, goal, G);
end

