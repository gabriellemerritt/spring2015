clc
clear
%%
% % a = ceil(rand(10000000,3)*900);
% a = randi(50000,50000,3);
% graph = a(a(:,1)~=a(:,2),:);
%%
load('test_graph2.mat')
%%
profile off; 
profile on;
[path, cost] = working(graph, min(graph(:,1)), max(graph(:,1)))
profile viewer