function [path, cost] = shortestpath(Graph, start, goal)
% Shortest path assignment by Gabrielle Merritt

if isempty(Graph)
    cost = 0; 
    path = zeros(0,1);
    return; 
end
    
nodes = unique(Graph(:,1:2)); 
Graph = sortrows(Graph,3);
[u, ia , ic] = unique(Graph(:,1:2),'rows');

Dji_mat = sparse(Graph(ia,1),Graph(ia,2),Graph(ia,3)); 
[m,d] = size(Dji_mat); 
if (m ~= d) 
    if m > d
        Dji_mat(:,d+(m -d)) = 0; 
    else
        Dji_mat(m+ (d-m),:) = 0; 
    end 
end 
Dji_mat = Dji_mat + Dji_mat'; 
n = length(Dji_mat);
%%

node = start; 
prev = NaN(n,1);
%distance from start vertex 
dist = inf(n,1); 
dist(start) = 0; 
prev(start) = start;
cost = 0;


%%
% order rows so start row is at the top 
 calc_m = Dji_mat;
 cl = zeros(n,1); 
 cl(nodes) = nodes; 
 calc_m = [calc_m cl];
% calc_m = sortrows(calc_m, start);
% for i=1:n
%     if (calc_m(i, end) ~= 0)
%          q(i) = calc_m(i,end); 
%         adj(i) = calc_m(i,end); 
%     end
% end
%%
% non visted node list 
q = [1:n];
% Basically a list of the nodes for adjacency stuff 

adj = [1:n]; 

for v =1:n-1
    %node = minDist(dist,q,n); 
    % find min distance from current node to adjacent node
    min_dis = inf;  
    for i = 1:length(q)
        %  any (q ==i ) seems to be what my program hates 
        if(any(q == i) && dist(i) <= min_dis)
             min_dis = dist(i); 
             node = i;
        end
    end
  
    %%
    % find  and delete current node from not visited queue 
    r = find(q == node); 
    q(r) = []; 
    % 
    % vectorizing find columns where distance from start to a certain node
    % is less than what i have in my dist array (basically the cost array) 
%     test = (Dji_mat(node, adj)+dist(node) < dist(adj)') & Dji_mat(node, adj);    
    test = (Dji_mat(adj, node)+dist(node) < dist(adj)) & Dji_mat(adj, node);

    if (dist(node) ~=inf && any(test))
        dist(test) = dist(node)+Dji_mat(test, node);
        prev(test) = node; 
    end
        
end

cost = dist(goal);
if cost == inf 
    disp('No path to goal'); 
    path = zeros(0,1); 
else
    path = findPath(prev, goal,start);
end 


end
