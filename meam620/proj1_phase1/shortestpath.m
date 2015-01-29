function [path, nv] =shortestpath(Graph, start, goal)
if isempty(Graph)
    cost = 0; 
    nv = 0; 
    path = zeros(0,1);
    return; 
end
nodes = unique(Graph(:,1:2));
n = max(nodes); 
vertex_list = [1:n]'; 
G = [Graph; Graph(:,[2 1 3])]; 
G = sortrows(G,3); 
[~, ia ,~] = unique(G(:,1:2),'rows','first'); 
G = G(ia,:); 
Dji_mat = sparse(G(:,1),G(:,2),G(:,3)); 
node = start; 
prev = NaN(max(nodes),1);
dist = inf(max(nodes),1); 
dist(start) = 0; 
prev(start) = start;
temp = inf(max(nodes),1);
cost = 0;
visited = []; 
for i=1:n    
    if (dist(node) ~=inf)
        weight_vector  = Dji_mat(:,node);
        % try  > 0 
        test_vector = ((weight_vector + dist(node)) < dist) &weight_vector; 
        if (any(test_vector))
            dist(test_vector) = dist(node)+ Dji_mat(node,test_vector); 
            prev(test_vector) = node;
        end
    end
    if (node == goal)
        break; 
    end
   %  visited(node) = true; 
     visited(i) = node; 
%      too slow
      % adj_vert(visited) = []; 
   temp = dist; 
   temp(visited) = inf; 
   [~, idx] = min(temp); 
   node = (idx);
   
     
end
cost = dist(goal);
size(dist)
if cost == inf 
    disp('No path to goal'); 
    path = zeros(0,1); 
    nv = length(visited) ;
% elseif isempty(cost)
%     path = zeros(0,1); 
else
    path = findPath(prev, goal,start);
nv = length(visited); 
end 