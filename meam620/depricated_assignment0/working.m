function [path, cost] =working(Graph, start, goal)
if isempty(Graph)
    cost = 0; 
    path = zeros(0,1);
    return; 
end
nodes = unique(Graph(:,1:2)); 
n = max(nodes); 
vertex_list = [1:n]'; 
G = [Graph; Graph(:,[2 1 3])]; 
G = sortrows(G,3); 
[~, ia ,~] = unique(G(:,1:2),'rows','first'); 
G1 = G(ia,:); 
Dji_mat = sparse(G1(:,1),G1(:,2),G1(:,3)); 
node = start; 
prev = NaN(max(nodes),1);
dist = inf(max(nodes),1); 
dist(start) = 0; 
prev(start) = start;
cost = 0;
%visited = false(n,1); 
visited = []; 
for i=1:n 
    temp = dist; 
    if (dist(node) ~=inf)
        weight_vector  = Dji_mat(:,node);
        test_vector = (weight_vector & ((weight_vector + dist(node)) < dist)); 
        if (any(test_vector))
            dist(test_vector) = dist(node)+ Dji_mat(node,test_vector); 
            prev(test_vector) = node;
        end
    end
        
    if (node == goal)
        break; 
    end
    visited(i) = node; 
    %    temp = dist; 
    temp(visited) = inf; 
    [~, idx] = min(temp); 
    node = (idx); 
     
end
cost = dist(goal);
if cost == inf 
    disp('No path to goal'); 
    path = zeros(0,1); 
else
    path = findPath(prev, goal,start);
end 