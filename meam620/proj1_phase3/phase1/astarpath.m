function [path, nv] =astarpath(Graph, start, goal, heu)
if isempty(Graph)
    cost = 0; 
    nv = 0; 
    path = zeros(0,1);
    return; 
end
nodes = unique(Graph(:,1:2));
n = max(nodes); 
% vertex_list = [1:n]'; 
G = [Graph; Graph(:,[2 1 3 ])]; 
G = sortrows(G,3); 
[~, ia ,~] = unique(G(:,1:2),'rows','first'); 
G = G(ia,:); 
% h = [0;heu]; 
h = heu'; 
% n
Dji_mat = sparse(G(:,1),G(:,2),G(:,3)); 

node = start; 
prev = NaN(max(nodes),1);
dist = inf(1,max(nodes)); 
dist(start) = 0; 
prev(start) = start;
cost = 0;
visited = []; 
temp = zeros(1,max(nodes));
ast = temp;
t0 = clock;
test_vector = zeros(1,max(nodes));
for i=1:n   
    if(etime(clock,t0) > 450)
        path = zeros(0,1); 
        nv = 0;
        return;
    end
    if (dist(node) ~=inf)
        weight_vector  = Dji_mat(:,node)';
        d = dist(node); 
        test_vector = ((weight_vector + d) < dist) & weight_vector; 
        t = max(test_vector); 
        if (t)
            dist(test_vector) = dist(node)+ Dji_mat(node,test_vector); 
            prev(test_vector) = node;
        end
    end
    if (node == goal)
        break; 
    end
     visited(i) = node; 
%      too slow
      % adj_vert(visited) = []; 
   temp = dist; 
   temp(visited) = inf; 
   ast = temp + h; 
   [~, idx] = min(ast); 
   node = (idx);
end
cost = dist(goal);
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