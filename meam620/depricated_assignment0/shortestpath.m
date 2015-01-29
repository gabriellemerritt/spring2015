function [path, cost] =shortestpath(Graph, start, goal)
% Shortest path assignment by Gabrielle Merritt
% SHORTESTPATH Find the shortest path from start to goal on the given Graph.
%   PATH = SHORTESTPATH(Graph, start, goal) returns an M-by-1 matrix, where each row
%   consists of the node on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-1 matrix.
%
%   [PATH, COST] = SHORTESTPATH(...) returns the path as well as
%   the total cost of the path.
%   Arguments:
%      Graph is a N-by-3 matrix, representing all edges in the given graph.
%      Each row has the format of [U, V, W] where U and V are two nodes that
%      this edge connects and W is the weight(cost) of traveling through
%      the edge. For example, [1 5 20.2] means this edge connects
%      node 1 and 5 with cost 20.2. Please note that all edges are
%      undirected and you can assume that the nodes in Graph are indexed by
%      1, ..., n where n is the number of nodes in the Graph.
%

% Hint: You may consider constructing a different graph structure to speed up
% you code.

if isempty(Graph)
    cost = 0; 
    path = zeros(0,1);
    return; 
end
data =sortrows(Graph,3); 
[dt, ia ,ic] = unique(data(:,1:2),'rows'); 
data = data(ia,:); 
Dji_mat = sparse(data(:,1),data(:,2),data(:,3));
[m,d] = size(Dji_mat); 
if (m ~= d) 
    if m > d
        Dji_mat(:,d+(m -d)) = 0; 
    else
        Dji_mat(m+ (d-m),:) = 0; 
    end 
end 
nodes = unique(Graph(:,1:2)); 
u = length(nodes); 
n = length(Graph);  
node = start; 
prev = NaN(u,1);
dist = inf(u,1); 
dist(start) = 0; 
prev(start) = start;
cost = 0;

% for i=1:u
%     [row, col] = find(Graph(:,1:2)==nodes(i));
%     %calc_m = Graph(row,:);
%     %[r,c] = find(Graph(row,1:2)~=i); 
%     for j=1:length(row)
%         if col(j) == 1
%             col(j) = 2;
%         else 
%             col(j) = 1;
%         end
%    
%         r2 = find(nodes == Graph(row(j),col(j))); 
%         if (Graph(row(j),3) <= Dji_mat(i,r2) || Dji_mat(i,r2)== 0)
%            % disp('did i make it');
%             Dji_mat(i,r2)= Graph(row(j),3);
%         end
%     end
% end 
% Dji_mat(start,start) = 0;

    
q = nodes;
adj = [1:u];
Q = [1:u];
size(Dji_mat)
size(q)


for v =1:n-1
% while (~isempty(q))

 %  node = minDist(dist,q,u,nodes); 
   % min_dis = inf;  
    
    %node = nodes(dist ==(min(dist(Q))));
    

    idx = find(dist == (min(dist(Q)))); 
%     node = nodes(idx);
%     if( length(node) > 1) 
%         if (any(q == node(1)))
%             node = node(1);
%         else
%             node = node(2); 
%         end
%     end
     if( length(idx) > 1) 
        if (any(q == idx(1)))
            idx = idx(1);
        else
            idx = idx(2); 
        end
     end
     
     %idx = find(nodes == node);
     node = nodes(idx);

%     for i = 1:length(q)
%         %  any (q ==i ) seems to be what my program hates 
%         if(any(q == i) && dist(i) <= min_dis)
%              min_dis = dist(i); 
%              node = i;
%         end
%     end
% %     
%     size(q)
   
   r = find(q == node); 
   q(r) = []; 
   Q(r) = [];

%     
%     if (dist(node) ~=inf)
%         test = (Dji_mat(node, adj)+ dist(node) < dist(adj)') & Dji_mat(node, adj);
%         if any(test)
%             dist(test) = dist(node)+Dji_mat(node, test);
%             prev(test) = node;
%         end
%     end
%     
%    if (dist(node) ~=inf)
%        test1 =  ((Dji_mat(adj,node)+ dist(node) < dist(adj)) & Dji_mat(adj, node));
%        if any(test1)
%             dist(test1) = dist(node)+Dji_mat(test1, node);
%             prev(test1) = node;
%        end
%    end
    
    if (dist(idx) ~=inf)
        test = (Dji_mat(idx, adj)+ dist(idx) < dist(adj)') & Dji_mat(idx, adj);
        if any(test)
            dist(test) = dist(node)+Dji_mat(node, test);
            prev(test) = idx;
        end
    end
    
   if (dist(idx) ~=inf)
       test1 =  ((Dji_mat(adj,idx)+ dist(idx) < dist(adj)) & Dji_mat(adj, idx));
       if any(test1)
            dist(test1) = dist(idx)+Dji_mat(test1, idx);
            prev(test1) =  idx;
       end
   end

    if (nodes(idx) == goal)
        break; 
    end
    
 end

%cost = dist(goal);
c = find(nodes == goal);
cost = dist(c) 

% if cost == inf 
%     disp('No path to goal'); 
%     path = zeros(0,1); 
% elseif isempty(cost)
%     path = zeros(0,1); 
%         
% else
%     path = findPath(prev, goal,start,nodes);
% %     for (p =1:length(path))
% %         path(p) = nodes(path(p)); 
% %     end 
%     path = nodes(path); 
% end 
% 

    
end
