function [min_dis_vertex] = minDist(dist, Q, node_num, nodes)
min_dis = inf;  
for i = 1:node_num 
  if(dist(i) <= min_dis)
    if(any (Q == nodes(i)))
        min_dis = dist(i); 
        min_dis_vertex = nodes(i);
    end
  end
end

min_dis_vertex;
end
