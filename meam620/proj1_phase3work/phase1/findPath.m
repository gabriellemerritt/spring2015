function [path] = findPath(prev, goal_idx, start_idx)
g = goal_idx;
% j=2;
path(1) = g;
for j =2:length(prev)
  path(j) = prev(g);
  g = prev(g);
  if (g == start_idx)
      break; 
  end 
 end
path = fliplr(path);
path = path'; 
end
