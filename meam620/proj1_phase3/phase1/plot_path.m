function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
bounds = map{1,7}; 
board_bound = map{1,8}; 
xmin = bounds(:,1); 
ymin = bounds(:,2); 
zmin =  bounds(:,3);
xmax =  bounds(:,4);
ymax =  bounds(:,5);
zmax =  bounds(:,6); 
r = bounds(:,7); 
g = bounds(:,8); 
b = bounds(:,9); 

for i=1:length(board_bound)
    if(isequal(board_bound{i,1},'boundary'))
%         disp('I should skip'); 
          voxel([xmin(i),ymin(i),zmin(i)], [xmax(i)-xmin(i),ymax(i)-ymin(i),zmax(i)-zmin(i)],[0 1 1],0);
          continue; 
    end   
    hold on 
    voxel([xmin(i),ymin(i),zmin(i)], [xmax(i)-xmin(i),ymax(i)-ymin(i),zmax(i)-zmin(i)],[r(i) g(i) b(i)],1);
end

scatter3(path(:,1),path(:,2),path(:,3),'g'); 

end