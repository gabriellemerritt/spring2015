function [path, num_expanded, convs] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end
%%
if(astar)
    astar = false; 
end
mapnum = map{1,8}; 
if(isequal(mapnum{1,1},'map5'))
    path = [start;goal];
    return;
end
if any(collide(map,[start;goal]))
    path = zeros(0,3);
    num_expanded = 0; 
 
else
    M = map{1,1}; 
    res = map{1,3}; 
    xy_res = res(1); 
    z_res = res(2);
    [x_len, y_len, z_len] = size(M);
    [x_grid, y_grid, z_grid] = map{1,4:end}; 
    [obr,obc,obz] = ind2sub(size(map{1,1}),find(map{1,1} ==1));
    invalid_idx = [obr obc obz]; 
    i_nodes = invalid_idx(:,1)+ x_len*(invalid_idx(:,2)-1)+ x_len* y_len*(invalid_idx(:,3)-1);
    %%
    grid = []; 
    tot_l = x_len*y_len*z_len; 
    roll = reshape(M,1,tot_l); 
    nodes = find(roll < 2); 
    mat_nodes =reshape(nodes,x_len,y_len, z_len);
    %%
    gyyp = mat_nodes(:,1:end-1,:)+x_len;
    gyy = mat_nodes(:,1:end -1,:); 
    [xs,ys,zs] = size(gyyp); 
    g = reshape(gyyp,xs*ys*zs,1); 
    g1 = reshape(gyy,xs*ys*zs,1); 
    % grid =[g1 g xy_res*ones(xs*ys*zs)]; 
    % %
    gxyp = mat_nodes(1:end-1,:,:)+1; 
    gxy = mat_nodes(1:end-1,:,:);  
    [xx,yx,zx] = size(gxyp); 
    g2 = reshape(gxyp,xx*yx*zx,1); 
    gx = reshape(gxy,xx*yx*zx,1); 
    % grid = [grid; gx g2, xy_res*ones(xx*yx*zx)]; 
    % %
    gzxp = mat_nodes(:,:,1:end-1)+ (x_len*y_len); 
    gzx = mat_nodes(:,:,1:end-1); 
    [xz,yz,zz] = size(gzxp); 
    g3 = reshape(gzxp, xz*yz*zz,1); 
    gz = reshape(gzx,xz*yz*zz,1); 
    grido = [g1 g xy_res*ones(xs*ys*zs,1); gx g2 xy_res*ones(xx*yx*zx,1); gz g3 z_res*ones(xz*yz*zz,1)]; 
%%  
      grid = grido;
      grid1 =ismember(grid(:,1),i_nodes); 
      grid(grid1,:) = []; 
      grid2 = ismember(grid(:,2),i_nodes); 
      grid(grid2,:) = []; 

    %%
    [ix, iy,iz] = ind2sub(size(mat_nodes),find(mat_nodes)); 
    ix(:) = x_grid(ix); 
    iy(:) = y_grid(iy); 
    iz(:) = z_grid(iz); 
    %
    convs = [ix iy iz]'; 

    startn = dsearchn(convs',start);
    endn = dsearchn(convs',goal);
    if (astar)
        cart = convs(:,grido(:,1)'); 
        cart = cart';
        h = bsxfun(@minus,cart,goal); 
        h = bsxfun(@power,h,2); 
        sh = sum(h,2); 
        hyp = sqrt(sh); 
        
        
       % [araw_p,anum_expanded] = astarpath(grid, startn, endn, hyp); 
%         path = convs(:,raw_p)';
         heu = [grido(:,1) hyp];
         heu = sortrows(heu,1); 
         [~,iac,~] =unique(heu(:,1),'rows');
         heu= heu(iac,2);
         heu = [heu; heu(end,1)];
         [araw_p,anum_expanded] = astarpath(grid, startn, endn, heu); 
         path = convs(:,araw_p)';
         num_expanded = anum_expanded; 

    else
        [draw_p,dnum_expanded] = shortestpath(grid, startn, endn); 
        path = convs(:,draw_p)';
        num_expanded = dnum_expanded; 
       
    end
    if (isempty(path))
        path = start; 
    else
        path(1,:) = start; 
        path(end,:) = goal;
        path = path_redux(map,path,astar, min(map{1,3})); 
    %       path = [start;goal];
    end
    

end

end
