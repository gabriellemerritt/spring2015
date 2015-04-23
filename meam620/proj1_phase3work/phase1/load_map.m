function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

%1:xy_res:(xmax/xyres)+1 
%% Loading file in to cell
clc 


fileID =fopen(filename,'r'); 
format_spec= '%s%f%f%f%f%f%f%f%f%f';
% C= textscan(fileID,format_spec);
C=textscan(fileID,format_spec,'whitespace',' ', 'CommentStyle',{'#'});
[board_bound, xmin, ymin, zmin, xmax, ymax, zmax,r,g,b] = C{1,:}; 
XYZ = [xmin ymin zmin xmax ymax zmax]; 
for l = 1:length(board_bound)
    fix =strsplit(board_bound{l,1},{'k','y'}); 

    if(~isequal(fix{1,2},''))
       hack = str2double(fix{1,2});
       fix2 = [hack XYZ(l,1:end -1)];
       XYZ(l,:) = fix2;
    end
end
% if(any(board_bound{1,1} == '#'))
%     C = textscan(fileID,format_spec);
%     fclose(fileID); 
%     [board_bound, xmin, ymin, zmin, xmax,ymax, zmax,r,g,b] = C{1,:}; 
% else
    fclose(fileID); 
% end
r = r/255;
g = g/255; 
b = b/255; 
%% 
%plot grid
idx = 0 ; 
for i=1:length(board_bound)
    bon = strfind(board_bound{i,1},'boundar');
    if(~isempty(bon))
          
%           voxel([xmin(i),ymin(i),zmin(i)], [xmax(i)-xmin(i),ymax(i)-ymin(i),zmax(i)-zmin(i)],[0 1 1],0);
          idx = i; 
%           continue; 
         break; 
    end 
    
%     hold on 
%     voxel([xmin(i),ymin(i),zmin(i)], [xmax(i)-xmin(i),ymax(i)-ymin(i),zmax(i)-zmin(i)],[r(i) g(i) b(i)],1);    
end
%% 
   
xmin = XYZ(:,1); 
ymin = XYZ(:,2); 
zmin = XYZ(:,3); 
xmax = XYZ(:,4); 
ymax = XYZ(:,5); 
zmax = XYZ(:,6); 

x_grid = xmin(idx):xy_res:xmax(idx);
y_grid = ymin(idx):xy_res:ymax(idx); 

z_grid = zmin(idx):z_res:zmax(idx); 
%  
% x_dif = (xmax(idx) - xmin(idx)); 
% y_dif = ymax(idx) - ymin(idx); 
% z_dif = zmax(idx) - zmin(idx); 
if (x_grid(end) ~=xmax(idx))
    nxmax =x_grid(end)+xy_res;
    x_grid = xmin(idx):xy_res:nxmax;
end
if (y_grid(end)~=ymax(idx))
    nymax = y_grid(end)+xy_res;
    y_grid = ymin(idx):xy_res:nymax;
end
if (z_grid(end)~=zmax(idx))
    nzmax = z_grid(end)+z_res;
     z_grid = zmin(idx):xy_res:nzmax;
end
    
% z_grid
% zmax(idx)
% zmin(idx)
%%
% Occupancy Grid
M = false(length(x_grid), length(y_grid), length(z_grid));
for(i=1:length(board_bound))
    bon = strfind(board_bound{i,1},'boundar');
    if(~isempty(bon)) 
        continue; 
    end 
    mx = (xmin(i)-margin)<=x_grid & (xmax(i)+margin)>=x_grid; 
    my = (ymin(i)-margin)<=y_grid & (ymax(i)+margin)>=y_grid;
    mz = (zmin(i)-margin)<=z_grid & (zmax(i)+margin)>=z_grid; 
    [Y,X,Z] = meshgrid(my,mx,mz); 
    M = (X&Y&Z)| M;
    
end 

%%  
%More plotting 
% for(k=1:length(z_grid)-1)
%     for(j=1:length(y_grid)-1)
%         for(i=1:length(x_grid)-1)
%               voxel([x_grid(i), y_grid(j), z_grid(k)], [x_grid(i+1)- x_grid(i), y_grid(j+1)-y_grid(j), z_grid(k+1)-z_grid(k)] ,[r(2) g(2) b(2)],0);
%         end
%     end
% end

%%
data = [ xmin ymin zmin xmax ymax zmax]; 
M = M(1:end-1,1:end-1,1:end-1); 
map = cell(1,2); 
c = cell(1,10); 
for m =1:6
    c{1,m+1} = data(:,m); 
end
c{1,1} = board_bound; 
map{1,1} = M; 
map{1,2} = c;
map{1,3}= [xy_res z_res]; 
map{1,4} = x_grid; 
map{1,5} = y_grid; 
map{1,6} = z_grid; 
% map{1,7}=[ xmin ymin zmin xmax ymax zmax r g b ];  
% map{1,8} = board_bound;
map{1,7} = 0; 
map{1,8} = 0; 
map{1,9} = margin; 

% map{1,10} = mapnum;

end
    