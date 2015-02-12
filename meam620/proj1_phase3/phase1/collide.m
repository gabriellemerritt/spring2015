function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
% xmin = min(map{1,2}(1,:)); 
% ymin = min(map{1,2}(2,:)); 
% zmin = min(map{1,2}(3,:)); 
% 
% [x_len, y_len, z_len] =size(map{1,3}); 
% pts = points(:,1)+1+xmin+x_len*(points(:,2)-ymin)+x_len*y_len*(points(:,3)-zmin); 
% pts = [rem(pts,x_len) rem(pts,y_len) rem(pts,z_len)]; 
% 
% for(i=1:length(pts))
%    C(i) = map{1,3}(pts(i,1),pts(i,2),pts(i,3));
% end
% 
if (~map{1,1})
    C = zeros(length(points(:,1)),1); 
end

boundCell =map{1,2};
margin = map{1,9}; 
[board_bound, xmin, ymin, zmin, xmax,ymax, zmax,r,g,b] = boundCell{1,:}; 
for(j =1:length(board_bound))
    if(isequal(board_bound{j,1},'boundary'))
        idx =j; 
        break;
    end
end
xmin(idx) = []; 
ymin(idx) =[]; 
zmin(idx)=[]; 
xmax(idx) =[]; 
ymax(idx) =[]; 
zmax(idx) = []; 
 for i =1:length(points(:,1)) 
   x = (xmin- margin)<=points(i,1) & (xmax+margin)>=points(i,1); 
   y = (ymin- margin)<=points(i,2) & (ymax+margin)>=points(i,2); 
   z = (zmin- margin)<=points(i,3)&  (zmax+margin)>=points(i,3); 
   C(i) = any(x&y&z); 
   
 end 
C = C'; 
