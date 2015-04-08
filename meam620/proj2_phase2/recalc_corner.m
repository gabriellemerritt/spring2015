function [corners,counter] = recalc_corner(pointTracker,sensor)

corners = corner(sensor.img,1000); 
%need to release 
initialize(pointTracker,corners, sensor.img); 
counter =1; 
end
