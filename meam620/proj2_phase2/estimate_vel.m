function [vel, omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor
persistent loop_num; 
persistent corners; 
persistent pointTracker;  
if (loop_num ==1 )|| (corners < 200)  
    corners = corner(sensor.img,1000);
    pointTracker = vision.PointTracker; 
    initialize(pointTracker,corners, sensor.img); 
end 
[tracked_points, valid] = step(pointTracker,sensor.img); 
valid_points = tracked_points(valid,:s); 
%% do transform to find Vx, Vy, Vz in real world from image
%%need to find Depth 

vel = zeros(3,1);
omg = zeros(3,1);

end
