function [ desired_state ] = trajectory_generator1(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0 
persistent map0;  
persistent path0;

if nargin > 2 
   map0 = map; 
   path0 = path{1,1}; 
   return;
end 
% path = path_redux(path0,1); 
% linspace <== find smallest res between x y and z res)

xmax =10;
tz = 0;
tf = 15; 
[n,~] = size(path0);
div =tf/n; % points needed to travel per second
grid =1/div; % seconds per point
idx = floor((t - tz)/div)+1; 

t_init = 0; 
t_final = tf; 
bt = t /tf; 
pos = ((1-bt)^2)*path0(1,:) + 2*(1 -bt)*bt*path0(idx,:)+bt^2*path0(end,:);
vel = 2*(1-bt)*(path0(idx,:) - path0(1,:)) + 2*bt*(path0(end,:) - path0(idx,:)); 
acc = 2*(path0(end,:) -2*path0(idx,:) + path0(1,:)); 
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end







