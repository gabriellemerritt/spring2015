function [ desired_state ] = trajectory_generator(t, qn, map, path)
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
persistent lengths; 
persistent tvector; 
persistent tf; 


if nargin > 2 
   map0 = map; 
   path0 = path{1,1}; 
   lengths =[]; 
   for i =1:size(path0,1)-1 
        lengths(i) = nthroot(norm(path0(i+1,:) -path0(i,:)),3)*1.5; 
   end
    tvector = [0 cumsum(lengths)];
    tf = tvector(end);
   return;
end 
% path = path_redux(path0,1); 
% linspace <== find smallest res between x y and z res)

i =1; 
i2 = 2; 
tz = 0;
[n,~] = size(path0); 
div = (tvector > t);   % points needed to travel per second
% grid =1/div; % seconds per point
[~ ,id] = max(div);  
if (id == 1) 
    pos = path0(1,:); 
    vel = [0,0,0]; 
    acc = [0,0,0]; 
    yaw = 0;
    yawdot = 0;
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
    return; 
end 

    
idx = id -1; 
% if idx >= size(path0,1)
%     pos = path0(end,:);  
%     vel = [0,0,0]; 
%     acc = [0,0,0]; 
% else
%     t_init =tz+(idx-1)*div; 
%     t_final = idx*div;
        t_init = tvector(idx);
        t_final = tvector(idx+1); 
        x = [path0(idx,1) path0((idx+1),1)]; 
        xdot = [0 0];
        xddot= [0 0];
        y = [path0(idx,2) path0((idx+1),2)];
        ydot = [0 0];
        yddot =[0 0];
        z= [path0(idx,3) path0((idx+1),3)];  
        zdot= [0 0];
        zddot = [0 0]; 

    %% calculating coef
        Q  = [  
            1 t_init t_init^2 t_init^3 t_init^4 t_init^5;
            
            0 1 2*t_init 3*t_init^2 4*t_init^3 5*t_init^4;  
            
            0  0  2 6*t_init 12*t_init^2 20*t_init^3;  
            
            1 t_final t_final^2 t_final^3 t_final^4 t_final^5;  
            
            0 1 2*t_final 3*t_final^2 4*t_final^3 5*t_final^4; 
            
            0 0 2 6*t_final 12*t_final^2 20*t_final^3
            ]; 
        
            x_b =[ x(i);xdot(i);xddot(i);x(i2);xdot(i2);xddot(i2)]; 
            y_b = [y(i);ydot(i);yddot(i);y(i2);ydot(i2);yddot(i2)]; 
            z_b = [z(i);zdot(i);zddot(i);z(i2);zdot(i2);zddot(i2)];
            cond = [ x_b y_b z_b];
            a = Q\cond; 

            pos(1) = a(1,1) +a(2,1)*t +a(3,1)*t^2 + a(4,1)*t^3 + a(5,1)*t^4 +a(6,1)*t^5; 
            pos(2) = a(1,2) +a(2,2)*t +a(3,2)*t^2 + a(4,2)*t^3 + a(5,2)*t^4 +a(6,2)*t^5; 
            pos(3) = a(1,3) +a(2,3)*t +a(3,3)*t^2 + a(4,3)*t^3 + a(5,3)*t^4 +a(6,3)*t^5; 
            vel(1) = a(2,1) + 2*a(3,1)*t + 3*a(4,1)*t^2 + 4*a(5,1)*t^3 + 5*a(6,1)*t^4 ; 
            vel(2) = a(2,2) + 2*a(3,2)*t + 3*a(4,2)*t^2 + 4*a(5,2)*t^3 + 5*a(6,2)*t^4 ; 
            vel(3) = a(2,3) + 2*a(3,3)*t + 3*a(4,3)*t^2 + 4*a(5,3)*t^3 + 5*a(6,3)*t^4 ; 
            acc(1) = 2*a(3,1) +6*a(4,1)*t + 12*a(5,1)*t^2 + 20*a(6,1)*t^3 ; 
            acc(2) = 2*a(3,2) +6*a(4,2)*t + 12*a(5,2)*t^2 + 20*a(6,2)*t^3 ;
            acc(3) = 2*a(3,3) +6*a(4,3)*t + 12*a(5,3)*t^2 + 20*a(6,3)*t^3 ;


% end
if ( t >=tf) 
    pos = path0(end,:);  
    vel = [0,0,0]; 
    acc = [0,0,0]; 
end
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end







