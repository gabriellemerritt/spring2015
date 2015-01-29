function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

% pos = [0; 0; 0];
% vel = [0; 0; 0];
% acc = [0; 0; 0];
yaw = 0;
yawdot = 0;
rad = 1; 
x = rad*cos(t*2*pi*rad); 
y = rad*sin(t*2*pi*rad);
z = rad*t;  
pos = [x;y;z]; 
vel = [x/t;y/t;z/t]; 
acc = [x/(t^2);y/(t^2);z/(t^2)]; 

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
