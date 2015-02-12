function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
%initial conditions 
% pos at t = 0 
% v at t = 0 
% a at t = 0 
%final conditions 
%post at t = final
% v at t = final 
% a at t = final
%q(t) = a0 +a1*t +a2*t^2 + a3*t^3 + a4*t^4 +a5*t^5  
%qdot(t) = a1*t + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4  
%qddot(t) = 2*a2 +6*a3*t + 12*a4*t^2 + 20*a5*t^3 

%do another quintic for z 
z_init = 0; 
zdot_init = 0; 
zddot_init = 0; 
z_final = 2.5; 
zdot_final = 0; 
zddot_final =0;

t_final= 11;
t_init = 0; 
theta_init = 0; 
thetadot_init = 0; 
thetaddot_init = 0; 
theta_final = 2*pi; 
thetadot_final = 0; 
thetaddot_final = 0;

    Q  = [  1 t_init t_init^2 t_init^3 t_init^4 t_init^5; 
            0 1 2*t_init 3*t_init^2 4*t_init^3 5*t_init^4;  
            0  0  2 6*t_init 12*t_init^2 20*t_init^3;  
            1 t_final t_final^2 t_final^3 t_final^4 t_final^5;  
            0 1 2*t_final 3*t_final^2 4*t_final^3 5*t_final^4; 
            0 0 2 6*t_final 12*t_final^2 20*t_final^3]; 

    conditions = [theta_init;thetadot_init;thetaddot_init;theta_final;thetadot_final;thetaddot_final]; 
    zconditions = [z_init;zdot_init;zddot_init;z_final;zdot_final;zddot_final]; 
    cond = [conditions zconditions]; 
    a = Q\cond; 
    

theta = a(1,1) +a(2,1)*t +a(3,1)*t^2 + a(4,1)*t^3 + a(5,1)*t^4 +a(6,1)*t^5;  
theta_dot= a(2,1)+ 2*a(3,1)*t + 3*a(4,1)*t^2 + 4*a(5,1)*t^3 + 5*a(6,1)*t^4  ;
theta_ddot = 2*a(3,1) +6*a(4,1)*t + 12*a(5,1)*t^2 + 20*a(6,1)*t^3 ;

z = a(1,2) +a(2,2)*t +a(3,2)*t^2 + a(4,2)*t^3 + a(5,2)*t^4 +a(6,2)*t^5 ; 
z_dot= a(2,2) + 2*a(3,2)*t + 3*a(4,2)*t^2 + 4*a(5,2)*t^3 + 5*a(6,2)*t^4 ; 
z_ddot = 2*a(3,2) +6*a(4,2)*t + 12*a(5,2)*t^2 + 20*a(6,2)*t^3 ;

yaw = 0;
yawdot = 0;
rad = 5; 
x = rad*cos(theta); 
y = rad*sin(theta);
% z = 0+ -6.6613*1e-18*t+9.4868*1e-19*t^2 + 1.7769*1e-04*t^3 +3.5615*1e-06*t^4 +  -1.7077*1e-07*t^5;
vx = -rad*sin(theta)* theta_dot; 
vy =  rad*cos(theta)* theta_dot; 
% vz =  6.6613*1e-18+ 2*9.4868*1e-19*t + 3*1.7769*1e-04*t^2 +4*3.5615*1e-06*t^3 +  5*-1.7077*1e-07*t^4;
ax = -rad*cos(theta)*theta_dot + -rad*sin(theta)*theta_ddot;
ay = -rad*sin(theta)*theta_dot + rad*cos(theta)*theta_ddot; 
if t > t_final 
    x = 5; 
    y = 0;
    z = 2.5; 
    vx = 0; 
    vy = 0; 
    z_dot = 0;
    ax = 0;
    ay = 0; 
    z_ddot = 0; 
end
pos = [x;y;z]; 
vel = [vx;vy;z_dot]; 
acc = [ax;ay;z_ddot]; 


% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
