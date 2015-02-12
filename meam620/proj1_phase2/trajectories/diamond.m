function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
tf = 10; 
sq2 = sqrt(2);
x = [ 0,.25,.5,.75,1];
xdot = [ 0,0,0,0,0]; 
xddot = [0,0,0,0,0]; 
y = [0, sq2, 0 , -sq2,0];
ydot = [0,0,0,0,0];
yddot = [0,0,0,0,0];
z= [ 0 , sq2, 2*sq2, sq2,0];  
zdot= [0, 0, 0, 0,0]; 
zddot =[0,0,0,0,0];
pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];
tmat = [0:tf/5:tf];
yaw = 0;
yawdot = 0;
t_init =0; 
t_final = 2;
i = 1; 
i2 = 2; 
if t <=tmat(2);
    t_init = 0; 
    t_final = tf/5;
    i = 1; 
    i2 = 2; 
elseif t <= tmat(3)
    i = 2; 
    i2 = 3; 
    t_init = tmat(2);
    t_final = tmat(3);
elseif t <= tmat(4) 
    i = 3; 
    i2 = 4; 
    t_init = tmat(3); 
    t_final = tmat(4); 
elseif t <= tmat(5) 
    i = 4; 
    i2 = 5; 
    t_init = tmat(4); 
    t_final = tmat(5); 
end 

       Q  = [  1 t_init t_init^2 t_init^3 t_init^4 t_init^5; 
            0 1 2*t_init 3*t_init^2 4*t_init^3 5*t_init^4;  
            0  0  2 6*t_init 12*t_init^2 20*t_init^3;  
            1 t_final t_final^2 t_final^3 t_final^4 t_final^5;  
            0 1 2*t_final 3*t_final^2 4*t_final^3 5*t_final^4; 
            0 0 2 6*t_final 12*t_final^2 20*t_final^3]; 
       x_b =[ x(i);xdot(i);xddot(i);x(i2);xdot(i2);xddot(i2)]; 
       y_b = [y(i);ydot(i);yddot(i);y(i2);ydot(i2);yddot(i2)]; 
       z_b = [z(i);zdot(i);zddot(i);z(i2);zdot(i2);zddot(i2)];
       cond = [ x_b y_b z_b]
       a = Q\cond 
       pos(1) = a(1,1) +a(2,1)*t +a(3,1)*t^2 + a(4,1)*t^3 + a(5,1)*t^4 +a(6,1)*t^5;  
       pos(2) = a(1,2) +a(2,2)*t +a(3,2)*t^2 + a(4,2)*t^3 + a(5,2)*t^4 +a(6,2)*t^5;  
       pos(3) = a(1,3) +a(2,3)*t +a(3,3)*t^2 + a(4,3)*t^3 + a(5,3)*t^4 +a(6,3)*t^5;  
       vel(1) = a(2,1) + 2*a(3,1)*t + 3*a(4,1)*t^2 + 4*a(5,1)*t^3 + 5*a(6,1)*t^4 ; 
       vel(2) = a(2,2) + 2*a(3,2)*t + 3*a(4,2)*t^2 + 4*a(5,2)*t^3 + 5*a(6,2)*t^4 ; 
       vel(3) = a(2,3) + 2*a(3,3)*t + 3*a(4,3)*t^2 + 4*a(5,3)*t^3 + 5*a(6,3)*t^4 ; 
       acc(1) = 2*a(3,1) +6*a(4,1)*t + 12*a(5,1)*t^2 + 20*a(6,1)*t^3 ; 
       acc(2) = 2*a(3,2) +6*a(4,2)*t + 12*a(5,2)*t^2 + 20*a(6,2)*t^3 ;
       acc(3) = 2*a(3,3) +6*a(4,3)*t + 12*a(5,3)*t^2 + 20*a(6,3)*t^3 ;


if t > tmat(5) 
   pos = [1; 0; 0];
   vel = [0; 0; 0];
   acc = [0; 0; 0];
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
