% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the
% function
world_points;
K = [314.1779 0         199.4848; ...
0         314.2218  113.7838; ...
0         0         1]; 
%%
n_v= sym('n_v',[3,1]);
n_g = sym ('n_g',[3,1]);
n_a = sym('n_a',[3,1]); 
vel = sym ('v',[3,1]);
omega = sym('w',[3,1]);
b_g = sym('bg',[3,1]); 
b_a = sym('ba',[3,1]); 
grav = [0;0;-9.81]; 
syms x y z phi theta psi  
accel_in = sym('a_m',[3,1]); 
n_bias_gyro = sym('n_bg',[3,1]); 
n_bias_accel = sym('n_ba',[3,1]); 
% syms y
% syms z
% syms phi
% syms theta
% syms psi
%% 
G = [cos(theta) 0  -cos(phi)*sin(theta);...
0           1   sin(phi); ...
sin(theta)  0   cos(phi)*cos(theta)];
%%
%%
R = [ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta)  -cos(phi)*sin(psi)  cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi); ...
    cos(theta)*sin(phi)+cos(psi)*sin(phi)*sin(theta)      cos(phi)*cos(psi)     sin(phi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);...
    -cos(phi)*sin(theta)                                    sin(phi)            cos(phi)*cos(theta)]; 
%% making functions 

x_dot1 = [ vel - n_v; (G)\(omega-n_g)];
x_dot2 = [ vel - n_v; (G)\(omega-b_g-n_g);grav+ R*(accel_in -b_g - n_a); n_bias_gyro;n_bias_accel]; 
% x_dot2 = [(G)\(omega-b_g-n_g);grav+ R*(accel_in -b_g - n_a); n_bias_gyro;n_bias_accel]; 
Xd_2 = matlabFunction(x_dot2);  
Xd_1 = matlabFunction(x_dot1); 
j1 = jacobian(x_dot1,[x,y,z,phi,theta,psi]); 
j2 = jacobian(x_dot1,[n_v; n_g]);
j3 = jacobian(x_dot2,[x;y;z;phi;theta;psi;vel; b_g; b_a;]); % depnding on vel
% j3 = jacobian(x_dot2,[x,y,z,phi,theta,psi, b_g', b_a']); %not depnding on vel
j4 = jacobian(x_dot2,[n_g; n_a; n_bias_gyro; n_bias_accel]); 

A_t1 = matlabFunction(j1); 
U_t1 = matlabFunction(j2);  
A_t2 = matlabFunction(j3); 
U_t2 = matlabFunction(j4); 


ekf1_handle = @(sensor, vic) ekf1(sensor, vic,Wp,K, A_t1, U_t1,Xd_1);
ekf2_handle = @(sensor) ekf2(sensor,Wp,K, A_t2,U_t2,Xd_2);
