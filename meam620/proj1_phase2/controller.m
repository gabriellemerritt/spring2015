function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

omega = qd{qn}.omega; % p; q ; r2

% Desired roll, pitch and yaw 
pitch_current = qd{qn}.euler(1); %phi theta psi
roll_current = qd{qn}.euler(2); 
yaw_current = qd{qn}.euler(3); 

 
x_traj =qd{qn}.pos_des(1); 
y_traj =qd{qn}.pos_des(2); 
z_traj = qd{qn}.pos_des(3); 
xdot_traj = qd{qn}.vel_des(1);
ydot_traj = qd{qn}.vel_des(2);
zdot_traj = qd{qn}.vel_des(3);
xddot_traj = qd{qn}.acc_des(1);
yddot_traj =  qd{qn}.acc_des(2);
zddot_traj =  qd{qn}.acc_des(3);
yaw_traj = qd{qn}.yaw_des; 
yawdot_traj = qd{qn}.yawdot_des; 
%% 
x_current= qd{qn}.pos(1);
y_current = qd{qn}.pos(2);
z_current =qd{qn}.pos(3);
xdot_current =qd{qn}.vel(1);
ydot_current =qd{qn}.vel(2);
zdot_current =qd{qn}.vel(3);

%%
xddot_des = xddot_traj + params.kdx*(xdot_traj - xdot_current) + params.kpx*(x_traj -x_current);
yddot_des = yddot_traj + params.kdy*(ydot_traj - ydot_current) + params.kpy*(y_traj - y_current);
zddot_des = zddot_traj + params.kdz*(zdot_traj - zdot_current) + params.kpz*(z_traj - z_current);

phi_traj = 1/params.grav *(xddot_des*sin(yaw_traj) - yddot_des*cos(yaw_traj));
theta_traj = 1/params.grav*(xddot_des*cos(yaw_traj) + yddot_des*sin(yaw_traj)); 

%% use controller to find 
phi_des = 0; 
theta_des = 0; 
psi_des = 0; 
%%
% F1 =params.kf*omega^2;

% Thurst
F = params.mass*params.grav + params.mass*(zddot_des+ params.kdz*(zdot_traj - zdot_current) + params.kpz*(z_traj - z_current));
F = min(params.maxF, F);
F = max(params.minF, F); 

% Moment
M    = zeros(3,1); % You should fill this in
M(1)=  params.kpphi*(phi_traj - pitch_current) + params.kdphi*(-omega(1));
M(2) = params.kptheta*(theta_traj - roll_current) + params.kdtheta*(-omega(2));
M(3) = params.kppsi*(yaw_traj - yaw_current) + params.kdpsi*(yawdot_traj -omega(3));
% F = 0; 
M(1) = min(params.maxangle,M(1)); 
M(2) = min(params.maxangle,M(2)); 
M(3) = min(params.maxangle,M(3)); 



% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
