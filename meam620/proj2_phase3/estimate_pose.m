function [pos, eul] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
        % - is_ready: logical, indicates whether sensor data is valid
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor

world_points =varargin{1}; 
K = varargin{2}; 
%%
if ( isempty(sensor.id))
    pos = []; 
    eul = []; 
    error('im here'); 
else
% First construct A1 - A5 
X_w = [world_points.p0(1,:)', world_points.p1(1,:)', world_points.p2(1,:)', world_points.p3(1,:)', world_points.p4(1,:)']; 
Y_w  = [world_points.p0(2,:)', world_points.p1(2,:)', world_points.p2(2,:)', world_points.p3(2,:)', world_points.p4(2,:)']; 
%% For loop 
idx = sensor.id+1; 
x_p = [sensor.p0(1,:)', sensor.p1(1,:)', sensor.p2(1,:)',  sensor.p3(1,:)', sensor.p4(1,:)']; 
y_p = [sensor.p0(2,:)', sensor.p1(2,:)', sensor.p2(2,:)',  sensor.p3(2,:)', sensor.p4(2,:)'];
A = []; 
for i = 1:size(idx,2)
A = [A; 
    [X_w(idx(i),1) Y_w(idx(i),1) 1 0 0 0 -X_w(idx(i),1)*x_p(i,1) -Y_w(idx(i),1)*x_p(i,1) -x_p(i,1);
    
    0 0 0 X_w(idx(i),1) Y_w(idx(i),1) 1  -X_w(idx(i),1)*y_p(i,1) -Y_w(idx(i),1)*y_p(i,1) -y_p(i,1);
    
    X_w(idx(i),2) Y_w(idx(i),2) 1 0 0 0 -X_w(idx(i),2)*x_p(i,2) -Y_w(idx(i),2)*x_p(i,2) -x_p(i,2);
    
    0 0 0 X_w(idx(i),2) Y_w(idx(i),2) 1  -X_w(idx(i),2)*y_p(i,2) -Y_w(idx(i),2)*y_p(i,2) -y_p(i,2);
    
    X_w(idx(i),3) Y_w(idx(i),3) 1 0 0 0 -X_w(idx(i),3)*x_p(i,3) -Y_w(idx(i),3)*x_p(i,3) -x_p(i,3);
    
    0 0 0 X_w(idx(i),3) Y_w(idx(i),3) 1  -X_w(idx(i),3)*y_p(i,3) -Y_w(idx(i),3)*y_p(i,3) -y_p(i,3);
    
    X_w(idx(i),4) Y_w(idx(i),4) 1 0 0 0 -X_w(idx(i),4)*x_p(i,4) -Y_w(idx(i),4)*x_p(i,4) -x_p(i,4);
    
    0 0 0 X_w(idx(i),4) Y_w(idx(i),4) 1  -X_w(idx(i),4)*y_p(i,4) -Y_w(idx(i),4)*y_p(i,4) -y_p(i,4);
    
    X_w(idx(i),5) Y_w(idx(i),5) 1 0 0 0 -X_w(idx(i),5)*x_p(i,5) -Y_w(idx(i),5)*x_p(i,5) -x_p(i,5);
    
    0 0 0 X_w(idx(i),5) Y_w(idx(i),5) 1  -X_w(idx(i),5)*y_p(i,5) -Y_w(idx(i),5)*y_p(i,5) -y_p(i,5)
    ]]; 
end 
%% Homography 

 [U,S,V] = svd(A); % last column of V is null space of A 
 h = V(:,end); 
 H = reshape(h,[3,3]); 
 H= H'; 
 H = H/H(3,3);
 H_cam = inv(K)*H; 
 %% Rotation Matrix World to Camera
 H_svd = [H_cam(:,1), H_cam(:,2), (cross(H_cam(:,1), H_cam(:,2)))]; 
 [U_2,S_2,V_2] =svd(H_svd) ;
 calc = U_2*(V_2'); 
 R_w2cam = U_2*[1 0 0; 0 1 0; 0 0 det(calc)]*(V_2');  
 %% Transform World to Camera
 T_w2cam = H_cam(:,3)/(norm(H_cam(:,1))); 
 
 H_w2cam = [R_w2cam T_w2cam; 0 0 0 1];
%% quad to cam XYZ 
imuXYZ = [-0.04, 0.0, -0.03];
R_quad2cam = [cosd(-45), -sind(-45), 0; sind(-45), cosd(-45), 0; 0, 0,1];
X_rot = [1 0 0 ;0 cosd(180) -sind(180); 0 sind(180) cosd(180)]; 
R_quad2cam = R_quad2cam* X_rot; %% potential problem area  
T_quad2cam = imuXYZ; %3cm 


%% cam to quad 
H_cam2quad = inv([ R_quad2cam T_quad2cam' ; 0 0 0 1]); 
%%

final_form =  H_cam2quad *H_w2cam; %% swap order 
R_final = final_form(1:3,1:3)';  % Rotation in world frame of the quad 
T_final = final_form(1:3,end);  % -R' T is  
pos = -R_final*T_final; 
[phi, theta, psi]  = rot_RPY(R_final');
eul = [phi,theta,psi]';
end
end
