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

persistent corners; 
persistent pointTracker;  
persistent counter; 
persistent p_corners;
persistent t_old; 
world_points = varargin{1};
K = varargin{2}; 
%%
if ( isempty(sensor.id))
   counter = counter +1;

    vel = []; 
    omg = [];
    t_old = sensor.t; 
elseif (isempty(corners))  
    corners = corner(sensor.img,1000);
    size(corners)
    pointTracker = vision.PointTracker; 
    initialize(pointTracker,corners, sensor.img); 
    track = [ corners ones(length(corners),1)]; 
    p_corners = (K)\track'; 
    counter = 1; 
    vel = zeros(3,1); 
    omg = zeros(3,1); 
    t_old = sensor.t; 
else
   counter = counter +1;
      t_new = sensor.t;
%     t_step = counter/50; 
      t_step = t_new - t_old; 
%     track = [ corners ones(length(corners),1)]; 
%     p_corners = (K)\track';

    [tracked_points, valid] = step(pointTracker,sensor.img); 
%     corners = corners(valid,:);  % corners is n x 2
     

%     p_corners = p_corners(:, valid); %p_corners 3 x n 
 

    %% do transform to find Vx, Vy, Vz in real world from image
    %% convert points into camera frame  
    tracked_points(:,3) = 1; 
    tracked_points = tracked_points'; 
    p =(K)\tracked_points;  %% Inv(K)*points

    % p = [x y]; 
    p_dot = (p - p_corners)/t_step;  
% 
%     p = p(:,valid); 
    p_dot = p_dot(:,valid); 
    p = p(:,valid); 
    %% find cam to world rotation  (in order to find depth) 
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
     H_cam = (K)\H; 
     %% Rotation Matrix World to Camera
     H_svd = [H_cam(:,1), H_cam(:,2), (cross(H_cam(:,1), H_cam(:,2)))]; 
     [U_2,S_2,V_2] =svd(H_svd) ;
     calc = U_2*(V_2'); 
     R_w2cam = U_2*[1 0 0; 0 1 0; 0 0 det(calc)]*(V_2'); 
%      R_w2cam = R_w2cam';
    %% Transform World to Camera
     T_w2cam = H_cam(:,3)/(norm(H_cam(:,1)))'; 
     

    %% Depth 
    lambda = (R_w2cam(:,3)'*T_w2cam)./ (R_w2cam(:,3)'*p); 
    depth = lambda;  
    %% calculate vel and omg 
    A_vel = [];
    for j=1:length(p_dot) %x's first 
        A_vel =[A_vel; [ (-1/depth(j)) 0 (p(1,j)/depth(j)) p(1,j)*p(2,j) -(1+p(1,j)^2) p(2,j)]]; 
    end
    for k = 1:length(p_dot) %y's second
       A_vel =[A_vel; [  0   (-1/depth(k)) (p(2,k)/depth(k))  (1+p(2,k)^2) -p(1,k)*p(2,k) -p(1,k)]]; 
    end 
    b = [p_dot(1,:)';p_dot(2,:)'];
    vels = A_vel\b; 
    Tc_b = [-0.04, 0.0, -0.03]';% T body in camera frame
    Vw_c = R_w2cam * ( vels(1:3) + (cross(vels(4:6),Tc_b)));  %V camera in world frame
    Ow_c  =R_w2cam*vels(4:6); 
    
    %%
    

    vel = Vw_c;
    omg = Ow_c;
    t_old = t_new; 
   if (1) 
        corners = corner(sensor.img,1000);
        pointTracker = vision.PointTracker; 
        initialize(pointTracker,corners, sensor.img);
        track = [corners ones(length(corners),1)]; 
        p_corners = (K)\track';
  
        counter = 1; 
%     [corners, counter] = recalc_corner(pointTracker, sensor);  
   end 

end
end
