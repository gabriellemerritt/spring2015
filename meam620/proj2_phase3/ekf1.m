function [X, Z] = ekf1(sensor, vic, varargin)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%

% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; roll; pitch; yaw; other states you use]
%     we will only take the first 6 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
persistent mu;  % previous state
persistent sig; % previous covars
persistent tv_old;


world_points = varargin{1};
K = varargin{2};
A_t1 = varargin{3};
U_t1 = varargin{4};
Xd = varargin{5};

if(isempty(mu) && sensor.is_ready && ~isempty(sensor.id))
    %estimate initial pose from vicon vel readings
    tv_old = vic.t;
    
    [est_pos, est_eul]=estimate_pose(sensor,world_points,K);
    X = [est_pos ;est_eul];
    Z = [est_pos ;est_eul];
    sig = eye(6);
    mu = X;
elseif (isempty(mu) && isempty(sensor.id))
    X = [];
    Z = [];
    
elseif (isempty(mu) && ~sensor.is_ready)
    X = [];
    Z = [];
    
else  %% need to make a case where we have 0 vicon readings or 0 sensor
    if (~isempty(vic))
        %% vars
        omega_m = vic.vel(4:6);
        v_m = vic.vel(1:3);
        dtv = vic.t - tv_old;
        phi = mu(4);
        theta = mu(5);
        psi = mu(6);
        %% biases / noises
        
        A_t= A_t1(0,0,0,phi,theta,omega_m(1),omega_m(2),omega_m(3));
        U_t = U_t1(phi,theta);
        
        %% setting up for prediction
        F_t = eye(6) + A_t*dtv;
        V_t = U_t*dtv;
        X_d = Xd(0,0,0,0,0,0,phi,theta,v_m(1),v_m(2),v_m(3),omega_m(1),omega_m(2),omega_m(3));
        Q = eye(6)*1;
        %% prediction
        mu = mu + X_d*dtv;
        sig = (F_t * sig *F_t')+ (V_t*Q*V_t');
    end
    %% update
    if(~isempty(sensor))
        [est_pos, est_eul] = estimate_pose(sensor,world_points,K);
        z = [est_pos; est_eul];
        C = eye(6);
        W = eye(6);
        R = eye(6);
        
        K_t = sig*C'/(C*sig*C' + W*R*W');
        mu = mu + K_t*(z - C*mu);
        sig = sig - K_t*C*sig;
    end
    %mu 6x1
    X = mu;
    Z = z;
    tv_old = vic.t;
    
end

end


