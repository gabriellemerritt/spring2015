function [X, Z] = ekf2(sensor, varargin)
% EKF2 Extended Kalman Filter with IMU as inputs
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
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order

%     [x; y; z; vx; vy; vz; roll; pitch; yaw; other states you use]
%     we will only take the first 9 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement

persistent mu;  % previous state
persistent sig; % previous covars
persistent ts_old;
persistent b_g;
persistent b_a;
persistent n_a;
persistent n_g;
persistent n_v;


world_points = varargin{1};
K =    varargin{2};
A_t2 = varargin{3};
U_t2 = varargin{4};
Xd =   varargin{5};


if(isempty(mu) && sensor.is_ready)
    %estimate initial pose from vicon vel readings
    [est_pos, est_eul]=estimate_pose(sensor,world_points,K);
    %     [est_vel, est_omg] = estimate_vel(sensor,world_points,K);
    b_a = [0;0;0];
    b_g = [0;0;0];
    sig = eye(15);
    n_v = [0;0;0];
    n_a = [0;0;0];
    n_g =[0;0;0];
    ts_old = sensor.t;
    mu = [est_pos ;est_eul; 0;0;0;b_g;b_a];
    X = [mu(1:3); mu(7:9); mu(4:6)];
    Z = [est_pos;est_eul];
elseif (isempty(mu) && isempty(sensor.id))
    X = [];
    Z = [];
elseif (isempty(mu) && ~sensor.is_ready)
    X = [];
    Z = [];
    
else  %% need to make a case where we have 0 vicon readings or 0 sensor
    if(~isempty(sensor.id))
        [est_pos, est_eul] = estimate_pose(sensor,world_points,K);
        z = [est_pos;est_eul];
        C = [eye(3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3); zeros(3,3) eye(3) zeros(3,3) zeros(3,3) zeros(3,3); zeros(3,3) zeros(3,3) eye(3) zeros(3,3) zeros(3,3)];
        C = C(1:6,:);
        %         W = eye(9);
        R = eye(9);
        W = eye(6);
        R = R(1:6,1:6);
        R = diag([0.0159 0.0120  0.0116  0.0020  0.0022  0.0021].^2); 
%         
        dts = sensor.t - ts_old;
        phi =   mu(4);
        theta = mu(5);
        psi =   mu(6);
        accel = sensor.acc;
        omega_m = sensor.omg;
        
        %% biases / noises
        
        %        A_t= A_t2(accel(1),accel(2),accel(3),b_g(1),b_g(2),b_g(3),0,0,0,0,0,0,phi,psi,theta,omega_m(1),omega_m(2),omega_m(3));
        A_t= A_t2(accel(1),accel(2),accel(3),b_g(1),b_g(2),b_g(3),0,0,0,0,0,0,phi,psi,theta,omega_m(1),omega_m(2),omega_m(3));
        U_t = U_t2(phi,psi,theta);
        %% setting up for prediction
        F_t = eye(size(A_t,1)) + A_t*dts;
        V_t = U_t*dts;
        X_d = Xd(accel(1),accel(2),accel(3),b_g(1),b_g(2),b_g(3),n_a(1),n_a(2),n_a(3),0,0,0,0,0,0,n_g(1),n_g(2),n_g(3),n_v(1),n_v(2),n_v(3),phi,psi,theta,mu(7),mu(8),mu(9),omega_m(1),omega_m(2),omega_m(3));
        %        X_d = Xd(accel(1),accel(2),accel(3),b_g(1),b_g(2),b_g(3),n_a(1),n_a(2),n_a(3),0,0,0,0,0,0,n_g(1),n_g(2),n_g(3),phi,psi,theta,omega_m(1),omega_m(2),omega_m(3));
        
       Q = eye(12);
% %         Q = eye(12)*1e-5;
%         Q = diag([0.0159^2 0.0120^2  0.0116^2  0.0020^2  0.0022^2  0.0021^2 1e-5 1e-5 1e-5 1e-5 1e-5 1e-5]);

        %% prediction
        mu = mu + X_d*dts;
        sig = (F_t * sig *F_t')+ (V_t*Q*V_t');
        K_t = sig*C'/(C*sig*C' + W*R*W');
        mu = mu + K_t*(z(1:6) - C*mu);
        sig = sig - K_t*C*sig;
    end
    %mu 6x1
    X = [mu(1:3); mu(7:9); mu(4:6)];
    Z = [];
    ts_old = sensor.t;
end

end
