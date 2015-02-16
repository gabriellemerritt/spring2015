function params = nanoplus()
% NANOPLUS basic parameters for nanoplus quadrotor
%   nanoplus outputs the basic parameters (mass, gravity and inertia) of
%   the quadrotor nanoplus

m = 0.176; %kg: nanoplus without gumstix and camera
g = 9.81;
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];
 km = 1.5*1e-09; 
 kf = 6.11*1e-08;

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.086;

% Ixx = I(1,1);
% Iyy = I(2,2);
% Izz = I(3,3);

params.maxangle = 40*pi/180; %you can specify the maximum commanded angle here
params.maxF = 2.5*m*g;
params.minF = 0.05*m*g;
%% gains
params.kpphi = .475; 
params.kdphi = .115; 
params.kppsi = .5; 
params.kdpsi = .1; 
params.kptheta = .475; 
params.kdtheta = .115; 

params.kpx =15*2; 
params.kdx = 6*3; 
params.kpy =15*2; 
params.kdy =6*3; 
params.kpz =15*2;
params.kdz =4*2.5; 
%% lift
params.km = km;
params.kf = kf;  
params.gamma = km/kf; 
% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

end
