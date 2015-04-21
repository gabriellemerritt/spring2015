%% PREPARE DIRECTORY
% clear all
clear init_script
close all
clc
curr_dir = pwd;
addpath(genpath([curr_dir,'/data']))

%% INITIALIZE THINGS
init_script
load studentdata1
% load studentdata4
% load studentdata9


% data, time, vicon
% [pos, eul] = estimate_pose(sensor, varargin)
dataL = length(data);
ts = zeros(dataL, 1);
vel = zeros(dataL, 3);
omg = zeros(dataL, 3);
tic
vic.t = []; 
vic.vel = []; 
for i = 1:dataL
%     disp([num2str(i),'/',num2str(dataL)])
   
    sensor = data(i);
    ts(i) = sensor.t;
%     [~, t_idx] = max(sensor.t < time); 
    t_idx = find(time == ts(i)); 
    vic.t = time(t_idx); 
    vic.vel = vicon(7:12,t_idx); 
    [X, Z] = ekf1_handle(sensor,vic);
    if ~isempty(X)
        pos(i, :) = X';
%         omg(i, :) = Z';
    end
    vic.t = []; 
    vic.vel = []; 
end
t = toc;
disp(['Time to completion: ', num2str(t)])
time = time';
vicon = vicon';

% plot vicon data [x y z roll pitch yaw vx vy vz wx wy wz]'
% plot estimates
figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:,1), 'r', ts, pos(:, 1), 'b')
xlabel('Time [s]')
ylabel('X [m]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 2), 'r', ts, pos(:, 2), 'b')
xlabel('Time [s]')
ylabel('Y [m]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 3), 'r', ts, pos(:, 3), 'b')
xlabel('Time [s]')
ylabel('Z  [m]')
hold off

figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 4), 'r', ts, pos(:, 4), 'b')
xlabel('Time [s]')
ylabel('R [rad]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:,5), 'r', ts, pos(:,5), 'b')
xlabel('Time [s]')
ylabel('P [rad]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 6), 'r', ts, pos(:, 6), 'b')
xlabel('Time [s]')
ylabel('Y [rad]')
hold off
