%% PREPARE DIRECTORY
% clear all
clear init_script
close all
clc
curr_dir = pwd;
addpath(genpath([curr_dir,'/data']))

%% INITIALIZE THINGS
init_script
% load studentdata1
% load studentdata4
load studentdata9


% data, time, vicon
% [pos, eul] = estimate_pose(sensor, varargin)
dataL = length(data);
ts = zeros(dataL, 1);
vel = zeros(dataL, 3);
omg = zeros(dataL, 3);
tic
for i = 1:dataL
%     disp([num2str(i),'/',num2str(dataL)])
    sensor = data(i);
    ts(i) = sensor.t;
    [v, w] = estimate_vel_handle(sensor);
    if ~isempty(v)
        vel(i, :) = v';
        omg(i, :) = w';
    end
    i
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
plot(time, vicon(:, 7), 'r', ts, vel(:, 1), 'b')
xlabel('Time [s]')
ylabel('X Velocity [m/s]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 8), 'r', ts, vel(:, 2), 'b')
xlabel('Time [s]')
ylabel('Y Velocity [m/s]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 9), 'r', ts, vel(:, 3), 'b')
xlabel('Time [s]')
ylabel('Z Velocity [m/s]')
hold off

figure
subplot(3, 1, 1)
hold on
plot(time, vicon(:, 10), 'r', ts, omg(:, 1), 'b')
xlabel('Time [s]')
ylabel('Omega X [rad/s]')
hold off
subplot(3, 1, 2)
hold on
plot(time, vicon(:, 11), 'r', ts, omg(:, 2), 'b')
xlabel('Time [s]')
ylabel('Omega Y [rad/s]')
hold off
subplot(3, 1, 3)
hold on
plot(time, vicon(:, 12), 'r', ts, omg(:, 3), 'b')
xlabel('Time [s]')
ylabel('Omega Z [rad/s]')
hold off
