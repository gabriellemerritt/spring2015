% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function

estimate_vel_handle = @(sensor) estimate_vel(sensor);
