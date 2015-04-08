% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function
world_points; 
K = [314.1779 0         199.4848; ...
0         314.2218  113.7838; ...
0         0         1]; 
loop_num= 0; 
estimate_vel_handle = @(sensor) estimate_vel(sensor, Wp, K);
