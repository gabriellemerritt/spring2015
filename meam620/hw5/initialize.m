%% Simulation related paramters
dt  = 0.001;             % Time step
                        % Time interval at each of which the Kalman filter
                        % is updated. 
                        % Note : ode45 populates the output with solutions 
                        % at the given time points.
Tf  = 1;                % Simulation duration
Ts  = 0:dt:Tf;          % 

%% Model definition
N    = 4;                  % Dimension of the system
bias = 0 * randn(1);       % Bias term for the process noise (n)
sgm  = 10;  % Variance of the noise (n)
                           % Note : Increase in the variance (sgm) will 
                           % cause input_fun() to generate peaky inputs. 
                           % Higher this value is, slower will be ode45 
                           % solver since smaller time steps will be 
                           % required a stiffer system. 

x0   = zeros(N, 1);        % Initial state
A    = [0 1 0 0;           % State transition matrix
        0 0 1 0;
        0 0 0 1;
        0 0 0 0];
B    = [0; 0; 0; 1];       % Input transformation matrix
                           % xdot  = A * x + B * (u + n)  
                           % where n is the noise (scalar).
%% Kalman Filter definition
H    = eye(N);              % Mapping from state space to measurement space.
                            % We assume we can observe all states. H is C 
F    = eye(N) + A * dt; 
G    = B * dt;              % xpred = F * x + G * u
                            
Q    = 10*eye(N) * sgm^2;   % Process noise covariance matrix.
                            % This is added up to the predicted covariance
R    = 10*eye(N) * sgm^2;   % Measurement noise. 
                            % This is added up to the measurement
                            % uncertainty before it is integrated to the
                            % predicted state.

P0   = eye(N);              % Initial covariance matrix for the Kalman 
                            % filter