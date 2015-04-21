close all;
clear;
clc;

initialize;

%% Simulate the system using ODE45
[Ts, XS] = ode45(@(t,x) noisy_model(t, x, A, B, sgm, bias), Ts, x0); % Solve ODE
XS = XS';



%% Simualate the sensors

for i=1:4 
XSMax(i)=max(XS(i,:))
end
ZS(1,:)=XS(1,:)+0.05*randn(size(Ts'))*XSMax(1);
ZS(2,:)=XS(2,:)+0.1*randn(size(Ts'))*XSMax(2);
ZS(3,:)=XS(3,:)+0.15*randn(size(Ts'))*XSMax(3);
ZS(4,:)=XS(4,:)+0.2*randn(size(Ts'))*XSMax(4);

%% Filter Part 1 
V = dt* eye(N); 
b_sgm = zeros(4,4,length(Ts));  
b_sgm(:,:,1) = P0; 
b_mu = zeros(4,length(Ts));  
b_mu(:,1) = x0; 
for i = 2:length(Ts) 
   %% Predict 
   pred_sgm = F*b_sgm(:,:,i-1)*F' +V*Q*V'; 
   pred_mu = F*b_mu(:,i-1)+ G*input_fun(Ts(i)); 
   %% Update 
   Kt = pred_sgm*H'* inv(H*pred_sgm*H' + R); 
   b_mu(:,i) = pred_mu  + Kt*(ZS(:,i) - H*pred_mu);
   b_sgm(:,:,i) = pred_sgm - Kt*H*pred_sgm; 
end
b2_sgm = zeros(4,4,length(Ts));  
b2_sgm(:,:,1) = P0; 
b2_mu = zeros(4,length(Ts));  
b2_mu(:,1) = x0; 
pred_sgm = [];
pred_mu = []; 
C = [1;0;0;0]'; % Only accounts for position sensor 
for i = 2:length(Ts) 
   %% Predict 
   pred_sgm = F*b2_sgm(:,:,i-1)*F' +V*Q*V'; 
   pred_mu = F*b2_mu(:,i-1)+ G*input_fun(Ts(i)); 
   %% Update 
   Kt =  pred_sgm*C'*inv(C*pred_sgm*C' + R(1,1)); 
   b2_mu(:,i) = pred_mu  + Kt*(ZS(1,i) - C*pred_mu);
   b2_sgm(:,:,i) = pred_sgm - Kt*C*pred_sgm; 
end
%% part c 
b3_sgm = zeros(4,4,length(Ts));  
b3_sgm(:,:,1) = P0; 
b3_mu = zeros(4,length(Ts));  
b3_mu(:,1) = x0; 
pred_sgm = [];
pred_mu = []; 
C = H(2:end,:); % Only accounts for position sensor 
for i = 2:length(Ts) 
   %% Predict 
   pred_sgm = F*b3_sgm(:,:,i-1)*F' +V*Q*V'; 
   pred_mu = F*b3_mu(:,i-1)+ G*input_fun(Ts(i)); 
   %% Update 
   Kt =  pred_sgm*C'*inv(C*pred_sgm*C' + R(2:4,2:4)); 
   b3_mu(:,i) = pred_mu  + Kt*(ZS(2:end,i) - C*pred_mu);
   b3_sgm(:,:,i) = pred_sgm - Kt*C*pred_sgm; 
end
% 
subplot(2,2,1);
hold on; grid on;
plot(Ts, XS(1, :),  'r');
plot(Ts,b2_mu(1,:),'g','MarkerSize',20);
h = legend('$x$',  'K filter only positon sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');
subplot(2,2,2); 
 hold on; grid on;
plot(Ts, XS(2, :),'b');
plot(Ts,b2_mu(2,:),'g','MarkerSize',20);
h = legend('$\dot{x}$',  'K filter only positon sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');
subplot(2,2,3);
 hold on; grid on;
plot(Ts, XS(3, :), 'k');
plot(Ts,b2_mu(3,:),'g','MarkerSize',20);
h = legend('$\ddot{x}$', 'K filter only positon sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');
subplot(2,2,4); 
 hold on; grid on;
plot(Ts, XS(4, :),'r');
plot(Ts,b2_mu(4,:),'g','MarkerSize',20);
h = legend('${x}^{(3)}$', 'K filter only positon sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');

figure;
subplot(2,2,1);
hold on; grid on;
plot(Ts, XS(1, :),  'r');
plot(Ts,b3_mu(1,:),'m','MarkerSize',20);
h = legend('$x$', 'K filter only vel,accel, and jerk sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');
subplot(2,2,2); 
 hold on; grid on;
plot(Ts, XS(2, :),  'b');
plot(Ts,b3_mu(2,:),'m','MarkerSize',20);
h = legend('$\dot{x}$',  'K filter only vel,accel, and jerk sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');
subplot(2,2,3);
 hold on; grid on;
plot(Ts, XS(3, :),'k');
plot(Ts,b3_mu(3,:),'m','MarkerSize',20);
h = legend('$\ddot{x}$', 'K filter only vel,accel, and jerk sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');
subplot(2,2,4); 
 hold on; grid on;
plot(Ts, XS(4, :), 'r');
plot(Ts,b3_mu(4,:),'m','MarkerSize',20);
h = legend('${x}^{(3)}$', 'K filter only vel,accel, and jerk sensing','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');
%% Figures
% figure; hold on; grid on;
% plot(Ts, XS(1, :), 'r');
% plot(Ts, XS(2, :), 'g');
% plot(Ts, XS(3, :), 'b');
% plot(Ts, XS(4, :), 'c');
% plot(Ts, input_fun(Ts), 'm');
% h = legend('$x$', '$x^{(i)}$', '$x^{(ii)}$', '$x^{(iii)}$', 'u', 'Location', 'NorthWest');
% set(h,'Interpreter','latex');
% set(h,'FontSize', 16);
% xlabel('Time (seconds)');
% title('Model Simulation');
figure
subplot(2,2,1);
hold on; grid on;
plot(Ts, XS(1, :),  'r');
plot(Ts,b_mu(1,:),'g','MarkerSize',20);
h = legend('$x$', 'K filter with all 4 sensors','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Position)');
subplot(2,2,2); 
 hold on; grid on;
plot(Ts, XS(2, :), 'b');
plot(Ts,b_mu(2,:),'g','MarkerSize',20);
h = legend('$\dot{x}$',  'K filter with all 4 sensors','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Velocity)');
subplot(2,2,3);
 hold on; grid on;
plot(Ts, XS(3, :), 'k');
plot(Ts,b_mu(3,:),'g','MarkerSize',20);
h = legend('$\ddot{x}$','K filter with all 4 sensors','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Acceleration)');
subplot(2,2,4); 
 hold on; grid on;
plot(Ts, XS(4, :),  'r');
plot(Ts,b_mu(4,:),'g','MarkerSize',20);
h = legend('${x}^{(3)}$','K filter with all 4 sensors','Location', 'NorthWest');
set(h,'Interpreter','latex');
set(h,'FontSize', 16);
xlabel('Time (seconds)');
title('Sensed Information (Jerk)');

