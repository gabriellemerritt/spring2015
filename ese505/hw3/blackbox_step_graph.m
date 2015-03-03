a  = 21; 
num = [ 2*a^3]; 
den = [ 1 3*a 3*a^2 a^3]; 
Gp = tf(num,den); 
[y,t] = step(Gp); 
plot(t,y,'-r','LineWidth',2); 
grid on; set(gcf,'Color','w'); 
title('Black Box Step Response'); 
xlabel('Time [sec]'); 
ylabel('Response [volts]'); 
figure 
sim('blackbox'); 
plot(ScopeData(:,1),ScopeData(:,2),'-r','LineWidth',2);
grid on; set(gcf,'Color','w');
title('Black Box Simulink Step Response'); 
xlabel('Time [sec]'); 
ylabel('Response [volts]'); 
per = ScopeData2(loc,1); 
T = ScopeData2(loc(2) - loc(1)); 
n = 4; 
[peaks, loc] = findpeaks(ScopeData2(:,2));
[~,locnT] = max(ScopeData2(:,1) >(ScopeData2(loc(2),2)+ n*T)); 
% sigma = (1/n)*log(ScopeData2(loc(2),2)/(ScopeData2(locnT-1,2))); 
% damp_ratio = 1/ sqrt(1 + (2*pi/sigma)^2); 
% wd = 2*pi/T ; 
% wn =  wd /sqrt(1- damp_ratio^2); 

