function[gain ,sigma, wd, wn, damp_ratio, poles] = gain_table(ScopeData2,n, K)

[peaks, loc] = findpeaks(ScopeData2(:,2));
T = ScopeData2(loc(2) - loc(1)); 
[~,locnT] = max(ScopeData2(:,1) >(ScopeData2(loc(1),2)+ n*T)); 
d = (1/n)*log(ScopeData2(loc(1),2)/(ScopeData2(locnT-1,2))); 
% sigma = log(ScopeData2(loc(1),2)/ScopeData2(loc(2),2));
damp_ratio = 1/ sqrt(1 + (2*pi/d)^2); 
wd = 2*pi/T ; 
wn =  wd /sqrt(1- damp_ratio^2); 
gain = K; 
poles =[(-damp_ratio+ sqrt(damp_ratio^2 -1))*wn (-damp_ratio- sqrt(damp_ratio^2 -1))*wn];
sigma = real(poles);
% poles =[(-KSWW(:,5)+sqrt(KSWW(:,5).^2 -1)).*KSWW(:,4)  (-KSWW(:,5)- sqrt(KSWW(:,5).^2 -1)).*KSWW(:,4)];