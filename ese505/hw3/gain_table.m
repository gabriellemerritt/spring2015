function[gain ,sigma, wd, wn, damp_ratio, poles] = gain_table(ScopeData2,n, K)

[peaks, loc] = findpeaks(ScopeData2(:,2));
T = ScopeData2(loc(2),1) - ScopeData2(loc(1),1); 
pk2pk = ScopeData2(loc(1),2) - min(ScopeData2(loc(1):end,2));
pk2pk = pk2pk/2;  

npk2pk = (ScopeData2(loc(n),2) - min(ScopeData2(loc(n):end,2)))/2; 

d = (1/(n-1))*log(pk2pk/npk2pk); 
% sigma = log(ScopeData2(loc(1),2)/ScopeData2(loc(2),2));
damp_ratio = d/ sqrt(4*pi^2 +d^2); 
wd = 2*pi/T ; 
wn =  wd /sqrt(1- damp_ratio^2); 
gain = K; 
poles =[(-damp_ratio+ sqrt(damp_ratio^2 -1))*wn (-damp_ratio- sqrt(damp_ratio^2 -1))*wn];
sigma = real(poles);
% poles =[(-KSWW(:,5)+sqrt(KSWW(:,5).^2 -1)).*KSWW(:,4)  (-KSWW(:,5)- sqrt(KSWW(:,5).^2 -1)).*KSWW(:,4)];