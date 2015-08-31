figure;
w = logspace(-1,10,1000);
a = 21; 
num = 2*a^3; 
den =[ 1 3*a 3*a^2 a^3]; 
G1 = tf(num,den); 
margin(G1); 
print('part4_analog','-dpng');

figure; 
tau = .0033; 
T = .01; 
B = conv([-.5*T 1 ], num); 
A = conv(conv([tau 1], [tau 1 ]), conv([.5*T 1 ], den));  
G = tf(B,A); 
margin(G); 
print('part4_digital','-dpng');