% figure;
% a = 0;
% den = [1 6 5 0 0]; 
% num = [1 2.*a a^2];
% G = tf(num,den); 
% margin(G); 
% % bode(G);
% print('3e_a0','-dpng'); 
% 
% figure;
% a = 2; 
% den1 = [1 6 5 0 0]; 
% num1 = [ 1 2.*a a^2]; 
% G1 = tf(num1,den1); 
% margin(G1);
% % bode(G1); 
% print('3e_a2','-dpng'); 

% figure;
% a = .5; 
% den2 = [ 1 6 5 0 0]; 
% num2 = [1 2.*a a.^2];
% G2 = tf(num2,den2); 
% margin(G2); 
%  bode(G2);
% print('3e_ahalf','-dpng'); 
% % 
figure;
a = .9; 
den3 = [ 1 6 5 0 0 ]; 
num3 = [ 1 2.*a a.^2]; 
G3 = tf(num3,den3); 
%bode(G3);
margin(G3);
print('3e_apoint9','-dpng'); 

