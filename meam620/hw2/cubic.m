x0_y0 = [ -1 0]; 
x1_y1 = [0 2]; 
x2_y2 = [ 1 0]; 
x0_y0dot = [ -1 -5]; 
x2_y2dot = [ 0 0]; 
t = [ 0 5 6]; 
t01 = [0:.05:5]; 
t12 = [5:.05:6]; 
A = [ 1 t(1) t(1)^2 t(1)^3 0 0 0 0; ...
      0 1 2*t(1) 3*t(1)^2 0 0 0 0; ...
      1 t(2) t(2)^2 t(2)^3 0 0 0 0; ...
      0 0 0 0 1 t(2) t(2)^2 t(2)^3; ... 
      0 1 2*t(2) 3*t(2)^2 0 -1 -2*t(2) -3*t(2)^2; ...
      0 0 2 6*t(2) 0 0 -2 -6*t(2); ...
      0 0 0 0 1 t(3) t(3)^2 t(3)^3; ... 
      0 0 0 0 0 1 2*t(3) 3*t(3)^2; ];  
b = [x0_y0; x0_y0dot; x1_y1; x1_y1; 0 0; 0 0; x2_y2; x2_y2dot];
coeff = A\b; 

P0x = coeff(1,1) + coeff(2,1)*t01 + coeff(3,1)*t01.^2 + coeff(4,1)*t01.^3;
P0y = coeff(1,2) + coeff(2,2)*t01 + coeff(3,2)*t01.^2 + coeff(4,2)*t01.^3;
P1x = coeff(5,1) + coeff(6,1)*t12 + coeff(7,1)*t12.^2 + coeff(8,1)*t12.^3; 
P1y = coeff(5,2) + coeff(6,2)*t12 + coeff(7,2)*t12.^2 + coeff(8,2)*t12.^3;
hold on
plot([P0x P1x],[P0y P1y],'-r'); 
scatter([x0_y0(1) x1_y1(1) x2_y2(1)], [x0_y0(2) x1_y1(2) x2_y2(2)],'b');
title('Position'); 
xlabel('X position m'); 
ylabel('Y position m'); 

