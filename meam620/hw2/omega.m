R =[ -0.3038  -0.6313 -0.7135;...
   -0.9332   0.3481    0.0893;...
    0.1920   0.6930  -0.6949];
tau = sum(diag(R)); 
angle = acos((tau -1)/2); 
omega = (1/(2*sin(angle)))*(R -R'); 