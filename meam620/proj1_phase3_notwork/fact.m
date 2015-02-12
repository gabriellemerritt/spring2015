function y = fact(n, m)
% using 'prod' and appropriate vectors
y = prod([m+1 : n]) / prod([1 : n-m]); 

end