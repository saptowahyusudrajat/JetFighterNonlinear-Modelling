% Resort the terms:
% 8*y[n] - 6*y[n-1] + 2*y[n-2] = 1
% y[n] = (1 + 6*y[n-1] - 2*y[n-2]) / 8
% or in Matlab:
% 
% y(n) = (1 + 6*y(n-1) - 2*y(n-2)) / 8;
% Now the indices cannot start at -1, because in Matlab indices are greater than 0. This can be done by a simple translation:


y = zeros(1, 100);  % Pre-allocate
y(1:2) = [2, 0];
for k = 3:100
  y(k) = (1 + 6*y(k-1) - 2*y(k-2)) / 8;
end