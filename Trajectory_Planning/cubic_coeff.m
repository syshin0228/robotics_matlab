function a = cubic_coeff(t0, tf, x0, xf, dx0, dxf)

% Cubic Polynomial Coefficient Calculation
M = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
Val = [x0 dx0 xf dxf]';
a = inv(M)*Val;
