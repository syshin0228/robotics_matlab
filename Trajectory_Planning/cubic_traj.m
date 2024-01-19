function [y, dy, ddy] = cubic_traj(t0, tf, y0, yf, dy0, dyf, Hz)

% Cubic Polynomial Coefficient Calculation
M = [1 t0 t0^2 t0^3; 0 1 2*t0 3*t0^2; 1 tf tf^2 tf^3; 0 1 2*tf 3*tf^2];
Val = [y0 dy0 yf dyf]';
a = inv(M)*Val;

% Time
dt = 1/Hz;
t = [t0*Hz:1:tf*Hz]*dt;

% Cubic Polynomial Trajectory
y = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;  % Position
dy = a(2) + 2*a(3)*t + 3*a(4)*t.^2;         % Velocity
ddy = 2*a(3) + 6*a(4)*t;                    % Acceleration

