function [y, dy, ddy] = quintic_traj(t0, tf, y0, yf, dy0, dyf, ddy0, ddyf, Hz)

% Quintic Polynomial Coefficient Calculation
M = [1 t0 t0^2 t0^3 t0^4 t0^5; 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 0 2 6*t0 12*t0^2 20*t0^3; 1 tf tf^2 tf^3 tf^4 tf^5; 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 0 0 2 6*tf 12*tf^2 20*tf^3];
Val = [y0 dy0 ddy0 yf dyf ddyf]';
a = inv(M)*Val;

% Time
dt = 1/Hz;
t = [t0*Hz:1:tf*Hz]*dt;

% Quintic Polynomial Trajectory
y = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;      % Position
dy = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;         % Velocity
ddy = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3 ;                    % Acceleration
