function [t_out, y_out, dy_out, ddy_out] = cubic_via_points(t_via, y_via, v_via, Hz)

dt = 1/ Hz;

NoP = length(y_via);
t_out = [];
y_out = [];
dy_out = [];
ddy_out = [];

for i = 1:1:NoP-1
    % Get Cubic Polynomial Coefficients
    a = cubic_coeff(t_via(i), t_via(i+1), y_via(i), y_via(i+1), v_via(i), v_via(i+1));
    
    % Define Time
    t = [t_via(i)*Hz:1:t_via(i+1)*Hz-1]*dt;
    
    % Cubic Polynomial Trajectory
    y = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;  % Position
    dy = a(2) + 2*a(3)*t + 3*a(4)*t.^2;         % Velocity
    ddy = 2*a(3) + 6*a(4)*t;                    % Acceleration

    % Connect via points
    t_out = [t_out t];
    y_out = [y_out y];
    dy_out = [dy_out dy];
    ddy_out = [ddy_out ddy];
    
    clear t y dy ddy
    
end
