function [y_out, dy_out, ddy_out] = min_time_traj(t0, tf, x0, xf, Hz)

% Minimum time trajectory parameters
ts = tf/2;
Vs = (xf - x0) / ts;
alpha = (xf - x0)/ts^2;

tb = ts;
V = Vs;

% Time
dt = 1/Hz;
time = [t0*Hz:1:tf*Hz]*dt;

% Linear Segments with Parabolic Blends (LSPB) calculation
for i = 1:1:length(time)
    t = time(i);
    
    if (t >= 0) && (t <= tb)
        y = x0 + alpha*t^2/2;
        dy = alpha*t;
        ddy = alpha;
    elseif (t > tb) && (t <= tf - tb)
        y = (xf + x0 - V*tf)/2 + V*t;
        dy = V;
        ddy = 0.0;
    elseif (t > tf - tb) && (t <= tf)
        y = xf - alpha*tf^2/2 + alpha*tf*t - alpha*t^2/2;
        dy = alpha*tf - alpha*t;
        ddy = -alpha;
    end
    
    y_out(i) = y;       % Position
    dy_out(i) = dy;     % Velocity
    ddy_out(i) = ddy;   % Acceleration
end