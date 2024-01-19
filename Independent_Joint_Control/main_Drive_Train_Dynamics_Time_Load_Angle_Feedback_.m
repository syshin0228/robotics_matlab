% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% PD controller with load angle feedback in time domain
simulation = 1;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% Simulation time
t0 = 0.0;   % Initial time
tf = 10.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% System parameters
Jl = 1;     % Load inertia 
Bl = 1;     % Load damping constant
Jm = 10;    % Motor inertia 
Bm = 10;    % Motor damping constant
k = 20;     % Torsional stiffness b/w load and motor

% Control gain
Kp = 20;    % P gain
Kd = 5;     % D gain

% Initialization
ddth_l = 0;
dth_l = 0;
th_l = 0;
ddth_m = 0;
dth_m = 0;
th_m = 0;
th_d = 45;  % Desired step input

% Simulation in time domain
for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d - th_l) - Kd*dth_l;                        % PD controller with load angle feedback
    ddth_m = (1/Jm) * (u - Bm*dth_m + k*(th_l - th_m));     % Motor dynamics
    ddth_l = (1/Jl) * (- Bl*dth_l + k*(th_m - th_l));       % Load dynamics
    
    % Numerical integration
    dth_m = dth_m + ddth_m*dt;
    th_m = th_m + dth_m*dt;
    dth_l = dth_l + ddth_l*dt;
    th_l = th_l + dth_l*dt;
    
    th_d_out(i) = th_d;
    ddth_m_out(i) = ddth_m;
    dth_m_out(i) = dth_m;
    th_m_out(i) = th_m;
    u_m_out(i) = u;
    
    ddth_l_out(i) = ddth_l;
    dth_l_out(i) = dth_l;
    th_l_out(i) = th_l;
    
end

figure(1)
h1 = plot(time, th_d_out,'k:');
hold on
h2 = plot(time, th_m_out,'r','linewidth',2);
h3 = plot(time, th_l_out,'k','linewidth',2);
ylabel('Joint Angle [deg]')
xlabel('Time [Sec]')
legend([h1, h2, h3], 'Desired Motor Angle', 'Motor Angle', 'Load Angle')
set(gca,'fontsize',13)

% Simulation visualization
step = 500;  % Simulation visualization speed
if simulation == 1
    % Draw Motor and Load as inner and outer circles
    x = [0:1:360]*pi/180;
    circle_xm = sin(x);
    circle_ym = cos(x);
    circle_xL = 2*sin(x);
    circle_yL = 2*cos(x);
    center = [0 0]';
    
    for i = 1:step:length(time)

        Px_m = 1*cosd(th_m_out(i));
        Py_m = 1*sind(th_m_out(i));

        Px_L = 2*cosd(th_l_out(i));
        Py_L = 2*sind(th_l_out(i));

        Px_d = 2*cosd(th_d);
        Py_d = 2*sind(th_d);

        figure(2)
        plot(circle_xm,circle_ym,'r','linewidth',2)
        hold on
        plot(circle_xL,circle_yL,'k','linewidth',2)
        line([center(1) Px_m],[center(2) Py_m], 'color', 'r', 'linewidth', 2)
        line([center(1) Px_L],[center(2) Py_L], 'color', 'k', 'linewidth', 2)
        line([center(1) Px_d],[center(2) Py_d], 'color', 'b', 'linewidth', 1, 'linestyle',':')
        axis equal
        xlabel('x- axis')
        ylabel('y- axis')
        set(gca,'fontsize',13)
        pause(0.001)
        hold off  
    end
end


