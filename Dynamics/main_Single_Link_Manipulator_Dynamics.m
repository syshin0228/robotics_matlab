% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Dynamics of a Single-Link Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

m = 10;             % Mass
Jm = 1;             % Inertia of motor
Jl = 1;             % Inertia of load
Bm = 1;             % Motor Damping
Bl = 1;             % Load Damping
r = 1;              % Gear ratio
J = r^2*Jm + Jl;    % Inertia of motor and link using th_m = r*th_l
g = 9.81;           % Gravity acceleration
B = r*Bm + Bl;      % Motor + Load damping 
Lc = 0.5;           % Length to CoM from origin
L = 1;              % Link length

% Simulation time
t0 = 0.0;                   % Initial time
tf = 10.0;                  % Final time
time = [t0:1:tf*Hz]*dt;     % Time

% Input torque parameters
A = 0;
w = 2*pi;
offset = 0.0;

% Initialization
th = pi/2;
dth = 0;
ddth = 0;
for i = 1:1:length(time)
    t = time(i);
    u = A*sin(w*t) + offset;                       % Control input torque
    ddth = (1/J)*(u -B*dth -m*g*Lc*sin(th));       % Dynamic equations of motion
    
    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;

    ddth_out(i) = ddth;
    dth_out(i) = dth;
    th_out(i) = th;
    u_out(i) = u;
    
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time, th_out*rad2deg,'k','linewidth',2)
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_out*rad2deg,'k','linewidth',2)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_out*rad2deg,'k','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
plot(time, u_out,'k','linewidth',2)
ylabel('Input Torque [Nm]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Visualize simulation
step = 200;
origin = [0 0]';
for i = 1:step:length(time)
    
    % End position of link
    Px = L*sin(th_out(i));
    Py = -L*cos(th_out(i));

    % Position of link CoM 
    Px_com = Lc*sin(th_out(i));
    Py_com = -Lc*cos(th_out(i));
    
    figure(100)    
    plot(Px_com,Py_com,'or','linewidth',3)
    hold on
    line([origin(1) Px],[origin(2) Py], 'color', 'k', 'linewidth', 2)
    plot(origin(1),origin(2),'ok','linewidth',5)
    axis equal
    axis([-1 1 -1 1])
    xlabel('x- axis')
    ylabel('y- axis')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off
end

