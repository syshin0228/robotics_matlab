% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate Dynamics of a Single-Link Manipulator 
% Get desired torque using given desired motion with Inverse Dynamics 
% then simulate Forward dynamics with desired torque input

% Angle coversion
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
t0 = 0.0;               % Initial time
tf = 3;                 % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Paramters for desired motion
A1 = pi/2;
w1 = 2*pi;
offset1 = pi/2;

% Desired joint trajectory 
th_d = A1*sin(w1*time)+offset1;
dth_d = A1*w1*cos(w1*time);
ddth_d = -A1*w1^2*sin(w1*time);

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h1 = plot(time, th_d*rad2deg,'k','linewidth',1);
hold on
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_d*rad2deg,'k','linewidth',1)
hold on
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_d*rad2deg,'k','linewidth',1)
hold on
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Inverse Dynamics 
for i = 1:1:length(time)
    t = time(i);
      
    q = th_d(i);
    dq = dth_d(i);
    ddq = ddth_d(i);
    
    u = B*dq + J*ddq + Lc*g*m*sin(q);   % Inverse Dynamics
    u_out(i) = u;                       % Desired torque
    
end

figure(2)
plot(time, u_out,'k','linewidth',1)
hold on
ylabel('Input Torque [Nm]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward Dynamics Simulation
% Initialization
th = th_d(1);
dth = dth_d(1);
ddth = ddth_d(1);
for i = 1:1:length(time)
    t = time(i);
    
    u = u_out(i);                               % Desired input torque
    ddth = (1/J)*(u -B*dth -m*g*Lc*sin(th));    % Dynamic equations of motion
    
    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;

    ddth_out(i) = ddth;
    dth_out(i) = dth;
    th_out(i) = th;
    
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h2 = plot(time, th_out*rad2deg,'r:','linewidth',3);
ylabel('Angle [Deg]')
legend([h1,h2],'Desired traj','Output traj from FD')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_out*rad2deg,'r:','linewidth',3)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_out*rad2deg,'r:','linewidth',3)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Visualize Simulation
step = 100;
origin = [0 0]';
for i = 1:step:length(time)
    
    Px = L*sin(th_out(i));
    Py = -L*cos(th_out(i));
    Px_com = Lc*sin(th_out(i));
    Py_com = -Lc*cos(th_out(i));
    
    figure(100)    
    plot(Px_com,Py_com,'ok','linewidth',3)
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

