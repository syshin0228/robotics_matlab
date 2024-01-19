% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate Dynamics of 1DoF linear system

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System Parameters
m = 1;      % mass
g = 9.81;   % gravity acceleration

% Simulation time
t0 = 0.0;   % Initial time
tf = 10.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Input force parameters
A = 0;
w = 2*pi;
offset = 0;

% Forward Dynamics of 1DoF System
% Initialization
x = 0;
y = 0;
dy = 0;
ddy = 0;
for i = 1:1:length(time)
    t = time(i);
    u = A*sin(w*t)+offset;    % Input force
    ddy = (1/m)*u - g;        % mass dynamics

    % Numerical integration
    dy = dy + ddy*dt;
    y = y + dy*dt;

    ddy_out(i) = ddy;
    dy_out(i) = dy;
    y_out(i) = y;
    u_out(i) = u;
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time, y_out,'k','linewidth',2)
ylabel('Displacement [m]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dy_out,'k','linewidth',2)
ylabel('Velocity [m/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddy_out,'k','linewidth',2)
ylabel('Acceleration [m/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
plot(time, u_out,'k','linewidth',2)
ylabel('Input Force [N]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)


% Visualize Simulation
step = 1000
for i = 1:step:length(time)
    figure(100)    
    plot(x,y_out(i),'or','linewidth',5)
    hold on
    axis([-1 1 min(y_out) max(y_out)])
    xlabel('x- axis')
    ylabel('y- axis')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off
end

