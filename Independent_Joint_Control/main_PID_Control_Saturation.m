% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate PID control in time domain with saturation 

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System parameters
J = 2;          % Motor inertia = actuator inertia + gear inertia (Ja + Jg)
b = 1;          % Effective Damping term
tau_l = 40.0;   % Load torque
r = 1;          % gear ratio
d = tau_l/r;    % disturbance input

% Simulation time
t0 = 0.0;   % Initial time
tf = 3.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% PID Control gain
Kp = 64;
Kd = 20;
Ki = 13;

% Initialization
ddth = 0;
dth = 0;
th = 0;
e = 0;
th_d = 10;  % Desired step input
for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d - th) - Kd*dth + Ki*e;  % PID controller
    ddth = (1/J)*(u - d - b*dth);    % Actuator dynamics

    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;
    
    % For I gain
    e = e + (th_d - th)*dt;

    th_d_out(i) = th_d;
    ddth_out(i) = ddth;
    dth_out(i) = dth;
    th_out(i) = th;
    u_out(i) = u;
end

set(gcf,'position',[0 10 1200 700])
figure(1)
subplot(2,2,1)
h1 = plot(time, th_out,'k','linewidth',2);
hold on
plot(time, th_d_out,'k:')
ylabel('Joint Angle [deg]')
ylim([0 th_d*1.2])
set(gca,'fontsize',13)
subplot(2,2,2)
plot(time, dth_out,'k','linewidth',2)
hold on
ylabel('Joint Velocity [deg/sec]')
set(gca,'fontsize',13)
subplot(2,2,3)
plot(time, ddth_out,'k','linewidth',2)
hold on
ylabel('Joint Acceleration [deg/sec^2]')
set(gca,'fontsize',13)
subplot(2,2,4)
plot(time, u_out,'k','linewidth',2)
hold on
ylabel('Input u(t)')
xlabel('Time [sec]')
set(gca,'fontsize',13)

% PID control with Saturation
% PID Control gain
Kp = 64;
Kd = 15;
Ki = 3.5;

% Initialization
ddth = 0;
dth = 0;
th = 0;
e = 0;
max = 50;
for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d - th) - Kd*dth + Ki*e;
    if u > max
        u = max;
    elseif u < -max
        u = -max;
    else
        u = u;
    end
        
    ddth = (u - d - b*dth);

    dth = dth + ddth*dt;
    th = th + dth*dt;
    
    % For I gain
    e = e + (th_d - th)*dt;

    ddth_s_out(i) = ddth;
    dth_s_out(i) = dth;
    th_s_out(i) = th;
    u_s_out(i) = u;
end

set(gcf,'position',[0 10 1200 700])
figure(1)
subplot(2,2,1)
h2 = plot(time, th_s_out,'r','linewidth',2);
hold on
h3 = plot(time, th_d_out,'k:');
ylabel('Joint Angle [deg]')
ylim([0 th_d*1.2])
legend([h1, h2, h3], 'Without Saturation', 'With Saturation', 'Desired Input')
set(gca,'fontsize',13)
subplot(2,2,2)
plot(time, dth_s_out,'r','linewidth',2)
hold on
ylabel('Joint Velocity [deg/sec]')
set(gca,'fontsize',13)
subplot(2,2,3)
plot(time, ddth_s_out,'r','linewidth',2)
hold on
ylabel('Joint Acceleration [deg/sec^2]')
set(gca,'fontsize',13)
subplot(2,2,4)
plot(time, u_s_out,'r','linewidth',2)
hold on
ylabel('Input u(t)')
xlabel('Time [sec]')
set(gca,'fontsize',13)
