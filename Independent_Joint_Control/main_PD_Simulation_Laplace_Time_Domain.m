% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate PD control in Laplace domain and in time domain
% Validate both domains show same performance

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System parameters
J = 1;          % Motor inertia = actuator inertia + gear inertia (Ja + Jg)
b = 1;          % Effective Damping term
tau_l = 10.0;   % Load torque
r = 1;          % gear ratio
d = tau_l/r;    % disturbance input

% parameters for PD control - 2nd order system 
w = 12;                 % natural frequency
zeta = 1;               % Damping ratio (zeta = 1 is critical damping)
Kp = w^2*J;             % P gain
Kd = 2*zeta*w*J - b;    % D gain

% System transfer function (Laplace domain)
% System parameters 
den = [J b+Kd Kp];      % Denominator 
num1 = [Kp];            % Numerator of controller term 
num2 = [d];             % Numerator of disturbance term
sys_c = tf(num1,den);   % transfer function of controller
sys_d = tf(num2,den);   % transfer function of disturbance
% Transfer function of full system (Laplace domain)
sys = sys_c - sys_d;

% Simulation time
t0 = 0.0;   % Initial time
tf = 2.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Step input
[y,t] = step(sys,time);

figure(1)
h1 = plot(t, y,'k','linewidth',1);
hold on
title('Step Respose of Actuator with PD controller')
ylabel('Joint Angle [deg]')
xlabel('Time [sec]')
set(gca,'fontsize',13)

% Simulation in time domain
ddth = 0;
dth = 0;
th = 0;
th_d = 1;   % Step input

for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d - th) - Kd*dth;    % PD controller
    ddth = (1/J)*(u - d - b*dth);   % Actuator dynamics

    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;

    th_d_out(i) = th_d;
    ddth_out(i) = ddth;
    dth_out(i) = dth;
    th_out(i) = th;
    u_out(i) = u;
end

figure(1)
h2 = plot(time, th_out,'r:','linewidth',2);
h3 = plot(time, th_d_out,'k:');
ylabel('Joint Angle [deg]')
xlabel('Time [sec]')
title('Step Respose of Actuator with PD controller')
legend([h1, h2, h3],'Laplace domain', 'Time domain', 'Desired Input')
set(gca,'fontsize',13)
ylim([0 1.2])
