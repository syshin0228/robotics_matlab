% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate feedforward controller 
% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System parameters
J = 1;          % Motor inertia = actuator inertia + gear inertia (Ja + Jg)
b = 1;          % Effective Damping constant
tau_l = 0.0;    % Load torque
r = 1;          % gear ratio
d = tau_l/r;    % disturbance input

% Parameters for PD control - 2nd order system 
w = 12;                 % Natural frequency
zeta = 1;               % Damping ratio (zeta = 1 is critical damping)
Kp = w^2*J;             % P gain
Kd = 2*zeta*w*J - b;    % D gain

% Simulation time
t0 = 0.0;   % Initial time
tf = 2.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Desired trajectory
A = 1.0;
w_d = 2*pi;
th_d = A*sin(w_d*time);   % Sinusoidal desired input trajectory
dth_d = A*w_d*cos(w_d*time);   % Desired Velocity 
ddth_d = -A*w_d^2*sin(w_d*time);   % Desired Acceleration 

% With out feedforward control
ddth = 0;
dth = 0;
th = 0;
for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d(i) - th) - Kd*dth;    % PD controller
%     u = Kp*(th_d(i) - th) + Kd*(dth_d(i) - dth);    % PD controller with desired velocity
    ddth = (1/J)*(u - d - b*dth);   % Actuator dynamics

    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;

    th_d_out(i) = th_d(i);
    ddth_out(i) = ddth;
    dth_out(i) = dth;
    th_out(i) = th;
    u_out(i) = u;
end

% With feedforward control
ddth = ddth_d(1);
dth = dth_d(1);
th = th_d(1);
for i = 1:1:length(time)
    t = time(i);
    u = J*ddth_d(i) + b*dth_d(i) + Kp*(th_d(i) - th) + Kd*(dth_d(i) - dth);    % PD controller + Feedforward
    ddth = (1/J)*(u - d - b*dth);   % Actuator dynamics

    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;

    ddth_f_out(i) = ddth;
    dth_f_out(i) = dth;
    th_f_out(i) = th;
    u_f_out(i) = u;
end

set(gcf,'position',[0 10 1200 700])
figure(1)
subplot(2,2,1)
h1 = plot(time, th_out,'b','linewidth',2);
hold on
h2 = plot(time, th_f_out,'r:','linewidth',2);
h3 = plot(time, th_d,'k');
ylabel('Joint Angle [deg]')
xlabel('Time [sec]')
legend([h1, h2, h3], 'PD', 'PD+Feedforward', 'Desired traj')
set(gca,'fontsize',13)

subplot(2,2,2)
plot(time, dth_out,'b','linewidth',2);
hold on
plot(time, dth_f_out,'r:','linewidth',2);
plot(time, dth_d,'k');
ylabel('Angular Velocity [deg/s]')
xlabel('Time [sec]')
set(gca,'fontsize',13)

subplot(2,2,3)
plot(time, ddth_out,'b','linewidth',2);
hold on
plot(time, ddth_f_out,'r:','linewidth',2);
plot(time, ddth_d,'k');
ylabel('Angular Acceleration [deg/s^2]')
xlabel('Time [sec]')
set(gca,'fontsize',13)

subplot(2,2,4)
h3 = plot(time, u_out,'b','linewidth',2);
hold on
h4 = plot(time, u_f_out,'r:','linewidth',2);
ylabel('Input u(t)')
xlabel('Time [sec]')
set(gca,'fontsize',13)
legend([h3, h4], 'PD', 'PD+Feedforward')
