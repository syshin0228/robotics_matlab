% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Sampling rate info
Hz = 1000;
dt = 1/ Hz;

% Given constraint variables
t0 = 0.0;
tf = 2.0;
q0 = 0.0;
qf = 40.0;
v0 = 0.0;
vf = 0.0;
alpha0 = 0.0;
alphaf = 0.0;

% Quintic Polynomial Coefficient Calculation
M = [1 t0 t0^2 t0^3 t0^4 t0^5; 0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 0 2 6*t0 12*t0^2 20*t0^3; 1 tf tf^2 tf^3 tf^4 tf^5; 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 0 0 2 6*tf 12*tf^2 20*tf^3];
Val = [q0 v0 alpha0 qf vf alphaf]';
a = inv(M)*Val;

% Time
t = [0:1:tf*Hz]*dt;

% Quintic Polynomial Trajectory
q = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;      % Position
dq = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;         % Velocity
ddq = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3 ;                    % Acceleration

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(t,q,'k','linewidth',1)
hold on
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(t,dq,'k','linewidth',1)
hold on
ylabel('Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(t,ddq,'k','linewidth',1)
hold on
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Using function
[q_out, dq_out, ddq_out] = quintic_traj(t0, tf, q0, qf, v0, vf, alpha0, alphaf, Hz);

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(t,q_out,'r:','linewidth',2)
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(t,dq_out,'r:','linewidth',2)
ylabel('Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(t,ddq_out,'r:','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

