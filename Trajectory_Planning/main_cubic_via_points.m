% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Sampling rate info
Hz = 1000;
dt = 1/ Hz;

q = [10 40 30 90 40 60 10 70];
v = [0 0 0 0 0 0 0 0];
t = [0 2 4 6 7 8 9 10];

[time, q_out, dq_out, ddq_out] = cubic_via_points(t, q, v, Hz);


figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time, q_out,'k','linewidth',2)
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dq_out,'k','linewidth',2)
ylabel('Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddq_out,'k','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)


