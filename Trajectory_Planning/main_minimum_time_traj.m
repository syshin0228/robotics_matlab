% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Sampling rate info
Hz = 1000;
dt = 1/ Hz;

% Given constraint variables
t0 = 0;
tf = 2.0;
q0 = 0.0;
qf = 40.0;

% Minimum time trajectory parameters
ts = tf/2;
Vs = (qf - q0) / ts;
alpha = (qf - q0)/ts^2;

tb = ts;
V = Vs;

% Time
time = [0:1:tf*Hz]*dt;

% Minimum time trajectory calculation
for i = 1:1:length(time)
    t = time(i);
    
    if (t >= 0) && (t <= tb)
        q = q0 + alpha*t^2/2;
        dq = alpha*t;
        ddq = alpha;
    elseif (t > tb) && (t <= tf - tb)
        q = (qf + q0 - V*tf)/2 + V*t;
        dq = V;
        ddq = 0.0;
    elseif (t > tf - tb) && (t <= tf)
        q = qf - alpha*tf^2/2 + alpha*tf*t - alpha*t^2/2;
        dq = alpha*tf - alpha*t;
        ddq = -alpha;
    end
    
    q_out(i) = q;       % Position
    dq_out(i) = dq;     % Velocity
    ddq_out(i) = ddq;   % Acceleration
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time,q_out,'k','linewidth',1)
hold on
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time,dq_out,'k','linewidth',1)
hold on
ylabel('Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time,ddq_out,'k','linewidth',1)
hold on
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Using function
[q, dq, ddq] = min_time_traj(t0, tf, q0, qf, Hz);

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time,q,'r:','linewidth',2)
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time,dq,'r:','linewidth',2)
ylabel('Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time,ddq,'r:','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)



