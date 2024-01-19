% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate Dynamics of 3 Link Planar Manipulator 
% Get desired torque using given desired motion with Inverse Dynamics 
% then simulate Forward Dynamics with desired torque input

% Angle coversion
deg2rad = pi/180;
rad2deg = 180/pi;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System Parameters
L1 = 0.5;       % Link1 length
L2 = 0.5;       % Link2 length
L3 = 0.5;       % Link3 length
L1c = 0.25;     % Link1 CoM length
L2c = 0.25;     % Link2 CoM length
L3c = 0.25;     % Link3 CoM length
m1 = 1;         % Mass 1
m2 = 1;         % Mass 2
m3 = 1;         % Mass 3
I1 = 1;         % Moment of inertia of link 1
I2 = 1;         % Moment of inertia of link 2
I3 = 1;         % Moment of inertia of link 3
g = 9.81;       % Gravity acceleration

% Simulation time
t0 = 0.0;                   % Initial time
tf = 3.0;                   % Final time
time = [t0:1:tf*Hz]*dt;     % Time

% Paramters for desired motion
A1 = pi/4;
A2 = pi/2;
A3 = pi/4;
w1 = 2*pi;
w2 = 2*pi;
w3 = 4*pi;
offset1 = 0;
offset2 = 0;
offset3 = 0;

% Desired joint trajectory 
th_d = [A1*sin(w1*time)+offset1; A2*cos(w2*time)+offset2; A3*sin(w3*time)+offset3]';
dth_d = [A1*w1*cos(w1*time); -A2*w2*sin(w2*time); A3*w3*cos(w3*time)]';
ddth_d = [-A1*w1^2*sin(w1*time); -A2*w2^2*cos(w2*time); -A3*w3^2*sin(w3*time)]';

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h1 = plot(time, th_d(:,1)*rad2deg,'k','linewidth',1);
hold on
h2 = plot(time, th_d(:,2)*rad2deg,'r','linewidth',1);
h3 = plot(time, th_d(:,3)*rad2deg,'b','linewidth',1);
ylabel('Angle [Deg]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_d(:,1)*rad2deg,'k','linewidth',1)
hold on
plot(time, dth_d(:,2)*rad2deg,'r','linewidth',1)
plot(time, dth_d(:,3)*rad2deg,'b','linewidth',1)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_d(:,1)*rad2deg,'k','linewidth',1)
hold on
plot(time, ddth_d(:,2)*rad2deg,'r','linewidth',1)
plot(time, ddth_d(:,3)*rad2deg,'b','linewidth',1)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Inverse Dynamics 
for i = 1:1:length(time)
    
    t = time(i);
    
    % Desired motion
    q1 = th_d(i,1);
    q2 = th_d(i,2);
    q3 = th_d(i,3);
    dq1 = dth_d(i,1);
    dq2 = dth_d(i,2);
    dq3 = dth_d(i,3);
    ddq1 = ddth_d(i,1);
    ddq2 = ddth_d(i,2);
    ddq3 = ddth_d(i,3);
    
    % Inverse Dynamics
    tau1 = I1*ddq1 + I2*ddq1 + I2*ddq2 + I3*ddq1 + I3*ddq2 + I3*ddq3 + L1^2*ddq1*m2 + L1^2*ddq1*m3 + L2^2*ddq1*m3 + L2^2*ddq2*m3 + L1c^2*ddq1*m1 + L2c^2*ddq1*m2 + L2c^2*ddq2*m2 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L2*g*m3*cos(q1 + q2) + L2c*g*m2*cos(q1 + q2) + L1*g*m2*cos(q1) + L1*g*m3*cos(q1) + L1c*g*m1*cos(q1) + L3c*g*m3*cos(q1 + q2 + q3) - L1*L2*dq2^2*m3*sin(q2) - L1*L2c*dq2^2*m2*sin(q2) - L2*L3c*dq3^2*m3*sin(q3) + 2*L1*L3c*ddq1*m3*cos(q2 + q3) + L1*L3c*ddq2*m3*cos(q2 + q3) + L1*L3c*ddq3*m3*cos(q2 + q3) + 2*L1*L2*ddq1*m3*cos(q2) + L1*L2*ddq2*m3*cos(q2) + 2*L1*L2c*ddq1*m2*cos(q2) + L1*L2c*ddq2*m2*cos(q2) + 2*L2*L3c*ddq1*m3*cos(q3) + 2*L2*L3c*ddq2*m3*cos(q3) + L2*L3c*ddq3*m3*cos(q3) - L1*L3c*dq2^2*m3*sin(q2 + q3) - L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2*dq1*dq2*m3*sin(q2) - 2*L1*L2c*dq1*dq2*m2*sin(q2) - 2*L2*L3c*dq1*dq3*m3*sin(q3) - 2*L2*L3c*dq2*dq3*m3*sin(q3) - 2*L1*L3c*dq1*dq2*m3*sin(q2 + q3) - 2*L1*L3c*dq1*dq3*m3*sin(q2 + q3) - 2*L1*L3c*dq2*dq3*m3*sin(q2 + q3);
    tau2 = I2*ddq1 + I2*ddq2 + I3*ddq1 + I3*ddq2 + I3*ddq3 + L2^2*ddq1*m3 + L2^2*ddq2*m3 + L2c^2*ddq1*m2 + L2c^2*ddq2*m2 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L2*g*m3*cos(q1 + q2) + L2c*g*m2*cos(q1 + q2) + L3c*g*m3*cos(q1 + q2 + q3) + L1*L2*dq1^2*m3*sin(q2) + L1*L2c*dq1^2*m2*sin(q2) - L2*L3c*dq3^2*m3*sin(q3) + L1*L3c*ddq1*m3*cos(q2 + q3) + L1*L2*ddq1*m3*cos(q2) + L1*L2c*ddq1*m2*cos(q2) + 2*L2*L3c*ddq1*m3*cos(q3) + 2*L2*L3c*ddq2*m3*cos(q3) + L2*L3c*ddq3*m3*cos(q3) + L1*L3c*dq1^2*m3*sin(q2 + q3) - 2*L2*L3c*dq1*dq3*m3*sin(q3) - 2*L2*L3c*dq2*dq3*m3*sin(q3);
    tau3 = I3*ddq1 + I3*ddq2 + I3*ddq3 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L3c*g*m3*cos(q1 + q2 + q3) + L2*L3c*dq1^2*m3*sin(q3) + L2*L3c*dq2^2*m3*sin(q3) + L1*L3c*ddq1*m3*cos(q2 + q3) + L2*L3c*ddq1*m3*cos(q3) + L2*L3c*ddq2*m3*cos(q3) + L1*L3c*dq1^2*m3*sin(q2 + q3) + 2*L2*L3c*dq1*dq2*m3*sin(q3);
    tau = [tau1 tau2 tau3]';    % Desired torque
    
    tau_out(i,:) = tau;
    
end

figure(2)
h10 = plot(time, tau_out(:,1),'k','linewidth',1);
hold on
h11 = plot(time, tau_out(:,2),'r','linewidth',1);
h12 = plot(time, tau_out(:,3),'b','linewidth',1);
ylabel('Input Torque [Nm]')
xlabel('Time [Sec]')
legend([h10, h11, h12],'Desired torque J1', 'Desired torque J2', 'Desired torque J3')
set(gca,'fontsize',13)

% Forward Dynamics Simulation
% Initialization
th = th_d(1,:)';
dth = dth_d(1,:)';
ddth = ddth_d(1,:)';
for i = 1:1:length(time)
    
    t = time(i);
    
    % Desired input torque
    tau1 = tau_out(i,1);
    tau2 = tau_out(i,2);
    tau3 = tau_out(i,3);

    q1 = th(1);
    q2 = th(2);
    q3 = th(3);
    dq1 = dth(1);
    dq2 = dth(2);
    dq3 = dth(3);
    
    % Dynamic equations of motion
    ddq1 = (4*I2*I3*tau1 - 4*I2*I3*tau2 + 4*I3*L2^2*m3*tau1 - 4*I3*L2^2*m3*tau2 + 4*I3*L2c^2*m2*tau1 + 4*I2*L3c^2*m3*tau1 - 4*I3*L2c^2*m2*tau2 - 4*I2*L3c^2*m3*tau2 + 2*L2^2*L3c^2*m3^2*tau1 - 2*L2^2*L3c^2*m3^2*tau2 + 4*L2c^2*L3c^2*m2*m3*tau1 - 4*L2c^2*L3c^2*m2*m3*tau2 - 2*L2^2*L3c^2*m3^2*tau1*cos(2*q3) + 2*L2^2*L3c^2*m3^2*tau2*cos(2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1*L2^3*dq1^2*m3^2*sin(q2) + 4*I3*L1*L2^3*dq2^2*m3^2*sin(q2) + 4*I3*L1*L2c^3*dq1^2*m2^2*sin(q2) + 4*I3*L1*L2c^3*dq2^2*m2^2*sin(q2) - 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 + q3) - 2*I3*L1*L2^2*g*m3^2*cos(q1) - 2*I3*L1*L2c^2*g*m2^2*cos(q1) - 2*I2*L1*L3c^2*g*m3^2*cos(q1) + 2*I2*L1*L3c^2*g*m3^2*cos(q1 + 2*q2 + 2*q3) - 2*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau3*cos(q2) - 4*I2*L1*L3c*m3*tau3*cos(q2 + q3) + 2*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) + 2*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + 2*I3*L1*L2^2*g*m3^2*cos(q1 + 2*q2) + 2*I3*L1*L2c^2*g*m2^2*cos(q1 + 2*q2) + 4*I2*L1*L3c^3*dq1^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq2^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq3^2*m3^2*sin(q2 + q3) - 4*I2*I3*L1*g*m2*cos(q1) - 4*I2*I3*L1*g*m3*cos(q1) - 4*I2*I3*L1c*g*m1*cos(q1) + 2*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau3*cos(q2 + 2*q3) - 4*I3*L1*L2*m3*tau2*cos(q2) + 4*I3*L1*L2*m3*tau3*cos(q2) - 4*I3*L1*L2c*m2*tau2*cos(q2) + 4*I3*L1*L2c*m2*tau3*cos(q2) + 8*I3*L1*L2^3*dq1*dq2*m3^2*sin(q2) + 8*I3*L1*L2c^3*dq1*dq2*m2^2*sin(q2) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 + q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2) + 4*I2*I3*L1*L3c*dq1^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq2^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1) + 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 - q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 - q3) - 4*L1*L2c^2*L3c*m2*m3*tau3*cos(q2 + q3) + 4*I2*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq2^2*m3*sin(q2) + 4*I2*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq2^2*m2*sin(q2) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1 + 2*q2) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 - 2*q3) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 + 2*q3) + 4*L1*L2c^2*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2^2*g*m2*m3*cos(q1) - 4*I3*L2^2*L1c*g*m1*m3*cos(q1) - 4*I2*L1*L3c^2*g*m2*m3*cos(q1) - 4*I3*L1*L2c^2*g*m2*m3*cos(q1) - 4*I3*L1c*L2c^2*g*m1*m2*cos(q1) - 4*I2*L1c*L3c^2*g*m1*m3*cos(q1) - 4*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau3*cos(q2) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1*L2c^3*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L1*L2c^3*L3c^2*dq2^2*m2^2*m3*sin(q2) + 8*I2*L1*L3c^3*dq1*dq2*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq1*dq3*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq2*dq3*m3^2*sin(q2 + q3) + 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq2^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq3^2*m2*m3*sin(q2 + q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2) + 4*I3*L1*L2*L2c^2*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq2^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq2^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq2^2*m2*m3*sin(q2) + 8*I2*I3*L1*L3c*dq1*dq2*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq1*dq3*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq2*dq3*m3*sin(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 + q3) - 4*L1c*L2c^2*L3c^2*g*m1*m2*m3*cos(q1) + 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2 + 2*q3) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 - q3) - 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1) + 8*I2*I3*L1*L2*dq1*dq2*m3*sin(q2) + 8*I2*I3*L1*L2c*dq1*dq2*m2*sin(q2) + 8*L1*L2c^2*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1 + 2*q2) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 - q3) + 8*L1*L2c^3*L3c^2*dq1*dq2*m2^2*m3*sin(q2) + 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 - q3) - 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 8*I3*L1*L2c^2*L3c*dq1*dq2*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq1*dq3*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) - 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) + 8*I3*L1*L2*L2c^2*dq1*dq2*m2*m3*sin(q2) + 8*I3*L1*L2^2*L2c*dq1*dq2*m2*m3*sin(q2) + 8*I2*L1*L2c*L3c^2*dq1*dq2*m2*m3*sin(q2) + 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 - q3) - 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 - q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2)));
    ddq2 = -(4*I2*I3*tau1 - 4*I1*I3*tau2 + 4*I1*I3*tau3 - 4*I2*I3*tau2 - 4*I3*L1^2*m2*tau2 + 4*I3*L1^2*m2*tau3 - 4*I3*L1^2*m3*tau2 + 4*I3*L2^2*m3*tau1 + 4*I3*L1^2*m3*tau3 - 4*I3*L2^2*m3*tau2 - 4*I3*L1c^2*m1*tau2 + 4*I3*L1c^2*m1*tau3 + 4*I3*L2c^2*m2*tau1 - 4*I1*L3c^2*m3*tau2 + 4*I2*L3c^2*m3*tau1 - 4*I3*L2c^2*m2*tau2 + 4*I1*L3c^2*m3*tau3 - 4*I2*L3c^2*m3*tau2 - 2*L1^2*L3c^2*m3^2*tau2 + 2*L2^2*L3c^2*m3^2*tau1 + 2*L1^2*L3c^2*m3^2*tau3 - 2*L2^2*L3c^2*m3^2*tau2 - 4*L1^2*L3c^2*m2*m3*tau2 + 4*L1^2*L3c^2*m2*m3*tau3 - 4*L1c^2*L3c^2*m1*m3*tau2 + 4*L1c^2*L3c^2*m1*m3*tau3 + 4*L2c^2*L3c^2*m2*m3*tau1 - 4*L2c^2*L3c^2*m2*m3*tau2 - 2*L2^2*L3c^2*m3^2*tau1*cos(2*q3) + 2*L2^2*L3c^2*m3^2*tau2*cos(2*q3) + 2*L1^2*L3c^2*m3^2*tau2*cos(2*q2 + 2*q3) - 2*L1^2*L3c^2*m3^2*tau3*cos(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1*L2^3*dq1^2*m3^2*sin(q2) + 4*I3*L1^3*L2*dq1^2*m3^2*sin(q2) + 4*I3*L1*L2^3*dq2^2*m3^2*sin(q2) + 4*I3*L1*L2c^3*dq1^2*m2^2*sin(q2) + 4*I3*L1^3*L2c*dq1^2*m2^2*sin(q2) + 4*I3*L1*L2c^3*dq2^2*m2^2*sin(q2) - 4*I1*L2*L3c^3*dq1^2*m3^2*sin(q3) - 4*I1*L2*L3c^3*dq2^2*m3^2*sin(q3) - 4*I1*L2*L3c^3*dq3^2*m3^2*sin(q3) - 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2 + 2*q3) + 2*I3*L1^2*L2*g*m3^2*cos(q1 + q2) + 2*I3*L1^2*L2c*g*m2^2*cos(q1 + q2) + 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2) - 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 + q3) - 2*I3*L1*L2^2*g*m3^2*cos(q1) - 2*I3*L1*L2c^2*g*m2^2*cos(q1) - 2*I2*L1*L3c^2*g*m3^2*cos(q1) + 2*I2*L1*L3c^2*g*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I1*I3*L2*g*m3*cos(q1 + q2) + 4*I1*I3*L2c*g*m2*cos(q1 + q2) + 2*L1*L2*L3c^2*m3^2*tau1*cos(q2) - 4*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau3*cos(q2) + 2*L1^2*L2*L3c*m3^2*tau3*cos(q3) - 4*I2*L1*L3c*m3*tau3*cos(q2 + q3) + 4*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) + 2*I3*L1^2*L2^2*dq2^2*m3^2*sin(2*q2) + 4*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + 2*I3*L1^2*L2c^2*dq2^2*m2^2*sin(2*q2) - 2*I1*L2^2*L3c^2*dq1^2*m3^2*sin(2*q3) - 2*I1*L2^2*L3c^2*dq2^2*m3^2*sin(2*q3) - 2*I3*L1^2*L2*g*m3^2*cos(q1 - q2) + 2*I3*L1*L2^2*g*m3^2*cos(q1 + 2*q2) - 2*I3*L1^2*L2c*g*m2^2*cos(q1 - q2) + 2*I3*L1*L2c^2*g*m2^2*cos(q1 + 2*q2) + 4*I2*L1*L3c^3*dq1^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq2^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq3^2*m3^2*sin(q2 + q3) - 4*I2*I3*L1*g*m2*cos(q1) - 4*I2*I3*L1*g*m3*cos(q1) - 4*I2*I3*L1c*g*m1*cos(q1) - 2*L1*L2*L3c^2*m3^2*tau1*cos(q2 + 2*q3) + 4*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau3*cos(q2 + 2*q3) - 2*L1^2*L2*L3c*m3^2*tau3*cos(2*q2 + q3) + 4*I3*L1*L2*m3*tau1*cos(q2) - 8*I3*L1*L2*m3*tau2*cos(q2) + 4*I3*L1*L2*m3*tau3*cos(q2) + 4*I3*L1*L2c*m2*tau1*cos(q2) - 8*I3*L1*L2c*m2*tau2*cos(q2) + 4*I3*L1*L2c*m2*tau3*cos(q2) + 4*I1*L2*L3c*m3*tau3*cos(q3) - 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(2*q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) - 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) + 8*I3*L1*L2^3*dq1*dq2*m3^2*sin(q2) + 8*I3*L1*L2c^3*dq1*dq2*m2^2*sin(q2) - 8*I1*L2*L3c^3*dq1*dq2*m3^2*sin(q3) - 8*I1*L2*L3c^3*dq1*dq3*m3^2*sin(q3) - 8*I1*L2*L3c^3*dq2*dq3*m3^2*sin(q3) + 4*I3*L1^3*L2*dq1^2*m2*m3*sin(q2) + 4*I3*L1^3*L2c*dq1^2*m2*m3*sin(q2) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 + q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2) + 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2) + 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 + q2) + 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2) - 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) - 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) - 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(q3) + 4*I2*I3*L1*L3c*dq1^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq2^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) + 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 2*I3*L1^2*L2*g*m2*m3*cos(q1 + q2) + 4*I3*L2*L1c^2*g*m1*m3*cos(q1 + q2) + 2*I3*L1^2*L2c*g*m2*m3*cos(q1 + q2) + 4*I3*L1c^2*L2c*g*m1*m2*cos(q1 + q2) + 4*I1*L2c*L3c^2*g*m2*m3*cos(q1 + q2) - 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 - q3) + 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 - q3) + 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) + 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(2*q2 + q3) - 4*L1*L2c^2*L3c*m2*m3*tau3*cos(q2 + q3) + 4*I1*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq2^2*m3*sin(q2) + 4*I1*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq2^2*m2*sin(q2) - 4*I1*I3*L2*L3c*dq1^2*m3*sin(q3) - 4*I1*I3*L2*L3c*dq2^2*m3*sin(q3) - 4*I1*I3*L2*L3c*dq3^2*m3*sin(q3) - 2*L1^2*L2^2*L3c^2*dq1^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2^2*L3c^2*dq2^2*m2*m3^2*sin(2*q3) + 4*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) - 2*L2^2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(2*q3) + 2*L1^2*L2c^2*L3c^2*dq2^2*m2^2*m3*sin(2*q2) - 2*L2^2*L1c^2*L3c^2*dq2^2*m1*m3^2*sin(2*q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 - q2) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 - q2) - 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 - q2) + 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1 + 2*q2) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 - 2*q3) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 + 2*q3) + 4*L1*L2c^2*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2^2*g*m2*m3*cos(q1) - 4*I3*L2^2*L1c*g*m1*m3*cos(q1) - 4*I2*L1*L3c^2*g*m2*m3*cos(q1) - 4*I3*L1*L2c^2*g*m2*m3*cos(q1) - 4*I3*L1c*L2c^2*g*m1*m2*cos(q1) - 4*I2*L1c*L3c^2*g*m1*m3*cos(q1) + 4*L1^2*L2*L3c*m2*m3*tau3*cos(q3) + 4*L1*L2c*L3c^2*m2*m3*tau1*cos(q2) - 8*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau3*cos(q2) + 4*L2*L1c^2*L3c*m1*m3*tau3*cos(q3) - 2*L1^2*L2c*L3c*m2*m3*tau3*cos(q3) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1^2*L2^2*dq1*dq2*m3^2*sin(2*q2) + 4*I3*L1^2*L2c^2*dq1*dq2*m2^2*sin(2*q2) - 4*I1*L2^2*L3c^2*dq1*dq2*m3^2*sin(2*q3) + 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2) - 4*L1^2*L2*L3c^3*dq1^2*m2*m3^2*sin(q3) - 4*L1^2*L2*L3c^3*dq2^2*m2*m3^2*sin(q3) - 4*L1^2*L2*L3c^3*dq3^2*m2*m3^2*sin(q3) + 4*L1*L2c^3*L3c^2*dq1^2*m2^2*m3*sin(q2) - 4*L2*L1c^2*L3c^3*dq1^2*m1*m3^2*sin(q3) + 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 4*L1^3*L2c*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L1*L2c^3*L3c^2*dq2^2*m2^2*m3*sin(q2) - 4*L2*L1c^2*L3c^3*dq2^2*m1*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q3) - 4*L2*L1c^2*L3c^3*dq3^2*m1*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q3) - 2*I3*L1^2*L2*g*m2*m3*cos(q1 - q2) - 2*I3*L1^2*L2c*g*m2*m3*cos(q1 - q2) + 8*I2*L1*L3c^3*dq1*dq2*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq1*dq3*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq2*dq3*m3^2*sin(q2 + q3) - 2*L1^2*L2c*L3c*m2*m3*tau3*cos(2*q2 + q3) + 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 - q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(2*q2 + q3) + 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq2^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1c^2*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) - 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q2 - q1 + 2*q3) - 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2) - 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) - 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(q3) - 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(q3) + 4*I3*L1*L2*L1c^2*dq1^2*m1*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq2^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq2^2*m2*m3*sin(q2) - 4*I3*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) - 4*I3*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) - 4*I3*L1^2*L2*L3c*dq3^2*m2*m3*sin(q3) + 4*I3*L1*L1c^2*L2c*dq1^2*m1*m2*sin(q2) + 4*I1*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq2^2*m2*m3*sin(q2) - 4*I3*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) - 4*I3*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) - 4*I3*L2*L1c^2*L3c*dq3^2*m1*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(q3) - 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 + q2) - 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 + q2) + 8*I2*I3*L1*L3c*dq1*dq2*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq1*dq3*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq2*dq3*m3*sin(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 + q3) - 4*L1c*L2c^2*L3c^2*g*m1*m2*m3*cos(q1) + 4*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) + 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q3) + 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2) + 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 - q2) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2 + 2*q3) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 - q3) + 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 - q3) + 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(2*q2 + q3) + 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(2*q2 + q3) - 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1) + 8*I2*I3*L1*L2*dq1*dq2*m3*sin(q2) + 8*I2*I3*L1*L2c*dq1*dq2*m2*sin(q2) - 8*I1*I3*L2*L3c*dq1*dq2*m3*sin(q3) - 8*I1*I3*L2*L3c*dq1*dq3*m3*sin(q3) - 8*I1*I3*L2*L3c*dq2*dq3*m3*sin(q3) - 4*L1^2*L2^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) + 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2^2*m3*sin(2*q2) - 4*L2^2*L1c^2*L3c^2*dq1*dq2*m1*m3^2*sin(2*q3) + 8*L1*L2c^2*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 8*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) + 4*I3*L1^2*L2*L2c*dq2^2*m2*m3*sin(2*q2) - 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 - q2) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1 + 2*q2) - 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 - q2) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 - q3) - 8*L1^2*L2*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 8*L1^2*L2*L3c^3*dq1*dq3*m2*m3^2*sin(q3) - 8*L1^2*L2*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 8*L1*L2c^3*L3c^2*dq1*dq2*m2^2*m3*sin(q2) - 8*L2*L1c^2*L3c^3*dq1*dq2*m1*m3^2*sin(q3) - 8*L2*L1c^2*L3c^3*dq1*dq3*m1*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 8*L2*L1c^2*L3c^3*dq2*dq3*m1*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 - q3) - 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L1c^2*L2c*L3c^2*dq1^2*m1*m2*m3*sin(q2) - 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) + 8*I3*L1*L2c^2*L3c*dq1*dq2*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq1*dq3*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) - 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) + 8*I3*L1*L2*L2c^2*dq1*dq2*m2*m3*sin(q2) + 8*I3*L1*L2^2*L2c*dq1*dq2*m2*m3*sin(q2) - 8*I3*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) - 8*I3*L1^2*L2*L3c*dq1*dq3*m2*m3*sin(q3) - 8*I3*L1^2*L2*L3c*dq2*dq3*m2*m3*sin(q3) + 8*I2*L1*L2c*L3c^2*dq1*dq2*m2*m3*sin(q2) - 8*I3*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) - 8*I3*L2*L1c^2*L3c*dq1*dq3*m1*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) - 8*I3*L2*L1c^2*L3c*dq2*dq3*m1*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(q3) + 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 - q3) + 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2) + 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) - 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 - q2) + 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) + 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(2*q2 + q3) + 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(2*q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) + 8*I3*L1^2*L2*L2c*dq1*dq2*m2*m3*sin(2*q2) - 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 - q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2)));
    ddq3 = -(4*I1*I3*tau2 - 4*I1*I2*tau3 - 4*I1*I3*tau3 - 4*I2*L1^2*m2*tau3 + 4*I3*L1^2*m2*tau2 - 4*I1*L2^2*m3*tau3 - 4*I2*L1^2*m3*tau3 - 4*I3*L1^2*m2*tau3 + 4*I3*L1^2*m3*tau2 - 4*I3*L1^2*m3*tau3 - 4*I2*L1c^2*m1*tau3 + 4*I3*L1c^2*m1*tau2 - 4*I1*L2c^2*m2*tau3 - 4*I3*L1c^2*m1*tau3 + 4*I1*L3c^2*m3*tau2 - 4*I1*L3c^2*m3*tau3 - 2*L1^2*L2^2*m3^2*tau3 - 2*L1^2*L2c^2*m2^2*tau3 + 2*L1^2*L3c^2*m3^2*tau2 - 2*L1^2*L3c^2*m3^2*tau3 - 4*L1^2*L2^2*m2*m3*tau3 - 4*L2^2*L1c^2*m1*m3*tau3 - 4*L1^2*L2c^2*m2*m3*tau3 + 4*L1^2*L3c^2*m2*m3*tau2 - 4*L1^2*L3c^2*m2*m3*tau3 - 4*L1c^2*L2c^2*m1*m2*tau3 + 4*L1c^2*L3c^2*m1*m3*tau2 - 4*L1c^2*L3c^2*m1*m3*tau3 + 2*L1^2*L2^2*m3^2*tau3*cos(2*q2) + 2*L1^2*L2c^2*m2^2*tau3*cos(2*q2) - 2*L1^2*L3c^2*m3^2*tau2*cos(2*q2 + 2*q3) + 2*L1^2*L3c^2*m3^2*tau3*cos(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq2^2*m3^2*sin(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq3^2*m3^2*sin(2*q2 + 2*q3) - 4*I3*L1^3*L2*dq1^2*m3^2*sin(q2) - 4*I3*L1^3*L2c*dq1^2*m2^2*sin(q2) + 4*I1*L2*L3c^3*dq1^2*m3^2*sin(q3) + 4*I1*L2^3*L3c*dq1^2*m3^2*sin(q3) + 4*I1*L2*L3c^3*dq2^2*m3^2*sin(q3) + 4*I1*L2^3*L3c*dq2^2*m3^2*sin(q3) + 4*I1*L2*L3c^3*dq3^2*m3^2*sin(q3) + 4*I1*I2*L3c*g*m3*cos(q1 + q2 + q3) - 2*I1*L2^2*L3c*g*m3^2*cos(q1 + q2 - q3) - 2*I2*L1^2*L3c*g*m3^2*cos(q2 - q1 + q3) + 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2 + 2*q3) - 2*I3*L1^2*L2*g*m3^2*cos(q1 + q2) - 2*I3*L1^2*L2c*g*m2^2*cos(q1 + q2) - 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2) + 2*L1*L2^2*L3c*m3^2*tau1*cos(q2 + q3) - 2*L1*L2^2*L3c*m3^2*tau2*cos(q2 + q3) - 4*I1*I3*L2*g*m3*cos(q1 + q2) - 4*I1*I3*L2c*g*m2*cos(q1 + q2) - 2*L1*L2*L3c^2*m3^2*tau1*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1^2*L2*L3c*m3^2*tau2*cos(q3) - 4*L1^2*L2*L3c*m3^2*tau3*cos(q3) + 4*I2*L1*L3c*m3*tau1*cos(q2 + q3) - 4*I2*L1*L3c*m3*tau2*cos(q2 + q3) - 2*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) - 2*I3*L1^2*L2^2*dq2^2*m3^2*sin(2*q2) - 2*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) - 2*I3*L1^2*L2c^2*dq2^2*m2^2*sin(2*q2) + 4*I1*L2^2*L3c^2*dq1^2*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq2^2*m3^2*sin(2*q3) + 2*I1*L2^2*L3c^2*dq3^2*m3^2*sin(2*q3) + 2*I3*L1^2*L2*g*m3^2*cos(q1 - q2) + 2*I3*L1^2*L2c*g*m2^2*cos(q1 - q2) + 4*I2*L1^3*L3c*dq1^2*m3^2*sin(q2 + q3) - 2*L1*L2^2*L3c*m3^2*tau1*cos(q2 - q3) + 2*L1*L2*L3c^2*m3^2*tau1*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau2*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) - 2*L1^2*L2*L3c*m3^2*tau2*cos(2*q2 + q3) + 4*L1^2*L2*L3c*m3^2*tau3*cos(2*q2 + q3) + 2*I1*L2^2*L3c*g*m3^2*cos(q1 + q2 + q3) + 2*I2*L1^2*L3c*g*m3^2*cos(q1 + q2 + q3) + 4*L1^2*L2*L2c*m2*m3*tau3 - 4*I3*L1*L2*m3*tau1*cos(q2) + 4*I3*L1*L2*m3*tau2*cos(q2) - 4*I3*L1*L2c*m2*tau1*cos(q2) + 4*I3*L1*L2c*m2*tau2*cos(q2) + 4*I1*L2*L3c*m3*tau2*cos(q3) - 8*I1*L2*L3c*m3*tau3*cos(q3) + 4*I2*L1^2*L3c^2*dq1*dq2*m3^2*sin(2*q2 + 2*q3) + 4*I2*L1^2*L3c^2*dq1*dq3*m3^2*sin(2*q2 + 2*q3) + 4*I2*L1^2*L3c^2*dq2*dq3*m3^2*sin(2*q2 + 2*q3) - 2*L1^3*L2^2*L3c*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^3*L2c^2*L3c*dq1^2*m2^2*m3*sin(q2 - q3) - 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c^3*L3c*dq1^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c^3*L3c*dq2^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(2*q2 + q3) - L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 + q2 - q3) + L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 - q2 + q3) - L1^2*L2^2*L3c*g*m2*m3^2*cos(q2 - q1 + q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) - 2*L2^2*L1c^2*L3c*g*m1*m3^2*cos(q1 + q2 - q3) + 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) - 2*L1^2*L2c^2*L3c*g*m2*m3^2*cos(q2 - q1 + q3) + L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 + q2 - q3) - L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 - q2 + q3) - L1^2*L2c^2*L3c*g*m2^2*m3*cos(q2 - q1 + q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) + 8*I1*L2*L3c^3*dq1*dq2*m3^2*sin(q3) + 8*I1*L2^3*L3c*dq1*dq2*m3^2*sin(q3) + 8*I1*L2*L3c^3*dq1*dq3*m3^2*sin(q3) + 8*I1*L2*L3c^3*dq2*dq3*m3^2*sin(q3) - 4*I3*L1^3*L2*dq1^2*m2*m3*sin(q2) - 4*I3*L1^3*L2c*dq1^2*m2*m3*sin(q2) + 4*L1^2*L2*L2c*m2*m3*tau3*cos(2*q2) + 2*I1*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2) - 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2) - 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 + q2) - 2*I2*L1^2*L3c*g*m2*m3*cos(q2 - q1 + q3) - 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) + 2*I2*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(q3) + 4*I1*I2*L1*L3c*dq1^2*m3*sin(q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) - 2*I3*L1^2*L2*g*m2*m3*cos(q1 + q2) - 4*I3*L2*L1c^2*g*m1*m3*cos(q1 + q2) - 2*I3*L1^2*L2c*g*m2*m3*cos(q1 + q2) - 4*I3*L1c^2*L2c*g*m1*m2*cos(q1 + q2) - 4*I1*L2c*L3c^2*g*m2*m3*cos(q1 + q2) - 2*I1*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I2*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) + 2*I2*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(2*q2 + q3) + 4*L1*L2c^2*L3c*m2*m3*tau1*cos(q2 + q3) - 4*L1*L2c^2*L3c*m2*m3*tau2*cos(q2 + q3) - 4*I1*I3*L1*L2*dq1^2*m3*sin(q2) - 4*I1*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I1*I2*L2*L3c*dq1^2*m3*sin(q3) + 4*I1*I2*L2*L3c*dq2^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq1^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq2^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq3^2*m3*sin(q3) + 4*L1^2*L2^2*L3c^2*dq1^2*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq2^2*m2*m3^2*sin(2*q3) + 2*L1^2*L2^2*L3c^2*dq3^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) + 4*L2^2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(2*q3) - 2*L1^2*L2c^2*L3c^2*dq2^2*m2^2*m3*sin(2*q2) + 4*L2^2*L1c^2*L3c^2*dq2^2*m1*m3^2*sin(2*q3) + 2*L2^2*L1c^2*L3c^2*dq3^2*m1*m3^2*sin(2*q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 - q2) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 - q2) + 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 - q2) + 2*L1^3*L2^2*L3c*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1^3*L2c^2*L3c*dq1^2*m2*m3^2*sin(q2 + q3) + 2*L1^3*L2c^2*L3c*dq1^2*m2^2*m3*sin(q2 + q3) + L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 + q2 + q3) + 2*L2^2*L1c^2*L3c*g*m1*m3^2*cos(q1 + q2 + q3) + 2*L1^2*L2c^2*L3c*g*m2*m3^2*cos(q1 + q2 + q3) + L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 + q2 + q3) + 4*L1^2*L2*L3c*m2*m3*tau2*cos(q3) - 8*L1^2*L2*L3c*m2*m3*tau3*cos(q3) - 4*L1*L2c*L3c^2*m2*m3*tau1*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L2*L1c^2*L3c*m1*m3*tau2*cos(q3) - 8*L2*L1c^2*L3c*m1*m3*tau3*cos(q3) - 2*L1^2*L2c*L3c*m2*m3*tau2*cos(q3) + 4*L1^2*L2c*L3c*m2*m3*tau3*cos(q3) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 2*L1^2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(2*q2 + 2*q3) + 2*L1^2*L2c^2*L3c^2*dq3^2*m2*m3^2*sin(2*q2 + 2*q3) - 4*I3*L1^2*L2^2*dq1*dq2*m3^2*sin(2*q2) - 4*I3*L1^2*L2c^2*dq1*dq2*m2^2*sin(2*q2) + 8*I1*L2^2*L3c^2*dq1*dq2*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq1*dq3*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq2*dq3*m3^2*sin(2*q3) - 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 4*L1^2*L2*L3c^3*dq1^2*m2*m3^2*sin(q3) + 4*L1^2*L2^3*L3c*dq1^2*m2*m3^2*sin(q3) + 4*L1^2*L2*L3c^3*dq2^2*m2*m3^2*sin(q3) + 4*L1^2*L2^3*L3c*dq2^2*m2*m3^2*sin(q3) + 4*L1^2*L2*L3c^3*dq3^2*m2*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq1^2*m1*m3^2*sin(q3) - 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) - 4*L1^3*L2c*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L2^3*L1c^2*L3c*dq1^2*m1*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq2^2*m1*m3^2*sin(q3) - 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q3) - 2*L1^2*L2c^3*L3c*dq1^2*m2^2*m3*sin(q3) + 4*L2^3*L1c^2*L3c*dq2^2*m1*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq3^2*m1*m3^2*sin(q3) - 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q3) - 2*L1^2*L2c^3*L3c*dq2^2*m2^2*m3*sin(q3) - 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q3) + 2*I3*L1^2*L2*g*m2*m3*cos(q1 - q2) + 2*I3*L1^2*L2c*g*m2*m3*cos(q1 - q2) + 4*I2*L1^3*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*L1^2*L2c*L3c*m2*m3*tau2*cos(2*q2 + q3) + 4*L1^2*L2c*L3c*m2*m3*tau3*cos(2*q2 + q3) + 2*I2*L1^2*L3c*g*m2*m3*cos(q1 + q2 + q3) + 4*I2*L1c^2*L3c*g*m1*m3*cos(q1 + q2 + q3) + 4*I1*L2c^2*L3c*g*m2*m3*cos(q1 + q2 + q3) + 2*L1^3*L2*L2c*L3c*dq1^2*m2*m3^2*sin(q2 - q3) - 2*L1^3*L2*L2c*L3c*dq1^2*m2^2*m3*sin(q2 - q3) + 2*L1*L2^2*L1c^2*L3c*dq1^2*m1*m3^2*sin(q2 + q3) + L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 + q2 - q3) + L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 - q2 + q3) - L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q2 - q1 + q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 + q2 - q3) - L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 - q2 + q3) + 3*L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q2 - q1 + q3) - L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 + q2 - q3) + L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 - q2 + q3) + L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q2 - q1 + q3) - 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c^3*L3c*dq1*dq2*m2^2*m3*sin(2*q2 + q3) - 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(2*q2 + q3) - 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(2*q2 + q3) - 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2^2*m3*sin(q3) - 6*L1^2*L2^2*L2c*L3c*dq1^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2^2*m3*sin(q3) - 6*L1^2*L2^2*L2c*L3c*dq2^2*m2*m3^2*sin(q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2) + 4*I2*L1*L1c^2*L3c*dq1^2*m1*m3*sin(q2 + q3) + 4*I1*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I2*L1*L1c*L3c*g*m1*m3*cos(q2 - q1 + q3) - 2*I1*L2*L2c*L3c*g*m2*m3*cos(q1 + q2 - q3) - 4*L1c^2*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) - 2*L1*L2^2*L1c^2*L3c*dq1^2*m1*m3^2*sin(q2 - q3) + 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2*m3^2*sin(2*q2 + q3) - 2*L1^2*L2*L2c^2*L3c*dq1^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2^2*L2c*L3c*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2*m3^2*sin(2*q2 + q3) - 2*L1^2*L2*L2c^2*L3c*dq2^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2^2*L2c*L3c*dq2^2*m2*m3^2*sin(2*q2 + q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q2 - q1 + 2*q3) + 4*I2*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(q3) - 4*I3*L1*L2*L1c^2*dq1^2*m1*m3*sin(q2) + 4*I2*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) + 4*I2*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq3^2*m2*m3*sin(q3) - 4*I3*L1*L1c^2*L2c*dq1^2*m1*m2*sin(q2) - 4*I1*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) + 4*I1*L2*L2c^2*L3c*dq1^2*m2*m3*sin(q3) + 4*I2*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) - 2*I2*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) + 4*I1*L2*L2c^2*L3c*dq2^2*m2*m3*sin(q3) - 2*I2*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq3^2*m1*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(q3) + 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 + q2) + 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 + q2) - 2*L1*L2*L2c*L3c*m2*m3*tau1*cos(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau2*cos(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) - 4*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2) - 4*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq3^2*m2*m3^2*sin(2*q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 - q2) + 4*I2*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(2*q2 + q3) + 2*I2*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) + 2*I2*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(2*q2 + q3) - 6*L1^3*L2*L2c*L3c*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1^3*L2*L2c*L3c*dq1^2*m2^2*m3*sin(q2 + q3) + 8*I1*I2*L2*L3c*dq1*dq2*m3*sin(q3) + 8*I1*I3*L2*L3c*dq1*dq2*m3*sin(q3) + 8*I1*I3*L2*L3c*dq1*dq3*m3*sin(q3) + 8*I1*I3*L2*L3c*dq2*dq3*m3*sin(q3) - L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 + q2 + q3) - 3*L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 + q2 + q3) - L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 + q2 + q3) + 8*L1^2*L2^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq1*dq3*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq2*dq3*m2*m3^2*sin(2*q3) - 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2^2*m3*sin(2*q2) + 8*L2^2*L1c^2*L3c^2*dq1*dq2*m1*m3^2*sin(2*q3) + 4*L2^2*L1c^2*L3c^2*dq1*dq3*m1*m3^2*sin(2*q3) + 4*L2^2*L1c^2*L3c^2*dq2*dq3*m1*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1^2*L2*L2c*L3c^2*dq3^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1c^2*L2c^2*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 4*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) - 4*I3*L1^2*L2*L2c*dq2^2*m2*m3*sin(2*q2) + 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 - q2) + 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 - q2) + 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1^2*L2c^2*L3c^2*dq1*dq3*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1^2*L2c^2*L3c^2*dq2*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1*L2*L2c*L3c*m2*m3*tau1*cos(q2 - q3) + 2*L1*L2*L2c*L3c*m2*m3*tau2*cos(q2 - q3) - 2*I2*L1*L1c*L3c*g*m1*m3*cos(q1 + q2 + q3) - 2*I1*L2*L2c*L3c*g*m2*m3*cos(q1 + q2 + q3) + 8*L1^2*L2*L3c^3*dq1*dq2*m2*m3^2*sin(q3) + 8*L1^2*L2^3*L3c*dq1*dq2*m2*m3^2*sin(q3) + 8*L1^2*L2*L3c^3*dq1*dq3*m2*m3^2*sin(q3) + 8*L1^2*L2*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 8*L2*L1c^2*L3c^3*dq1*dq2*m1*m3^2*sin(q3) + 8*L2^3*L1c^2*L3c*dq1*dq2*m1*m3^2*sin(q3) + 8*L2*L1c^2*L3c^3*dq1*dq3*m1*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 4*L1^2*L2c^3*L3c*dq1*dq2*m2^2*m3*sin(q3) + 8*L2*L1c^2*L3c^3*dq2*dq3*m1*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 4*L1*L1c^2*L2c^2*L3c*dq1^2*m1*m2*m3*sin(q2 + q3) - 2*L1*L1c*L2c^2*L3c*g*m1*m2*m3*cos(q2 - q1 + q3) - 2*L2*L1c^2*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 - q3) - 2*I1*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2*m3^2*sin(q3) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2^2*m3*sin(q3) - 12*L1^2*L2^2*L2c*L3c*dq1*dq2*m2*m3^2*sin(q3) - 4*L1*L1c^2*L2c*L3c^2*dq1^2*m1*m2*m3*sin(q2) + 4*L2*L1c^2*L2c^2*L3c*dq1^2*m1*m2*m3*sin(q3) + 4*L2*L1c^2*L2c^2*L3c*dq2^2*m1*m2*m3*sin(q3) + 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2*m3^2*sin(2*q2 + q3) - 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2^2*m3*sin(2*q2 + q3) - 4*L1^2*L2^2*L2c*L3c*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 8*I2*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq1*dq3*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq2*dq3*m2*m3*sin(q3) + 8*I2*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) + 8*I1*L2*L2c^2*L3c*dq1*dq2*m2*m3*sin(q3) - 4*I2*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq1*dq3*m1*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq2*dq3*m1*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(q3) - 2*I1*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2) - 8*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq3*m2*m3^2*sin(2*q3) - 4*L1^2*L2*L2c*L3c^2*dq2*dq3*m2*m3^2*sin(2*q3) + 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 - q2) + 4*I2*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(2*q2 + q3) - 2*L1*L1c*L2c^2*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 2*L2*L1c^2*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2 + 2*q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 4*L1^2*L2*L2c*L3c^2*dq2*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 8*I3*L1^2*L2*L2c*dq1*dq2*m2*m3*sin(2*q2) + 8*L2*L1c^2*L2c^2*L3c*dq1*dq2*m1*m2*m3*sin(q3) - 2*L1*L2*L1c^2*L2c*L3c*dq1^2*m1*m2*m3*sin(q2 - q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 2*L1*L2*L1c^2*L2c*L3c*dq1^2*m1*m2*m3*sin(q2 + q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 - q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 - q2 + q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q2 - q1 + q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2)));
    ddth = [ddq1 ddq2 ddq3]';
    
    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;
    
    ddth_out(i,:) = ddth;
    dth_out(i,:) = dth;
    th_out(i,:) = th;
    
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h4 = plot(time, th_out(:,1)*rad2deg,'k:','linewidth',3);
hold on
h5 = plot(time, th_out(:,2)*rad2deg,'r:','linewidth',3);
h6 = plot(time, th_out(:,3)*rad2deg,'b:','linewidth',3);
ylabel('Angle [Deg]')
legend([h1, h2, h3, h4, h5, h6],'Desired traj J1', 'Desired traj J2', 'Desired traj J3', 'Output traj from FD J1', 'Output traj from FD J2', 'Output traj from FD J3')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_out(:,1)*rad2deg,'k:','linewidth',3)
hold on
plot(time, dth_out(:,2)*rad2deg,'r:','linewidth',3)
plot(time, dth_out(:,3)*rad2deg,'b:','linewidth',3)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_out(:,1)*rad2deg,'k:','linewidth',3)
hold on
plot(time, ddth_out(:,2)*rad2deg,'r:','linewidth',3)
plot(time, ddth_out(:,3)*rad2deg,'b:','linewidth',3)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Visualize Simulation with Forward Kinematics
step = 200;

% Camera View
az = 0;
el = 90;

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Forward Kinematics
for i = 1:step:length(time)
    
    % Sinusoidal Input Joint Angle
    th1 = th_out(i,1);
    th2 = th_out(i,2);
    th3 = th_out(i,3);
    
    % Forward Kinematics using DH convention
    % DH parameter Link 1
    a1 = L1; alpha1 = 0; d1 = 0; q1 = th1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = L2; alpha2 = 0; d2 = 0; q2 = th2; 
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = L3; alpha3 = 0; d3 = 0; q3 = th3;
    T23 = DH(q3, d3, alpha3, a3);

    T02 = T01*T12;
    T03 = T02*T23;

    % Position and axis infomation
    axis_scale = 0.2;
    P1 = T01(1:3,4);
    P1_x = P1 + axis_scale * T01(1:3,1);
    P1_y = P1 + axis_scale * T01(1:3,2);
    P1_z = P1 + axis_scale * T01(1:3,3); 

    P2 = T02(1:3,4);
    P2_x = P2 + axis_scale * T02(1:3,1);
    P2_y = P2 + axis_scale * T02(1:3,2); 
    P2_z = P2 + axis_scale * T02(1:3,3);

    P3 = T03(1:3,4);
    P3_x = P3 + axis_scale * T03(1:3,1);
    P3_y = P3 + axis_scale * T03(1:3,2);
    P3_z = P3 + axis_scale * T03(1:3,3);

    figure(100)
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 3)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 3)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 3)

    plot3(P1(1),P1(2),P1(3),'ro','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)
    line([P1(1) P1_x(1)],[P1(2) P1_x(2)],[P1(3) P1_x(3)], 'color', 'r', 'linewidth', 1)
    line([P1(1) P1_y(1)],[P1(2) P1_y(2)],[P1(3) P1_y(3)], 'color','g', 'linewidth', 1)
    line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color','b', 'linewidth', 1)

    plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
    line([P2(1) P2_x(1)],[P2(2) P2_x(2)],[P2(3) P2_x(3)], 'color', 'r', 'linewidth', 1)
    line([P2(1) P2_y(1)],[P2(2) P2_y(2)],[P2(3) P2_y(3)], 'color','g', 'linewidth', 1)
    line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color','b', 'linewidth', 1)

    plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
    line([P3(1) P3_x(1)],[P3(2) P3_x(2)],[P3(3) P3_x(3)], 'color', 'r', 'linewidth', 1)
    line([P3(1) P3_y(1)],[P3(2) P3_y(2)],[P3(3) P3_y(3)], 'color','g', 'linewidth', 1)
    line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color','b', 'linewidth', 1)

    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title('3-Link Planar Manipulator')
    set(gca,'fontsize',13)
    axis equal
    axis([-1 1 -1 1 -1 1]*1.5)
    grid on
    view(az, el)
    pause(0.001)
    hold off
end
