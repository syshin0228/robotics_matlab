% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate Forward dynamics of 2 Link Planar Manipulator 
% Apply Inverse Dynamics Control

% Angle coversion
deg2rad = pi/180;
rad2deg = 180/pi;

% Sampling rate info
Hz = 1000;
dt = 1/Hz;

% System Parameters
L1 = 0.5;       % Link1 length
L2 =0.5;        % Link2 length
L1c = 0.25;     % Link1 CoM length
L2c = 0.25;     % Link2 CoM length
m1 = 1;         % Mass 1
m2 = 1;         % Mass 2
I1 = 1;         % Moment of inertia of link 1
I2 = 1;         % Moment of inertia of link 2
g = 9.81;       % Gravity acceleration
B = 0.0;        % Damping coefficient

% Simulation time
t0 = 0.0;                   % Initial time
tf = 3.0;                   % Final time
time = [t0:1:tf*Hz]*dt;     % Time

% Control Gain
w = [10 10]';
zeta = 1.0;
Kp = [w(1)^2 w(2)^2]';
Kd = zeta*[2*w(1) 2*w(2)]';

% Desired step input
th_d = [60 90]' * deg2rad;
ddth_d = [0 0]' * deg2rad;
 
% Forward Dynamics 
% Initialization
th = [0 0]';
dth = [0 0]';
ddth = [0 0]';
for i = 1:1:length(time)
    
    t = time(i);
    
    % Current variables (joint positions, joint velocities)
    q1 = th(1);
    q2 = th(2);
    dq1 = dth(1);
    dq2 = dth(2);
    
    % Control Input
    % Outer Loop Controller
    a_q(1) = ddth_d(1) -Kd(1)*dq1 + Kp(1)*(th_d(1) - q1);
    a_q(2) = ddth_d(2) -Kd(2)*dq2 + Kp(2)*(th_d(2) - q2);
    
    % Inner Loop Controller
    ddq1 = a_q(1);
    ddq2 = a_q(2);
    
    u(1) = ddq2*(m2*L2c^2 + L1*m2*cos(q2)*L2c + I2) + ddq1*(m2*L1^2 + 2*m2*cos(q2)*L1*L2c + m1*L1c^2 + m2*L2c^2 + I1 + I2) + g*m2*(L2c*cos(q1 + q2) + L1*cos(q1)) + L1c*g*m1*cos(q1) - L1*L2c*dq2*m2*sin(q2)*(2*dq1 + dq2);
    u(2) = L1*L2c*m2*sin(q2)*dq1^2 + ddq1*(m2*L2c^2 + L1*m2*cos(q2)*L2c + I2) + ddq2*(m2*L2c^2 + I2) + L2c*g*m2*cos(q1 + q2);  
    tau1 = u(1) -B*dth(1);
    tau2 = u(2) -B*dth(2);
    
    % Dynamic equations of motion
    ddq1 = (2*I2*tau1 - 2*I2*tau2 + 2*L2c^2*m2*tau1 - 2*L2c^2*m2*tau2 - L1*L2c^2*g*m2^2*cos(q1) + L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + L1*L2c^2*g*m2^2*cos(q1 + 2*q2) - 2*I2*L1*g*m2*cos(q1) - 2*I2*L1c*g*m1*cos(q1) - 2*L1*L2c*m2*tau2*cos(q2) + 2*L1*L2c^3*dq1^2*m2^2*sin(q2) + 2*L1*L2c^3*dq2^2*m2^2*sin(q2) + 2*I2*L1*L2c*dq1^2*m2*sin(q2) + 2*I2*L1*L2c*dq2^2*m2*sin(q2) - 2*L1c*L2c^2*g*m1*m2*cos(q1) + 4*L1*L2c^3*dq1*dq2*m2^2*sin(q2) + 4*I2*L1*L2c*dq1*dq2*m2*sin(q2))/(2*I1*I2 + L1^2*L2c^2*m2^2 + 2*I2*L1^2*m2 + 2*I2*L1c^2*m1 + 2*I1*L2c^2*m2 + 2*L1c^2*L2c^2*m1*m2 - L1^2*L2c^2*m2^2*cos(2*q2));
    ddq2 = -(I2*tau1 - I1*tau2 - I2*tau2 - L1^2*m2*tau2 - L1c^2*m1*tau2 + L2c^2*m2*tau1 - L2c^2*m2*tau2 + L1^2*L2c*g*m2^2*cos(q1 + q2) - L1*L2c^2*g*m2^2*cos(q1) + I1*L2c*g*m2*cos(q1 + q2) + L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + (L1^2*L2c^2*dq2^2*m2^2*sin(2*q2))/2 - I2*L1*g*m2*cos(q1) - I2*L1c*g*m1*cos(q1) + L1*L2c*m2*tau1*cos(q2) - 2*L1*L2c*m2*tau2*cos(q2) + L1*L2c^3*dq1^2*m2^2*sin(q2) + L1^3*L2c*dq1^2*m2^2*sin(q2) + L1*L2c^3*dq2^2*m2^2*sin(q2) + L1*L2c^2*g*m2^2*cos(q1 + q2)*cos(q2) - L1^2*L2c*g*m2^2*cos(q1)*cos(q2) + L1c^2*L2c*g*m1*m2*cos(q1 + q2) + I1*L1*L2c*dq1^2*m2*sin(q2) + I2*L1*L2c*dq1^2*m2*sin(q2) + I2*L1*L2c*dq2^2*m2*sin(q2) - L1c*L2c^2*g*m1*m2*cos(q1) + L1^2*L2c^2*dq1*dq2*m2^2*sin(2*q2) + 2*L1*L2c^3*dq1*dq2*m2^2*sin(q2) + L1*L1c^2*L2c*dq1^2*m1*m2*sin(q2) + 2*I2*L1*L2c*dq1*dq2*m2*sin(q2) - L1*L1c*L2c*g*m1*m2*cos(q1)*cos(q2))/(I1*I2 + L1^2*L2c^2*m2^2 + I2*L1^2*m2 + I2*L1c^2*m1 + I1*L2c^2*m2 + L1c^2*L2c^2*m1*m2 - L1^2*L2c^2*m2^2*cos(q2)^2);
    
    ddth = [ddq1 ddq2]';
    
    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;
    
    ddth_out(i,:) = ddth;
    dth_out(i,:) = dth;
    th_out(i,:) = th;
    u_out(i,:) = u;
    
end

figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h1 = plot(time, th_out(:,1)*rad2deg,'k','linewidth',2);
hold on
h2 = plot(time, th_out(:,2)*rad2deg,'r','linewidth',2);
h3 = plot(time, ones(length(time),1)*th_d(1)*rad2deg,'k:','linewidth',1);
h4 = plot(time, ones(length(time),1)*th_d(2)*rad2deg,'r:','linewidth',1);
ylabel('Angle [Deg]')
legend([h1, h2, h3, h4],'J1','J2','desired J1','desired J2')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_out(:,1)*rad2deg,'k','linewidth',2)
hold on
plot(time, dth_out(:,2)*rad2deg,'r','linewidth',2)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_out(:,1)*rad2deg,'k','linewidth',2)
hold on
plot(time, ddth_out(:,2)*rad2deg,'r','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
h3 = plot(time, u_out(:,1),'k','linewidth',2);
hold on
h4 = plot(time, u_out(:,2),'r','linewidth',2);
ylabel('Input Torque [Nm]')
xlabel('Time [Sec]')
legend([h3, h4],'J1','J2')
set(gca,'fontsize',13)

% Visualize Simulation
step = 50;

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';
% Forward Kinematics
for i = 1:step:length(time)
    
    % Sinusoidal Input Joint Angle
    th1 = th_out(i,1);
    th2 = th_out(i,2);

    P1(1) = L1*cos(th1);
    P1(2) = L1*sin(th1);

    P2(1) = L1*cos(th1) + L2*cos(th1+th2);
    P2(2) = L1*sin(th1) + L2*sin(th1+th2);

    figure(100)
    plot(P1(1), P1(2),'ro','linewidth',3)
    hold on
    plot(P2(1), P2(2),'ro','linewidth',3)
    line([origin(1) P1(1)],[origin(2) P1(2)], 'color', 'k', 'linewidth', 3)
    line([P1(1) P2(1)],[P1(2) P2(2)], 'color','k', 'linewidth', 3)

    % Draw global coordinate
    plot(origin(1), origin(2), 'ro', 'linewidth',5)
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 2)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 2)
    axis equal
    axis([-1 1 -1 1])
    xlabel('x- axis')
    ylabel('y- axis')
    title('2-Link Planar Manipulator')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off
    
end
