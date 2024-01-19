% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian Analysis of 2 Link Planar Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 15.0;
A2 = 15.0;
w1 = 2*pi;
w2 = 2*pi;
offset1 = 45.0;
offset2 = 45.0;

% Time
t = [0:1:Hz*Rep]*dt;

% Joint angles (Deg)
th1_deg = A1*sin(w1*t) + offset1;
th2_deg = A2*cos(w2*t) + offset2;
% Joint angular velocity (Deg/s)
dth1_deg = A1*w1*cos(w1*t);
dth2_deg = -A2*w2*sin(w2*t);

% Degree to radian conversion
th1 = th1_deg * deg2rad;
th2 = th2_deg * deg2rad;
dth1 = dth1_deg * deg2rad;
dth2 = dth2_deg * deg2rad;

% Plot given joint trajectory
figure(1)
subplot(2,1,1)
h_th1 = plot(t,th1_deg,'r','linewidth',1);
hold on
h_th2 = plot(t,th2_deg,'b','linewidth',1);
ylabel('Joint Angle [Deg]')
set(gca,'fontsize',13)
subplot(2,1,2)
h_dth1 = plot(t,dth1_deg,'r','linewidth',1);
hold on
h_dth2 = plot(t,dth2_deg,'b','linewidth',1);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward kinematics
Px = L1*cos(th1) + L2*cos(th1+th2);
Py = L1*sin(th1) + L2*sin(th1+th2);

% Plot End-point trajectory
figure(2)
h_Px = plot(t,Px,'r','linewidth',1);
hold on
h_Py = plot(t,Py,'g','linewidth',1);
ylabel('End-effector Position [m]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward kinematics with Jacobian
% Given initial position values
P_J = [Px(1) Py(1)];
for i = 0:1:Hz*Rep
    i = i+1;
    
    % Analytical Jacobian
    J = [-L1*sin(th1(i))-L2*sin(th1(i)+th2(i)) -L2*sin(th1(i)+th2(i)); L1*cos(th1(i))+L2*cos(th1(i)+th2(i)) L2*cos(th1(i)+th2(i))];

    dP(i,:) = J*[dth1(i) dth2(i)]';
    
    P_J = P_J + dP(i,:)*dt;
    
    Px_J(i) = P_J(1);
    Py_J(i) = P_J(2);
    
end

% Plot End-point trajectory from Jacobian
figure(2)
h_Px_J = plot(t,Px_J,'r:','linewidth',2);
hold on
h_Py_J = plot(t,Py_J,'g:','linewidth',2);
ylabel('End-effector Position [m]')
xlabel('Time [Sec]')
legend([h_Px h_Px_J h_Py h_Py_J],'Px (FK)','Px (J)','Py (FK)','Py (J)')
set(gca,'fontsize',13)

% Inverse kinematics with Jacobian
% Given initial joint values
th_J = [th1(1) th2(1)];
for i = 0:1:Hz*Rep
    i = i+1;
    
    % Analytical Jacobian
    J = [-L1*sin(th_J(1))-L2*sin(th_J(1)+th_J(2)) -L2*sin(th_J(1)+th_J(2)); L1*cos(th_J(1))+L2*cos(th_J(1)+th_J(2)) L2*cos(th_J(1)+th_J(2))];

    % Inverse Jacobian
    dth_J(i,:) = inv(J)*[dP(i,1) dP(i,2)]';
    
    th_J = th_J + dth_J(i,:)*dt;
    
    th1_J(i) = th_J(1);
    th2_J(i) = th_J(2);
    
end

dth_J_deg = dth_J * rad2deg;
th1_J_deg = th1_J * rad2deg;
th2_J_deg = th2_J * rad2deg;

% Plot joint trajectory from Inverse kinematics using Jacobian
figure(1)
subplot(2,1,1)
h_th1_J = plot(t,th1_J_deg,'r:','linewidth',2);
hold on
h_th2_J = plot(t,th2_J_deg,'b:','linewidth',2);
ylabel('Joint Angle [Deg]')
legend([h_th1 h_th2 h_th1_J h_th2_J],'th1 (given)', 'th1 (J)', 'th2 (given)', 'th2 (J)')
set(gca,'fontsize',13)
subplot(2,1,2)
h_dth1_J = plot(t,dth_J_deg(:,1),'r:','linewidth',2);
hold on
h_dth2_J = plot(t,dth_J_deg(:,2),'b:','linewidth',2);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
legend([h_dth1 h_dth2 h_dth1_J h_dth2_J],'dth1 (given)', 'dth1 (J)', 'dth2 (given)', 'dth2 (J)')
set(gca,'fontsize',13)

% Motion Simulation
step = 100;
% Forward Kinematics
for i = 1:step:Hz*Rep

    % Sinusoidal Input Joint Angle with given theta
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;

    P1(1) = L1*cos(th1);
    P1(2) = L1*sin(th1);

    P2(1) = L1*cos(th1) + L2*cos(th1+th2);
    P2(2) = L1*sin(th1) + L2*sin(th1+th2);

    % Sinusoidal Input Joint Angle with Inverse kinematics from Jacobian
    th1_J = th1_J_deg(i) * deg2rad;
    th2_J = th2_J_deg(i) * deg2rad;

    P1_J(1) = L1*cos(th1_J);
    P1_J(2) = L1*sin(th1_J);

    P2_J(1) = L1*cos(th1_J) + L2*cos(th1_J+th2_J);
    P2_J(2) = L1*sin(th1_J) + L2*sin(th1_J+th2_J);

    figure(100)
    % Given theta
    plot(P1(1), P1(2),'ko','linewidth',2)
    hold on
    plot(P2(1), P2(2),'ko','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)], 'color', 'k', 'linewidth', 2)
    line([P1(1) P2(1)],[P1(2) P2(2)], 'color','k', 'linewidth', 2)

    % From Jacobian
    plot(P1_J(1), P1_J(2),'rd','linewidth',2)
    plot(P2_J(1), P2_J(2),'rd','linewidth',2)
    line([origin(1) P1_J(1)],[origin(2) P1_J(2)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    line([P1_J(1) P2_J(1)],[P1_J(2) P2_J(2)], 'color','r', 'linewidth', 3, 'linestyle',':')

    % Draw global coordinate
    plot(origin(1), origin(2), 'ro', 'linewidth',5)
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 2)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 2)
    
    % Draw Path
    plot(Px_J, Py_J, 'k:','linewidth',1)
    axis equal
    axis([-2 2 -2 2])
    xlabel('x- axis')
    ylabel('y- axis')
    title('2-Link Planar Manipulator')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off

    
end