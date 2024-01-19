% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian Analysis of 3-Link Planar Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Camera View
az = 0;
el = 90;

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;
L3 = 1.0;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 30.0;
A2 = 15.0;
A3 = 15.0;
w1 = 2*pi;
w2 = 2*pi;
w3 = 2*pi;
offset1 = 45.0;
offset2 = 45.0;
offset3 = 45.0;

% Time
t = [0:1:Hz*Rep]*dt;

% Joint angles (Deg)
th1_deg = A1*sin(w1*t) + offset1;
th2_deg = A2*cos(w2*t) + offset2;
th3_deg = A3*sin(w3*t) + offset3;
% Joint angular velocity (Deg/s)
dth1_deg = A1*w1*cos(w1*t);
dth2_deg = -A2*w2*sin(w2*t);
dth3_deg = A3*w3*cos(w3*t);

% Plot given joint trajectory
figure(1)
subplot(2,1,1)
h_th1 = plot(t,th1_deg,'r','linewidth',1);
hold on
h_th2 = plot(t,th2_deg,'b','linewidth',1);
h_th3 = plot(t,th3_deg,'k','linewidth',1);
ylabel('Joint Angle [Deg]')
set(gca,'fontsize',13)
subplot(2,1,2)
h_dth1 = plot(t,dth1_deg,'r','linewidth',1);
hold on
h_dth2 = plot(t,dth2_deg,'b','linewidth',1);
h_dth3 = plot(t,dth3_deg,'k','linewidth',1);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward kinematics with Jacobian
P_J = [0 0 0 0 0 0];    % Zero initially
for i = 0:1:Hz*Rep

    i = i+1;
    
    % Sinusoidal Input Joint Angle
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    th3 = th3_deg(i) * deg2rad;
    dth1 = dth1_deg(i) * deg2rad;
    dth2 = dth2_deg(i) * deg2rad;
    dth3 = dth3_deg(i) * deg2rad;
    
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
    
    % Get position from Forward kinematics
    P(i,:) = T03(1:3,4);

    % Jacobian calculation
    z0 = [0 0 1]';
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);

    o0 = [0 0 0]';
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);

    J1 = [w2ss(z0)*(o3-o0); z0];
    J2 = [w2ss(z1)*(o3-o1); z1];
    J3 = [w2ss(z2)*(o3-o2); z2];
    
    % Jacobian
    J = [J1 J2 J3];

    dP(i,:) = J*[dth1 dth2 dth3]';
    P_J = P_J + dP(i,:)*dt;
    
    Px_J(i) = P_J(1);
    Py_J(i) = P_J(2);
    Pz_J(i) = P_J(3);
 
end

% From given initial position value (P(1,:))
Px_J = Px_J + P(1,1);
Py_J = Py_J + P(1,2);
Pz_J = Pz_J + P(1,3);

% Plot End-point trajectory from Foward kinematics and Jacobian
figure(2)
h_Px = plot(t,P(:,1),'r','linewidth',1);
hold on
h_Py = plot(t,P(:,2),'g','linewidth',1);
h_Px_J = plot(t,Px_J,'r:','linewidth',2);
h_Py_J = plot(t,Py_J,'g:','linewidth',2);
ylabel('End-effector Position [m]')
xlabel('Time [Sec]')
legend([h_Px h_Px_J h_Py h_Py_J],'Px (FK)', 'Px (J)','Py (FK)', 'Py (J)')
set(gca,'fontsize',13)

% Inverse kinematics with Jacobian
% Given initial joint values
th_J = [th1_deg(1) th2_deg(1) th3_deg(1)] *deg2rad;
for i = 0:1:Hz*Rep
    i = i+1;
    
    % Joint Angle
    th1 = th_J(1);
    th2 = th_J(2);
    th3 = th_J(3);

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
    
    % Jacobian calculation
    z0 = [0 0 1]';
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);

    o0 = [0 0 0]';
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);

    J1 = [w2ss(z0)*(o3-o0); z0];
    J2 = [w2ss(z1)*(o3-o1); z1];
    J3 = [w2ss(z2)*(o3-o2); z2];
    
    % Jacobian
    J = [J1 J2 J3];

    % Pseudoinverse Jacobian
    dth_J(i,:) = pinv(J)*dP(i,:)';
    
    th_J = th_J + dth_J(i,:)*dt;
    
    th1_J(i) = th_J(1);
    th2_J(i) = th_J(2);
    th3_J(i) = th_J(3);
    
end    

dth_J_deg = dth_J * rad2deg;
th1_J_deg = th1_J * rad2deg;
th2_J_deg = th2_J * rad2deg;
th3_J_deg = th3_J * rad2deg;

% Plot joint trajectory from Inverse kinematics using Jacobian
figure(1)
subplot(2,1,1)
h_th1_J = plot(t,th1_J_deg,'r:','linewidth',2);
hold on
h_th2_J = plot(t,th2_J_deg,'b:','linewidth',2);
h_th3_J = plot(t,th3_J_deg,'k:','linewidth',2);
ylabel('Joint Angle [Deg]')
legend([h_th1 h_th1_J h_th2 h_th2_J h_th3 h_th3_J],'th1 (given)', 'th1 (J)','th2 (given)', 'th2 (J)','th3 (given)', 'th3 (J)')
set(gca,'fontsize',13)
subplot(2,1,2)
h_dth1_J = plot(t,dth_J_deg(:,1),'r:','linewidth',2);
hold on
h_dth2_J = plot(t,dth_J_deg(:,2),'b:','linewidth',2);
h_dth3_J = plot(t,dth_J_deg(:,3),'k:','linewidth',2);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
legend([h_dth1 h_dth1_J h_dth2 h_dth2_J h_dth3 h_dth3_J],'dth1 (given)', 'dth1 (J)','dth2 (given)', 'dth2 (J)','dth3 (given)', 'dth3 (J)')
set(gca,'fontsize',13)

% Motion Simulation
step = 100;
% Forward Kinematics
for i = 1:step:Hz*Rep
    
    % Sinusoidal Input Joint Angle with given theta
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    th3 = th3_deg(i) * deg2rad;
    
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
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);
    
    % Sinusoidal Input Joint Angle from Jacobian
    th1_J = th1_J_deg(i) * deg2rad;
    th2_J = th2_J_deg(i) * deg2rad;
    th3_J = th3_J_deg(i) * deg2rad;
    
    % Forward Kinematics using DH convention
    % DH parameter Link 1
    a1 = L1; alpha1 = 0; d1 = 0; q1 = th1_J;
    T01_J = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = L2; alpha2 = 0; d2 = 0; q2 = th2_J; 
    T12_J = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = L3; alpha3 = 0; d3 = 0; q3 = th3_J;
    T23_J = DH(q3, d3, alpha3, a3);

    T02_J = T01_J*T12_J;
    T03_J = T02_J*T23_J;

    % Position and axis infomation
    P1_J = T01_J(1:3,4);
    P2_J = T02_J(1:3,4);
    P3_J = T03_J(1:3,4);

    figure(100)
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 3)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 3)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 3)

    % P1 from given theta
    plot3(P1(1),P1(2),P1(3),'ko','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)
    % P2 from given theta
    plot3(P2(1),P2(2),P2(3),'ko','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
    % P3 from given theta
    plot3(P3(1),P3(2),P3(3),'ko','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)

    % P1 from Jacobian
    plot3(P1_J(1),P1_J(2),P1_J(3),'rd','linewidth',2)
    line([origin(1) P1_J(1)],[origin(2) P1_J(2)],[origin(3) P1_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % P2 from Jacobian
    plot3(P2_J(1),P2_J(2),P2_J(3),'rd','linewidth',2)
    line([P1_J(1) P2_J(1)],[P1_J(2) P2_J(2)],[P1_J(3) P2_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % P3 from Jacobian
    plot3(P3_J(1),P3_J(2),P3_J(3),'rd','linewidth',2)
    line([P2_J(1) P3_J(1)],[P2_J(2) P3_J(2)],[P2_J(3) P3_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    
    % Draw Path
    plot3(Px_J,Py_J,Pz_J,'k:')
    
    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title('3-Link Planar Manipulator')
    set(gca,'fontsize',13)
    axis equal
    axis([-3 3 -3 3 -3 3])
    grid on
    view(az, el)
    pause(0.001)
    hold off
end

