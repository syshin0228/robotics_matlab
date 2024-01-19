% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian Analysis of 3DoF RRP Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Camera View
az = 30;
el = 30;

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 30.0;
A2 = 30.0;
A3 = 0.5;
w1 = 2*pi;
w2 = 2*pi;
w3 = 2*pi;
offset1 = 45.0;
offset2 = 0.0;
offset3 = 0.0;

% Time
t = [0:1:Hz*Rep]*dt;

% Joint angles (Deg) and displacement for d3 (m)
th1_deg = A1*sin(w1*t) + offset1;
th2_deg = A2*cos(w2*t) + offset2;
d3_in = A3*sin(w3*t) + offset3;
% Joint angular velocity (Deg/s) and velocity for dd3 (m/s)
dth1_deg = A1*w1*cos(w1*t);
dth2_deg = -A2*w2*sin(w2*t);
dd3_in = A3*w3*cos(w3*t);

% Plot given joint and position (d3) trajectory
figure(1)
subplot(2,2,1)
h_th1 = plot(t,th1_deg,'r','linewidth',1);
hold on
h_th2 = plot(t,th2_deg,'b','linewidth',1);
ylabel('Joint Angle [Deg]')
set(gca,'fontsize',13)
subplot(2,2,2)
h_d3 = plot(t,d3_in,'k','linewidth',1);
hold on
ylabel('Displacement [m]')
set(gca,'fontsize',13)
subplot(2,2,3)
h_dth1 = plot(t,dth1_deg,'r','linewidth',1);
hold on
h_dth2 = plot(t,dth2_deg,'b','linewidth',1);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,2,4)
h_dd3 = plot(t,dd3_in,'k','linewidth',1);
hold on
ylabel('Velocity [m/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward kinematics with Jacobian
P_J = [0 0 0 0 0 0];    % Zero initially
for i = 0:1:Hz*Rep

    i = i+1;
    
    % Sinusoidal Input Joint Angle (Displacement d3)
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    d3 = d3_in(i);
    dth1 = dth1_deg(i) * deg2rad;
    dth2 = dth2_deg(i) * deg2rad;
    dd3 = dd3_in(i);
    
    % Forward Kinematics using DH convention
    % DH parameter Ground 
    a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = 90*deg2rad; d1 = L1; q1 = th1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = -90*deg2rad; d2 = 0; q2 = th2-90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = 0; d3 = L2+d3; q3 = 0;
    T23 = DH(q3, d3, alpha3, a3);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;
    
    % Get position from Forward kinematics
    P(i,:) = T03(1:3,4);
    
    % Jacobian calculation
    z0 = TG0(1:3,3);
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);

    o0 = TG0(1:3,4);
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);

    J1 = [w2ss(z0)*(o3-o0); z0];    % revolute
    J2 = [w2ss(z1)*(o3-o1); z1];    % revolute
    J3 = [z2; [0 0 0]'];            % prismatic
    
    % Jacobian
    J = [J1 J2 J3];
    
    dP(i,:) = J*[dth1 dth2 dd3]';
    P_J = P_J + dP(i,:)*dt;

    Px_J(i) = P_J(1);
    Py_J(i) = P_J(2);
    Pz_J(i) = P_J(3);
    w_J(i,:) = P_J(4:6);
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
h_Pz = plot(t,P(:,3),'b','linewidth',1);
h_Px_J = plot(t,Px_J,'r:','linewidth',2);
h_Py_J = plot(t,Py_J,'g:','linewidth',2);
h_Pz_J = plot(t,Pz_J,'b:','linewidth',2);
legend([h_Px h_Px_J h_Py h_Py_J h_Pz h_Pz_J], 'Px (FK)', 'Px (J)', 'Py (FK)', 'Py (J)', 'Pz (FK)', 'Pz (J)')
ylabel('End-effector Position [m]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Inverse kinematics with Jacobian
% Given initial joint and position (d3) values
th_J = [th1_deg(1) th2_deg(1)] * deg2rad;
d3_J = d3_in(1);
th_J = [th_J d3_J];
for i = 0:1:Hz*Rep
    i = i+1;
    
    % Joint angle and displacement
    th1 = th_J(1);
    th2 = th_J(2);
    d3 = th_J(3);
    
    % Forward Kinematics using DH convention
    % DH parameter Ground 
    a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = 90*deg2rad; d1 = L1; q1 = th1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = -90*deg2rad; d2 = 0; q2 = th2-90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = 0; d3 = L2+d3; q3 = 0;
    T23 = DH(q3, d3, alpha3, a3);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;
    
    % Jacobian calculation
    z0 = TG0(1:3,3);
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);

    o0 = TG0(1:3,4);
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);

    J1 = [w2ss(z0)*(o3-o0); z0];
    J2 = [w2ss(z1)*(o3-o1); z1];
    J3 = [z2; [0 0 0]'];
    
    % Jacobian
    J = [J1 J2 J3];
    
    % Pseudoinverse Jacobian
    dth_J(i,:) = pinv(J)*dP(i,:)';
    
    th_J = th_J + dth_J(i,:)*dt;
    
    th1_J(i) = th_J(1);
    th2_J(i) = th_J(2);
    d3_J(i) = th_J(3);
    
end  
    
dth_J_deg(:,1:2) = dth_J(:,1:2) * rad2deg;
dd3_J = dth_J(:,3);
th1_J_deg = th1_J * rad2deg;
th2_J_deg = th2_J * rad2deg;

% Plot joint and position (d3) trajectory from Inverse kinematics using Jacobian
figure(1)
subplot(2,2,1)
h_th1_J = plot(t,th1_J_deg,'r:','linewidth',2);
hold on
h_th2_J = plot(t,th2_J_deg,'b:','linewidth',2);
ylabel('Joint Angle [Deg]')
legend([h_th1 h_th1_J h_th2 h_th2_J], 'th1 (given)', 'th1 (J)', 'th2 (given)', 'th2 (J)')
set(gca,'fontsize',13)
subplot(2,2,2)
h_d3_J = plot(t,d3_J,'k:','linewidth',2);
ylabel('Displacement [m]')
legend([h_d3 h_d3_J], 'd3 (given)', 'd3 (J)')
set(gca,'fontsize',13)
subplot(2,2,3)
h_dth1_J = plot(t,dth_J_deg(:,1),'r:','linewidth',2);
hold on
h_dth2_J = plot(t,dth_J_deg(:,2),'b:','linewidth',2);
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
legend([h_dth1 h_dth1_J h_dth2 h_dth2_J], 'dth1 (given)', 'dth1 (J)', 'dth2 (given)', 'dth2 (J)')
set(gca,'fontsize',13)
subplot(2,2,4)
h_dd3_J = plot(t,dd3_J,'k:','linewidth',2);
ylabel('Velocity [m/s]')
xlabel('Time [Sec]')
legend([h_dd3 h_dd3_J], 'dd3 (given)', 'dd3 (J)')
set(gca,'fontsize',13)

% Motion Simulation
step = 100;
% Forward Kinematics
for i = 1:step:Hz*Rep
    
    % Sinusoidal Input Joint Angle with given theta
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    d3 = d3_in(i);
    
    % Forward Kinematics using DH convention
    % DH parameter Ground 
    a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = 90*deg2rad; d1 = L1; q1 = th1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = -90*deg2rad; d2 = 0; q2 = th2-90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = 0; d3 = L2+d3; q3 = 0;
    T23 = DH(q3, d3, alpha3, a3);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;

    % Position and axis infomation
    P0 = TG0(1:3,4);
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);
    
    % Sinusoidal Input Joint Angle from Jacobian
    th1_J = th1_J_deg(i) * deg2rad;
    th2_J = th2_J_deg(i) * deg2rad;
    d3_J_ = d3_J(i);
    
    % Forward Kinematics using DH convention
    % DH parameter Ground 
    a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
    TG0_J = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = 90*deg2rad; d1 = L1; q1 = th1_J;
    T01_J = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = -90*deg2rad; d2 = 0; q2 = th2_J-90*deg2rad;
    T12_J = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = 0; d3 = L2+d3_J_; q3 = 0;
    T23_J = DH(q3, d3, alpha3, a3);

    T01_J = TG0_J*T01_J;
    T02_J = T01_J*T12_J;
    T03_J = T02_J*T23_J;

    % Position and axis infomation
    P0_J = TG0_J(1:3,4);
    P1_J = T01_J(1:3,4);
    P2_J = T02_J(1:3,4);
    P3_J = T03_J(1:3,4);


    figure(100)
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 2)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color', 'g', 'linewidth', 2)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color', 'b', 'linewidth', 2)

    % P0 from given theta
    plot3(P0(1),P0(2),P0(3),'ro','linewidth',2)
    line([origin(1) P0(1)],[origin(2) P0(2)],[origin(3) P0(3)], 'color', 'k', 'linewidth', 2)
    % P1 from given theta
    plot3(P1(1),P1(2),P1(3),'ko','linewidth',2)
    line([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)], 'color', 'k', 'linewidth', 2)
    % P2 from given theta
    plot3(P2(1),P2(2),P2(3),'ko','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
    % P3 from given theta
    plot3(P3(1),P3(2),P3(3),'ko','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)

    % P0 from Jacobian
    plot3(P0_J(1),P0_J(2),P0_J(3),'rd','linewidth',2)
    line([origin(1) P0_J(1)],[origin(2) P0_J(2)],[origin(3) P0_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % P1 from given theta
    plot3(P1_J(1),P1_J(2),P1_J(3),'rd','linewidth',2)
    line([P0_J(1) P1_J(1)],[P0_J(2) P1_J(2)],[P0_J(3) P1_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % P2 from Jacobian
    plot3(P2_J(1),P2_J(2),P2_J(3),'rd','linewidth',2)
    line([P1_J(1) P2_J(1)],[P1_J(2) P2_J(2)],[P1_J(3) P2_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % P3 from Jacobian
    plot3(P3_J(1),P3_J(2),P3_J(3),'rd','linewidth',2)
    line([P2_J(1) P3_J(1)],[P2_J(2) P3_J(2)],[P2_J(3) P3_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')

    % Draw Path
    plot3(Px_J, Py_J, Pz_J,'k:','linewidth',1)
    
    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title('3DoF RRP Manipulator')
    set(gca,'fontsize',13)
    axis equal
    axis([-2 2 -2 2 0 3])
    grid on
    view(az, el)
    pause(0.001)
    hold off
    
end
