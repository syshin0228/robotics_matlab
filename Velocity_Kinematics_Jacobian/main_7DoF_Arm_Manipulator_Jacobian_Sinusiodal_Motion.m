% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian Analysis of 7DoF Arm Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Camera View
az = 120;
el = 30;

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Constant link lengths
L0 = 0.0;
L1 = 1.0;
L2 = 1.0;
L3 = 0.4;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 20.0;
A2 = 15.0;
A3 = 20.0;
A4 = -30.0;
A5 = 10.0;
A6 = 30.0;
A7 = 20.0;
w1 = 2*pi;
w2 = 2*pi;
w3 = 2*pi;
w4 = 2*pi;
w5 = 2*pi;
w6 = 2*pi;
w7 = 2*pi;
offset1 = 30.0;
offset2 = -30.0;
offset3 = 30.0;
offset4 = -45.0;
offset5 = -15.0;
offset6 = -30.0;
offset7 = 30.0;
% Time
t = [0:1:Hz*Rep]*dt;

% Joint angles (Deg)
th1_deg = A1*sin(w1*t) + offset1;
th2_deg = A2*cos(w2*t) + offset2;
th3_deg = A3*sin(w3*t) + offset3;
th4_deg = A4*cos(w4*t) + offset4;
th5_deg = A5*sin(w5*t) + offset5;
th6_deg = A6*cos(w6*t) + offset6;
th7_deg = A7*sin(w7*t) + offset7;
% Joint angular velocity (Deg/s)
dth1_deg = A1*w1*cos(w1*t);
dth2_deg = -A2*w2*sin(w2*t);
dth3_deg = A3*w3*cos(w3*t);
dth4_deg = -A4*w4*sin(w4*t);
dth5_deg = A5*w5*cos(w5*t);
dth6_deg = -A6*w6*sin(w6*t);
dth7_deg = A7*w7*cos(w7*t);

% Plot given joint trajectory
figure(1)
set(gcf,'position',[0 10 1400 600])
subplot(2,4,1)
plot(t,th1_deg,'k','linewidth',1);
hold on
ylabel('J1 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,2)
plot(t,th2_deg,'k','linewidth',1);
hold on
ylabel('J2 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,3)
plot(t,th3_deg,'k','linewidth',1);
hold on
ylabel('J3 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,4)
plot(t,th4_deg,'k','linewidth',1);
hold on
ylabel('J4 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,5)
plot(t,th5_deg,'k','linewidth',1);
hold on
ylabel('J5 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,6)
plot(t,th6_deg,'k','linewidth',1);
hold on
ylabel('J6 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,7)
h_th = plot(t,th7_deg,'k','linewidth',1);
hold on
ylabel('J7 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
set(gcf,'position',[0 10 1400 600])
subplot(2,4,1)
plot(t,dth1_deg,'k','linewidth',1);
hold on
ylabel('J1 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,2)
plot(t,dth2_deg,'k','linewidth',1);
hold on
ylabel('J2 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,3)
plot(t,dth3_deg,'k','linewidth',1);
hold on
ylabel('J3 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,4)
plot(t,dth4_deg,'k','linewidth',1);
hold on
ylabel('J4 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,5)
plot(t,dth5_deg,'k','linewidth',1);
hold on
ylabel('J5 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,6)
plot(t,dth6_deg,'k','linewidth',1);
hold on
ylabel('J6 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,7)
h_dth = plot(t,dth7_deg,'k','linewidth',1);
hold on
ylabel('J7 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Forward kinematics with Jacobian
P_J = [0 0 0 0 0 0];    % Zero initially
% Forward Kinematics
for i = 0:1:Hz*Rep
    
    i = i + 1;
    
    % Sinusoidal Input Joint Angle
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    th3 = th3_deg(i) * deg2rad;
    th4 = th4_deg(i) * deg2rad;
    th5 = th5_deg(i) * deg2rad;
    th6 = th6_deg(i) * deg2rad;
    th7 = th7_deg(i) * deg2rad;
    dth1 = dth1_deg(i) * deg2rad;
    dth2 = dth2_deg(i) * deg2rad;
    dth3 = dth3_deg(i) * deg2rad;
    dth4 = dth4_deg(i) * deg2rad;
    dth5 = dth5_deg(i) * deg2rad;
    dth6 = dth6_deg(i) * deg2rad;
    dth7 = dth7_deg(i) * deg2rad;

    % Forward Kinematics using DH convention
    % DH parameter Ground
    a0 = 0; alpha0 = -90*deg2rad; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = -90*deg2rad; d1 = L0; q1 = th1-90*deg2rad;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = 90*deg2rad; d2 = 0; q2 = th2+90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = -90*deg2rad; d3 = -L1; q3 = th3+90*deg2rad;
    T23 = DH(q3, d3, alpha3, a3);

    % DH parameter Link 4
    a4 = 0; alpha4 = 90*deg2rad; d4 = 0; q4 = th4;
    T34 = DH(q4, d4, alpha4, a4);

    % DH parameter Link 5
    a5 = 0; alpha5 = -90*deg2rad; d5 = -L2; q5 = th5;
    T45 = DH(q5, d5, alpha5, a5);

    % DH parameter Link 6
    a6 = 0; alpha6 = -90*deg2rad; d6 = 0; q6 = th6-90*deg2rad;
    T56 = DH(q6, d6, alpha6, a6);

    % DH parameter Link 7
    a7 = -L3; alpha7 = 0; d7 = 0; q7 = th7;
    T67 = DH(q7, d7, alpha7, a7);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T06 = T05*T56;
    T07 = T06*T67;
    
    % Get position from Forward kinematics
    P(i,:) = T07(1:3,4);
    
    % Jacobian calculation
    z0 = TG0(1:3,3);
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);
    z3 = T03(1:3,3);
    z4 = T04(1:3,3);
    z5 = T05(1:3,3);
    z6 = T06(1:3,3);

    o0 = TG0(1:3,4);
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);
    o4 = T04(1:3,4);
    o5 = T05(1:3,4);
    o6 = T06(1:3,4);
    o7 = T07(1:3,4);

    J1 = [w2ss(z0)*(o7-o0); z0];
    J2 = [w2ss(z1)*(o7-o1); z1];
    J3 = [w2ss(z2)*(o7-o2); z2];
    J4 = [w2ss(z3)*(o7-o3); z3];
    J5 = [w2ss(z4)*(o7-o4); z4];
    J6 = [w2ss(z5)*(o7-o5); z5];
    J7 = [w2ss(z6)*(o7-o6); z6];
    
    % Jacobian
    J = [J1 J2 J3 J4 J5 J6 J7];
    
    dP(i,:) = J*[dth1 dth2 dth3 dth4 dth5 dth6 dth7]';
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
figure(3)
h_Px = plot(t,P(:,1),'r','linewidth',1);
hold on
h_Py = plot(t,P(:,2),'g','linewidth',1);
h_Pz = plot(t,P(:,3),'b','linewidth',1);
h_Px_J = plot(t,Px_J,'r:','linewidth',2);
h_Py_J = plot(t,Py_J,'g:','linewidth',2);
h_Pz_J = plot(t,Pz_J,'b:','linewidth',2);
ylabel('End-effector Position [m]')
xlabel('Time [Sec]')
legend([h_Px h_Px_J h_Py h_Py_J h_Pz h_Pz_J], 'Px (FK)', 'Px (J)', 'Py (FK)', 'Py (J)', 'Pz (FK)', 'Pz (J)')
set(gca,'fontsize',13)

% Inverse kinematics with Jacobian
% Given initial joint values
th_J = [th1_deg(1) th2_deg(1) th3_deg(1) th4_deg(1) th5_deg(1) th6_deg(1) th7_deg(1)] * deg2rad;
for i = 0:1:Hz*Rep
    
    i = i+1;
    
    % Joint Angle
    th1 = th_J(1);
    th2 = th_J(2);
    th3 = th_J(3);
    th4 = th_J(4);
    th5 = th_J(5);
    th6 = th_J(6);
    th7 = th_J(7);

    % Forward Kinematics using DH convention
    % DH parameter Ground
    a0 = 0; alpha0 = -90*deg2rad; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = -90*deg2rad; d1 = L0; q1 = th1-90*deg2rad;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = 90*deg2rad; d2 = 0; q2 = th2+90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = -90*deg2rad; d3 = -L1; q3 = th3+90*deg2rad;
    T23 = DH(q3, d3, alpha3, a3);

    % DH parameter Link 4
    a4 = 0; alpha4 = 90*deg2rad; d4 = 0; q4 = th4;
    T34 = DH(q4, d4, alpha4, a4);

    % DH parameter Link 5
    a5 = 0; alpha5 = -90*deg2rad; d5 = -L2; q5 = th5;
    T45 = DH(q5, d5, alpha5, a5);

    % DH parameter Link 6
    a6 = 0; alpha6 = -90*deg2rad; d6 = 0; q6 = th6-90*deg2rad;
    T56 = DH(q6, d6, alpha6, a6);

    % DH parameter Link 7
    a7 = -L3; alpha7 = 0; d7 = 0; q7 = th7;
    T67 = DH(q7, d7, alpha7, a7);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T06 = T05*T56;
    T07 = T06*T67;
    
    % Jacobian calculation
    z0 = TG0(1:3,3);
    z1 = T01(1:3,3);
    z2 = T02(1:3,3);
    z3 = T03(1:3,3);
    z4 = T04(1:3,3);
    z5 = T05(1:3,3);
    z6 = T06(1:3,3);

    o0 = TG0(1:3,4);
    o1 = T01(1:3,4);
    o2 = T02(1:3,4);
    o3 = T03(1:3,4);
    o4 = T04(1:3,4);
    o5 = T05(1:3,4);
    o6 = T06(1:3,4);
    o7 = T07(1:3,4);

    J1 = [w2ss(z0)*(o7-o0); z0];
    J2 = [w2ss(z1)*(o7-o1); z1];
    J3 = [w2ss(z2)*(o7-o2); z2];
    J4 = [w2ss(z3)*(o7-o3); z3];
    J5 = [w2ss(z4)*(o7-o4); z4];
    J6 = [w2ss(z5)*(o7-o5); z5];
    J7 = [w2ss(z6)*(o7-o6); z6];
    
    % Jacobian
    J = [J1 J2 J3 J4 J5 J6 J7];
    
    % Pseudoinverse Jacobian (Redundant system)
    % Solution 1
    % Solution with b = 0 from (I-pinv(J)*J)*b = 0
     dth_J(i,:) = pinv(J)*dP(i,:)';
     
%     % Solution 2
%     % Solution with b = arbitrary, (I-pinv(J)*J)*b ~= 0
%     b = [1 1 1 1 1 1 1]'*10.0;
%     dth_J(i,:) = pinv(J)*dP(i,:)' + (eye(7,7) - pinv(J)*J)*b;

%     % Solution 3
%     % Solution with b = th_subtask, (I-pinv(J)*J)*b ~= 0, th_subtask = planned motion (given)
%     b = [dth1_deg(i) dth2_deg(i) dth3_deg(i) dth4_deg(i) dth5_deg(i) dth6_deg(i) dth7_deg(i)]'*deg2rad;
%     dth_J(i,:) = pinv(J)*dP(i,:)' + (eye(7,7) - pinv(J)*J)*b;

    th_J = th_J + dth_J(i,:)*dt;
    
    th1_J(i) = th_J(1);
    th2_J(i) = th_J(2);
    th3_J(i) = th_J(3);
    th4_J(i) = th_J(4);
    th5_J(i) = th_J(5);
    th6_J(i) = th_J(6);
    th7_J(i) = th_J(7);
    
end

dth_J_deg = dth_J * rad2deg;
th1_J_deg = th1_J * rad2deg;
th2_J_deg = th2_J * rad2deg;
th3_J_deg = th3_J * rad2deg;
th4_J_deg = th4_J * rad2deg;
th5_J_deg = th5_J * rad2deg;
th6_J_deg = th6_J * rad2deg;
th7_J_deg = th7_J * rad2deg;

% Plot joint trajectory from Inverse kinematics using Jacobian
% Plot given joint trajectory
figure(1)
set(gcf,'position',[0 10 1400 600])
subplot(2,4,1)
plot(t,th1_J_deg,'k:','linewidth',2);
hold on
ylabel('J1 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,2)
plot(t,th2_J_deg,'k:','linewidth',2);
hold on
ylabel('J2 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,3)
plot(t,th3_J_deg,'k:','linewidth',2);
hold on
ylabel('J3 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,4)
plot(t,th4_J_deg,'k:','linewidth',2);
hold on
ylabel('J4 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,5)
plot(t,th5_J_deg,'k:','linewidth',2);
hold on
ylabel('J5 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,6)
plot(t,th6_J_deg,'k:','linewidth',2);
hold on
ylabel('J6 Angle [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,7)
h_th_J = plot(t,th7_J_deg,'k:','linewidth',2);
hold on
legend([])
ylabel('J7 Angle [Deg]')
xlabel('Time [Sec]')
legend([h_th h_th_J],'given','Jacobian')
set(gca,'fontsize',13)

figure(2)
set(gcf,'position',[0 10 1400 600])
subplot(2,4,1)
plot(t,dth_J_deg(:,1),'k:','linewidth',2);
hold on
ylabel('J1 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,2)
plot(t,dth_J_deg(:,2),'k:','linewidth',2);
hold on
ylabel('J2 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,3)
plot(t,dth_J_deg(:,3),'k:','linewidth',2);
hold on
ylabel('J3 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,4)
plot(t,dth_J_deg(:,4),'k:','linewidth',2);
hold on
ylabel('J4 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,5)
plot(t,dth_J_deg(:,5),'k:','linewidth',2);
hold on
ylabel('J5 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,6)
plot(t,dth_J_deg(:,6),'k:','linewidth',2);
hold on
ylabel('J6 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)
subplot(2,4,7)
h_dth_J = plot(t,dth_J_deg(:,7),'k:','linewidth',2);
hold on
ylabel('J7 Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
legend([h_dth h_dth_J],'given','Jacobian')
set(gca,'fontsize',13)

% Motion Simulation
step = 100;
% Forward Kinematics
for i = 1:step:Hz*Rep
    
    t = i*dt;
    
    % Sinusoidal Input Joint Angle with given theta
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;
    th3 = th3_deg(i) * deg2rad;
    th4 = th4_deg(i) * deg2rad;
    th5 = th5_deg(i) * deg2rad;
    th6 = th6_deg(i) * deg2rad;
    th7 = th7_deg(i) * deg2rad;

    % Forward Kinematics using DH convention
    % DH parameter Ground
    a0 = 0; alpha0 = -90*deg2rad; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = -90*deg2rad; d1 = L0; q1 = th1-90*deg2rad;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = 90*deg2rad; d2 = 0; q2 = th2+90*deg2rad;
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = -90*deg2rad; d3 = -L1; q3 = th3+90*deg2rad;
    T23 = DH(q3, d3, alpha3, a3);

    % DH parameter Link 4
    a4 = 0; alpha4 = 90*deg2rad; d4 = 0; q4 = th4;
    T34 = DH(q4, d4, alpha4, a4);

    % DH parameter Link 5
    a5 = 0; alpha5 = -90*deg2rad; d5 = -L2; q5 = th5;
    T45 = DH(q5, d5, alpha5, a5);

    % DH parameter Link 6
    a6 = 0; alpha6 = -90*deg2rad; d6 = 0; q6 = th6-90*deg2rad;
    T56 = DH(q6, d6, alpha6, a6);

    % DH parameter Link 7
    a7 = -L3; alpha7 = 0; d7 = 0; q7 = th7;
    T67 = DH(q7, d7, alpha7, a7);

    T01 = TG0*T01;
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T06 = T05*T56;
    T07 = T06*T67;
    
    % Position infomation
    P0 = TG0(1:3,4);
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);
    P4 = T04(1:3,4);
    P5 = T05(1:3,4);
    P6 = T06(1:3,4);
    P7 = T07(1:3,4);

    
    % Sinusoidal Input Joint Angle from Jacobian
    th1_J = th1_J_deg(i) * deg2rad;
    th2_J = th2_J_deg(i) * deg2rad;
    th3_J = th3_J_deg(i) * deg2rad;
    th4_J = th4_J_deg(i) * deg2rad;
    th5_J = th5_J_deg(i) * deg2rad;
    th6_J = th6_J_deg(i) * deg2rad;
    th7_J = th7_J_deg(i) * deg2rad;

    % Forward Kinematics using DH convention
    % DH parameter Ground
    a0 = 0; alpha0 = -90*deg2rad; d0 = 0; q0 = 0;
    TG0 = DH(q0, d0, alpha0, a0);

    % DH parameter Link 1
    a1 = 0; alpha1 = -90*deg2rad; d1 = L0; q1 = th1_J-90*deg2rad;
    T01_J = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = 0; alpha2 = 90*deg2rad; d2 = 0; q2 = th2_J+90*deg2rad;
    T12_J = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = 0; alpha3 = -90*deg2rad; d3 = -L1; q3 = th3_J+90*deg2rad;
    T23_J = DH(q3, d3, alpha3, a3);

    % DH parameter Link 4
    a4 = 0; alpha4 = 90*deg2rad; d4 = 0; q4 = th4_J;
    T34_J = DH(q4, d4, alpha4, a4);

    % DH parameter Link 5
    a5 = 0; alpha5 = -90*deg2rad; d5 = -L2; q5 = th5_J;
    T45_J = DH(q5, d5, alpha5, a5);

    % DH parameter Link 6
    a6 = 0; alpha6 = -90*deg2rad; d6 = 0; q6 = th6_J-90*deg2rad;
    T56_J = DH(q6, d6, alpha6, a6);

    % DH parameter Link 7
    a7 = -L3; alpha7 = 0; d7 = 0; q7 = th7_J;
    T67_J = DH(q7, d7, alpha7, a7);

    T01_J = TG0*T01_J;
    T02_J = T01_J*T12_J;
    T03_J = T02_J*T23_J;
    T04_J = T03_J*T34_J;
    T05_J = T04_J*T45_J;
    T06_J = T05_J*T56_J;
    T07_J = T06_J*T67_J;
    
    % Position infomation
    P0_J = TG0(1:3,4);
    P1_J = T01_J(1:3,4);
    P2_J = T02_J(1:3,4);
    P3_J = T03_J(1:3,4);
    P4_J = T04_J(1:3,4);
    P5_J = T05_J(1:3,4);
    P6_J = T06_J(1:3,4);
    P7_J = T07_J(1:3,4);

    
    
    figure(100)
    % Grobal Frame (Origin)
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 1)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 1)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 1)

    % Plot position from given theta
    % Joint 1 
    plot3(P0(1),P0(2),P0(3),'ko','linewidth',3)
    line([origin(1) P0(1)],[origin(3) P0(3)],[origin(3) P0(3)], 'color', 'k', 'linewidth', 3)
    % Joint 2 
    plot3(P1(1),P1(2),P1(3),'ko','linewidth',3)
    line([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)], 'color', 'k', 'linewidth', 2)
    % Joint 3 
    plot3(P2(1),P2(2),P2(3),'ko','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
    % Joint 4 
    plot3(P3(1),P3(2),P3(3),'ko','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
    % Joint 5
    plot3(P4(1),P4(2),P4(3),'ko','linewidth',2)
    line([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)], 'color', 'k', 'linewidth', 2)
    % Joint 6
    plot3(P5(1),P5(2),P5(3),'ko','linewidth',2)
    line([P4(1) P5(1)],[P4(2) P5(2)],[P4(3) P5(3)], 'color', 'k', 'linewidth', 2)
    % Joint 7
    plot3(P6(1),P6(2),P6(3),'ko','linewidth',2)
    line([P5(1) P6(1)],[P5(2) P6(2)],[P5(3) P6(3)], 'color', 'k', 'linewidth', 2)
    % End Effector
    plot3(P7(1),P7(2),P7(3),'ko','linewidth',2)
    line([P6(1) P7(1)],[P6(2) P7(2)],[P6(3) P7(3)], 'color', 'k', 'linewidth', 2)

    
    % Plot position from Jacobian
    % Joint 1 
    plot3(P0_J(1),P0_J(2),P0_J(3),'rd','linewidth',3)
    line([origin(1) P0_J(1)],[origin(3) P0_J(3)],[origin(3) P0_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 2 
    plot3(P1_J(1),P1_J(2),P1_J(3),'rd','linewidth',3)
    line([P0_J(1) P1_J(1)],[P0_J(2) P1_J(2)],[P0_J(3) P1_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 3 
    plot3(P2_J(1),P2_J(2),P2_J(3),'rd','linewidth',2)
    line([P1_J(1) P2_J(1)],[P1_J(2) P2_J(2)],[P1_J(3) P2_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 4 
    plot3(P3_J(1),P3_J(2),P3_J(3),'rd','linewidth',2)
    line([P2_J(1) P3_J(1)],[P2_J(2) P3_J(2)],[P2_J(3) P3_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 5
    plot3(P4_J(1),P4_J(2),P4_J(3),'rd','linewidth',2)
    line([P3_J(1) P4_J(1)],[P3_J(2) P4_J(2)],[P3_J(3) P4_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 6
    plot3(P5_J(1),P5_J(2),P5_J(3),'rd','linewidth',2)
    line([P4_J(1) P5_J(1)],[P4_J(2) P5_J(2)],[P4_J(3) P5_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % Joint 7
    plot3(P6_J(1),P6_J(2),P6_J(3),'rd','linewidth',2)
    line([P5_J(1) P6_J(1)],[P5_J(2) P6_J(2)],[P5_J(3) P6_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    % End Effector
    plot3(P7_J(1),P7_J(2),P7_J(3),'rd','linewidth',2)
    line([P6_J(1) P7_J(1)],[P6_J(2) P7_J(2)],[P6_J(3) P7_J(3)], 'color', 'r', 'linewidth', 3, 'linestyle',':')
    
    % Draw Path
    plot3(Px_J,Py_J,Pz_J,'k:','linewidth',1)

    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title('7DoF Arm Manipulator')
    set(gca,'fontsize',13)
    axis equal
    grid on
    view(az, el)
    pause(0.001)
    hold off
    
end


