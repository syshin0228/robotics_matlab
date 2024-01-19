% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% DH Parameter
syms q d alpha a
Rz = [cos(q) -sin(q) 0 0; sin(q) cos(q) 0 0; 0 0 1 0; 0 0 0 1]
Td = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1]
Ta = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1]
Ra = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1]

A = Rz*Td*Ta*Ra

% Forward Kinematics of 7DoF Arm Manipulator using DH Parameter
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

% Input Joint Angle
th1 = 0.0 * deg2rad;
th2 = 0.0 * deg2rad;
th3 = 0.0 * deg2rad;
th4 = 0.0 * deg2rad;
th5 = 0.0 * deg2rad;
th6 = 0.0 * deg2rad;
th7 = 0.0 * deg2rad;

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

% Position and axis infomation
axis_scale = 0.3;
P0 = TG0(1:3,4);
P0_z = P0 + axis_scale * TG0(1:3,3);
P1 = T01(1:3,4);
P1_z = P1 + axis_scale * T01(1:3,3);
P2 = T02(1:3,4);
P2_z = P2 + axis_scale * T02(1:3,3);
P3 = T03(1:3,4);
P3_z = P3 + axis_scale * T03(1:3,3);
P4 = T04(1:3,4);
P4_z = P4 + axis_scale * T04(1:3,3);
P5 = T05(1:3,4);
P5_z = P5 + axis_scale * T05(1:3,3);
P6 = T06(1:3,4);
P6_z = P6 + axis_scale * T06(1:3,3);
P7 = T07(1:3,4);
P7_x = P7 + axis_scale * T07(1:3,1);
P7_y = P7 + axis_scale * T07(1:3,2);
P7_z = P7 + axis_scale * T07(1:3,3);

figure(1)
% Grobal Frame (Origin)
plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 2)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 2)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 2)

% Joint 1 
plot3(P0(1),P0(2),P0(3),'ro','linewidth',3)
line([origin(1) P0(1)],[origin(3) P0(3)],[origin(3) P0(3)], 'color', 'k', 'linewidth', 3)
line([origin(1) P0_z(1)],[origin(2) P0_z(2)],[origin(3) P0_z(3)], 'color', 'g', 'linewidth', 3)

% Joint 2 
plot3(P1(1),P1(2),P1(3),'bo','linewidth',3)
line([P0(1) P1(1)],[P0(2) P1(2)],[P0(3) P1(3)], 'color', 'k', 'linewidth', 2)
line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color', 'r', 'linewidth', 3)

% Joint 3 
plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color', 'b', 'linewidth', 3)

% Joint 4 
plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color', 'g', 'linewidth', 3)

% Joint 5
plot3(P4(1),P4(2),P4(3),'ro','linewidth',2)
line([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)], 'color', 'k', 'linewidth', 2)
line([P4(1) P4_z(1)],[P4(2) P4_z(2)],[P4(3) P4_z(3)], 'color', 'b', 'linewidth', 3)

% Joint 6
plot3(P5(1),P5(2),P5(3),'ro','linewidth',2)
line([P4(1) P5(1)],[P4(2) P5(2)],[P4(3) P5(3)], 'color', 'k', 'linewidth', 2)
line([P5(1) P5_z(1)],[P5(2) P5_z(2)],[P5(3) P5_z(3)], 'color', 'g', 'linewidth', 3)

% Joint 7
plot3(P6(1),P6(2),P6(3),'ro','linewidth',2)
line([P5(1) P6(1)],[P5(2) P6(2)],[P5(3) P6(3)], 'color', 'k', 'linewidth', 2)
line([P6(1) P6_z(1)],[P6(2) P6_z(2)],[P6(3) P6_z(3)], 'color', 'r', 'linewidth', 3)

% End Effector
plot3(P7(1),P7(2),P7(3),'ro','linewidth',2)
line([P6(1) P7(1)],[P6(2) P7(2)],[P6(3) P7(3)], 'color', 'k', 'linewidth', 2)
line([P7(1) P7_x(1)],[P7(2) P7_x(2)],[P7(3) P7_x(3)], 'color', 'r', 'linewidth', 3)
line([P7(1) P7_y(1)],[P7(2) P7_y(2)],[P7(3) P7_y(3)], 'color', 'g', 'linewidth', 3)
line([P7(1) P7_z(1)],[P7(2) P7_z(2)],[P7(3) P7_z(3)], 'color', 'b', 'linewidth', 3)

xlabel('x- axis','fontsize',13)
ylabel('y- axis','fontsize',13)
zlabel('z- axis','fontsize',13)
title('7DoF Arm Manipulator')
set(gca,'fontsize',13)
axis equal
grid on
view(az, el)






