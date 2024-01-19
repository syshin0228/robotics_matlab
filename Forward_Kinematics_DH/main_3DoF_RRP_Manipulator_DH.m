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

% Forward Kinematics of 3DoF RRP Manipulator using DH Parameter
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

% Input Joint Angle
th1 = 0.0 * deg2rad;
th2 = 0.0 * deg2rad;
d3 = 0.5; 

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
axis_scale = 0.3;
P0 = TG0(1:3,4);
P0_z = P0 + axis_scale * TG0(1:3,3);

P1 = T01(1:3,4);
P1_z = P1 + axis_scale * T01(1:3,3); 

P2 = T02(1:3,4);
P2_z = P2 + axis_scale * T02(1:3,3);

P3 = T03(1:3,4);
P3_x = P3 + axis_scale * T03(1:3,1);
P3_y = P3 + axis_scale * T03(1:3,2);
P3_z = P3 + axis_scale * T03(1:3,3);

figure(1)
% Draw global coordinate
plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 2)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color', 'g', 'linewidth', 2)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color', 'b', 'linewidth', 2)

plot3(P0(1),P0(2),P0(3),'ro','linewidth',2)
line([origin(1) P0(1)],[origin(2) P0(2)],[origin(3) P0(3)], 'color', 'k', 'linewidth', 2)
line([P0(1) P0_z(1)],[P0(2) P0_z(2)],[P0(3) P0_z(3)], 'color', 'b', 'linewidth', 3)

plot3(P1(1),P1(2),P1(3),'ro','linewidth',2)
line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)
line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color', 'g', 'linewidth', 3)

plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color', 'r', 'linewidth', 3)

plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
line([P3(1) P3_x(1)],[P3(2) P3_x(2)],[P3(3) P3_x(3)], 'color', 'r', 'linewidth', 3)
line([P3(1) P3_y(1)],[P3(2) P3_y(2)],[P3(3) P3_y(3)], 'color', 'g', 'linewidth', 3)
line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color', 'b', 'linewidth', 3)

xlabel('x- axis','fontsize',13)
ylabel('y- axis','fontsize',13)
zlabel('z- axis','fontsize',13)
title('3DoF RRP Manipulator')
set(gca,'fontsize',13)
axis equal
grid on
view(az, el)


