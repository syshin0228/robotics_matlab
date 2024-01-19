% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian of 3DoF RRP Manipulator
syms L1 L2 q1 q2 q3 dq1 dq2 dq3

% Forward Kinematics using DH convention
% DH parameter Ground 
a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
TG0 = DH(q0, d0, alpha0, a0);

% DH parameter Link 1
a1 = 0; alpha1 = pi/2; d1 = L1;
T01 = DH(q1, d1, alpha1, a1);

% DH parameter Link 2
a2 = 0; alpha2 = -pi/2; d2 = 0;
T12 = DH(q2-pi/2, d2, alpha2, a2);

% DH parameter Link 3
a3 = 0; alpha3 = 0; d3 = L2+q3;
T23 = DH(0, q3, alpha3, a3);

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

J1 = [w2ss(z0)*(o3-o0); z0];    % Revolute
J2 = [w2ss(z1)*(o3-o1); z1];    % Revolute
J3 = [z2; [0 0 0].'];            % Prismatic

% Jacobian
J = [J1 J2 J3]

% Calculate time derivative of Jacobian
dJ11 = diff(J(1,1),q1)*dq1 + diff(J(1,1),q2)*dq2 + diff(J(1,1),q3)*dq3;
dJ12 = diff(J(1,2),q1)*dq1 + diff(J(1,2),q2)*dq2 + diff(J(1,2),q3)*dq3;
dJ13 = diff(J(1,3),q1)*dq1 + diff(J(1,3),q2)*dq2 + diff(J(1,3),q3)*dq3;

dJ21 = diff(J(2,1),q1)*dq1 + diff(J(2,1),q2)*dq2 + diff(J(2,1),q3)*dq3;
dJ22 = diff(J(2,2),q1)*dq1 + diff(J(2,2),q2)*dq2 + diff(J(2,2),q3)*dq3;
dJ23 = diff(J(2,3),q1)*dq1 + diff(J(2,3),q2)*dq2 + diff(J(2,3),q3)*dq3;

dJ31 = diff(J(3,1),q1)*dq1 + diff(J(3,1),q2)*dq2 + diff(J(3,1),q3)*dq3;
dJ32 = diff(J(3,2),q1)*dq1 + diff(J(3,2),q2)*dq2 + diff(J(3,2),q3)*dq3;
dJ33 = diff(J(3,3),q1)*dq1 + diff(J(3,3),q2)*dq2 + diff(J(3,3),q3)*dq3;

dJ41 = diff(J(4,1),q1)*dq1 + diff(J(4,1),q2)*dq2 + diff(J(4,1),q3)*dq3;
dJ42 = diff(J(4,2),q1)*dq1 + diff(J(4,2),q2)*dq2 + diff(J(4,2),q3)*dq3;
dJ43 = diff(J(4,3),q1)*dq1 + diff(J(4,3),q2)*dq2 + diff(J(4,3),q3)*dq3;

dJ51 = diff(J(5,1),q1)*dq1 + diff(J(5,1),q2)*dq2 + diff(J(5,1),q3)*dq3;
dJ52 = diff(J(5,2),q1)*dq1 + diff(J(5,2),q2)*dq2 + diff(J(5,2),q3)*dq3;
dJ53 = diff(J(5,3),q1)*dq1 + diff(J(5,3),q2)*dq2 + diff(J(5,3),q3)*dq3;

dJ61 = diff(J(6,1),q1)*dq1 + diff(J(6,1),q2)*dq2 + diff(J(6,1),q3)*dq3;
dJ62 = diff(J(6,2),q1)*dq1 + diff(J(6,2),q2)*dq2 + diff(J(6,2),q3)*dq3;
dJ63 = diff(J(6,3),q1)*dq1 + diff(J(6,3),q2)*dq2 + diff(J(6,3),q3)*dq3;

dJ = [dJ11 dJ12 dJ13; dJ21 dJ22 dJ23; dJ31 dJ32 dJ33; dJ41 dJ42 dJ43; dJ51 dJ52 dJ53; dJ61 dJ62 dJ63]






