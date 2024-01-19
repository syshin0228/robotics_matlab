% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian of 3-Link Planar Manipulator
syms L1 L2 L3 q1 q2 q3 dq1 dq2 dq3

% Forward Kinematics using DH convention
% DH parameter Link 1
a1 = L1; alpha1 = 0; d1 = 0;
T01 = DH(q1, d1, alpha1, a1);

% DH parameter Link 2
a2 = L2; alpha2 = 0; d2 = 0;
T12 = DH(q2, d2, alpha2, a2);

% DH parameter Link 3
a3 = L3; alpha3 = 0; d3 = 0;
T23 = DH(q3, d3, alpha3, a3);

T02 = T01*T12;
T03 = T02*T23;

% Jacobian calculation
z0 = [0 0 1].';
z1 = T01(1:3,3);
z2 = T02(1:3,3);

o0 = [0 0 0].';
o1 = T01(1:3,4);
o2 = T02(1:3,4);
o3 = T03(1:3,4);

J1 = [w2ss(z0)*(o3-o0); z0];
J2 = [w2ss(z1)*(o3-o1); z1];
J3 = [w2ss(z2)*(o3-o2); z2];

% Jacobian
J = [J1 J2 J3]

% Calculate time derivative of Jacobian
dJ11 = diff(J(1,1),q1)*dq1 + diff(J(1,1),q2)*dq2 + diff(J(1,1),q3)*dq3;
dJ12 = diff(J(1,2),q1)*dq1 + diff(J(1,2),q2)*dq2 + diff(J(1,2),q3)*dq3;
dJ13 = diff(J(1,3),q1)*dq1 + diff(J(1,3),q2)*dq2 + diff(J(1,3),q3)*dq3;
dJ21 = diff(J(2,1),q1)*dq1 + diff(J(2,1),q2)*dq2 + diff(J(2,1),q3)*dq3;
dJ22 = diff(J(2,2),q1)*dq1 + diff(J(2,2),q2)*dq2 + diff(J(2,2),q3)*dq3;
dJ23 = diff(J(2,3),q1)*dq1 + diff(J(2,3),q2)*dq2 + diff(J(2,3),q3)*dq3;

dJ = [dJ11 dJ12 dJ13; dJ21 dJ22 dJ23; 0 0 0]

