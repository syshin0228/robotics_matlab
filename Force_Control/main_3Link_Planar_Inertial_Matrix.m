% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

tic

% Euler-Lagrange Equations of Motion of 3 Link Planar Manipulator
% Generalized form
% Parameterization
syms m1 m2 m3 I1 I2 I3 L1 L2 L3 L1c L2c L3c g;      % Mass, Moment of Inertia, Link lengths, Gravity acc
syms q1 dq1 ddq1 q2 dq2 ddq2 q3 dq3 ddq3;           % Generalized coordinates
syms tau1 tau2 tau3;                                % Generalized force (input torques)
G = [0 g 0].';

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

% DH parameter Link 1 CoM
a1 = L1c; alpha1 = 0; d1 = 0;
T01c = DH(q1, d1, alpha1, a1);

% DH parameter Link 2 CoM
a2 = L2c; alpha2 = 0; d2 = 0;
T12c = DH(q2, d2, alpha2, a2);

% DH parameter Link 3 CoM
a3 = L3c; alpha3 = 0; d3 = 0;
T23c = DH(q3, d3, alpha3, a3);

% End points
T01 = T01;
T02 = T01*T12;
T03 = T02*T23;

% CoM
T01c = T01c;
T02c = T01*T12c;
T03c = T01*T12*T23c;

% Jacobian calculation
z0 = [0 0 1]';
z1 = T01(1:3,3);
z2 = T02(1:3,3);

o0 = [0 0 0]';
o1 = T01(1:3,4);
o2 = T02(1:3,4);
o3 = T03(1:3,4);
o1c = T01c(1:3,4);
o2c = T02c(1:3,4);
o3c = T03c(1:3,4);

% Jacobian of Link 1 CoM
J11c = [w2ss(z0)*(o1c-o0); z0];

% Jacobian of Link 2 CoM
J12c = [w2ss(z0)*(o2c-o0); z0];
J22c = [w2ss(z1)*(o2c-o1); z1];

% Jacobian of Link 3 CoM
J13c = [w2ss(z0)*(o3c-o0); z0];
J23c = [w2ss(z1)*(o3c-o1); z1];
J33c = [w2ss(z2)*(o3c-o2); z2];

% Jacobians
J1 = [J11c zeros(6,1) zeros(6,1)];
J2 = [J12c J22c zeros(6,1)];
J3 = [J13c J23c J33c];

dq = [dq1 dq2 dq3].';
% Kinetic Energy
M_T = (m1*J1(1:3,:).'*J1(1:3,:) + m2*J2(1:3,:).'*J2(1:3,:) + m3*J3(1:3,:).'*J3(1:3,:)); 
M_R = (J1(4:6,:).'*I1*J1(4:6,:) + J2(4:6,:).'*I2*J2(4:6,:) + J3(4:6,:).'*I3*J3(4:6,:)); 
M = M_T + M_R;

% Inertia Matrix
M1 = M(1,:)
M2 = M(2,:)
M3 = M(3,:)
