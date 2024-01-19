% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobian of 3DoF RRP Manipulator
syms L1 L2 th1 th2 d3 dth1 dth2 dd3

% Forward Kinematics using DH convention
% DH parameter Ground 
a0 = 0; alpha0 = 0; d0 = 0; q0 = 0;
TG0 = DH(q0, d0, alpha0, a0);

% DH parameter Link 1
a1 = 0; alpha1 = pi/2; d1 = L1; q1 = th1;
T01 = DH(q1, d1, alpha1, a1);

% DH parameter Link 2
a2 = 0; alpha2 = -pi/2; d2 = 0; q2 = th2-pi/2;
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

J1 = [w2ss(z0)*(o3-o0); z0];    % Revolute
J2 = [w2ss(z1)*(o3-o1); z1];    % Revolute
J3 = [z2; [0 0 0]'];            % Prismatic

% Jacobian
J = [J1 J2 J3]

dP = J*[dth1 dth2 dd3]'


