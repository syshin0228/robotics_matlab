% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Analytically solving for Jacobian of 2-Link Planar Manipulator
syms L1 L2 th1 th2 dth1 dth2

% Forward Kinematics
P2(1,:) = L1*cos(th1) + L2*cos(th1+th2);
P2(2,:) = L1*sin(th1) + L2*sin(th1+th2);

% Analytical Jacobian
Ja = [-L1*sin(th1)-L2*sin(th1+th2) -L2*sin(th1+th2); L1*cos(th1)+L2*cos(th1+th2) L2*cos(th1+th2)]
dP2 = Ja*[dth1 dth2]'



% Forward Kinematics using DH convention
% DH parameter Link 1
a1 = L1; alpha1 = 0; d1 = 0; q1 = th1;
T01 = DH(q1, d1, alpha1, a1);

% DH parameter Link 2
a2 = L2; alpha2 = 0; d2 = 0; q2 = th2;
T12 = DH(q2, d2, alpha2, a2);

T02 = T01*T12;

% Jacobian calculation
z0 = [0 0 1]';
z1 = T01(1:3,3);

o0 = [0 0 0]';
o1 = T01(1:3,4);
o2 = T02(1:3,4);

J1 = [w2ss(z0)*(o2-o0); z0];
J2 = [w2ss(z1)*(o2-o1); z1];

% Jacobian
Jg = [J1 J2]

dP = Jg*[dth1 dth2]'
