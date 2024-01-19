% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Jacobain of 7DoF Arm Manipulator using DH Parameter
syms L0 L1 L2 L3 th1 th2 th3 th4 th5 th6 th7 dth1 dth2 dth3 dth4 dth5 dth6 dth7 

% Forward Kinematics using DH convention
% DH parameter Ground
a0 = 0; alpha0 = -pi/2; d0 = 0; q0 = 0;
TG0 = DH(q0, d0, alpha0, a0);

% DH parameter Link 1
a1 = 0; alpha1 = -pi/2; d1 = L0; q1 = th1-pi/2;
T01 = DH(q1, d1, alpha1, a1);

% DH parameter Link 2
a2 = 0; alpha2 = pi/2; d2 = 0; q2 = th2+pi/2;
T12 = DH(q2, d2, alpha2, a2);

% DH parameter Link 3
a3 = 0; alpha3 = -pi/2; d3 = -L1; q3 = th3+pi/2;
T23 = DH(q3, d3, alpha3, a3);

% DH parameter Link 4
a4 = 0; alpha4 = pi/2; d4 = 0; q4 = th4;
T34 = DH(q4, d4, alpha4, a4);

% DH parameter Link 5
a5 = 0; alpha5 = -pi/2; d5 = -L2; q5 = th5;
T45 = DH(q5, d5, alpha5, a5);

% DH parameter Link 6
a6 = 0; alpha6 = -pi/2; d6 = 0; q6 = th6-pi/2;
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
J = [J1 J2 J3 J4 J5 J6 J7]
dP = J*[dth1 dth2 dth3 dth4 dth5 dth6 dth7]'

