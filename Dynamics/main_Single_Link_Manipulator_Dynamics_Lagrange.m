% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Euler-Lagrange Equations of Motion of Single Link Manipulator
% Parameterization
syms m J Lc L g B;  % m: mass, J: Inertia, Lc: link length to CoM, g: gravity acc, B: Damping coefficient
syms q dq ddq;      % Generalized coordinates
syms u tau;         % tau: generaized force (input torque), u: Control input

% Kinetic Energy
KE = 0.5*J*dq^2;

% Potential Energy
PE = m*g*Lc*(1-cos(q));

% Input torque
tau = u - B*dq;

% Euler-Lagrange Equations of Motion
pKEpdq = diff(KE,dq);
ddtpKEpdq = diff(pKEpdq,q)*dq+ ...
             diff(pKEpdq,dq)*ddq;
pKEpq = diff(KE,q);
pPEpq = diff(PE,q);

% Equations of Motion 
eq = simplify(ddtpKEpdq - pKEpq + pPEpq - tau);

% Inverse Dynamics
ID = solve(eq,u);

% Forward Dynamics
FD = solve(eq,ddq);

% Print out Inverse and Forward Dynamics
ID
FD
