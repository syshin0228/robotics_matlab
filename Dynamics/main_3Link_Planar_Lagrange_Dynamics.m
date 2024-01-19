% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Euler-Lagrange Equations of Motion of 3 Link Planar Manipulator
% Parameterization
syms m1 m2 m3 I1 I2 I3 L1 L2 L3 L1c L2c L3c g;      % Mass, Moment of Inertia, Link lengths, Gravity acc
syms q1 dq1 ddq1 q2 dq2 ddq2 q3 dq3 ddq3;           % Generalized coordinates
syms tau1 tau2 tau3;                                % Generalized force (input torques)

p1x_COM = L1c*cos(q1);              % Position of Link 1 x COM 
p1y_COM = L1c*sin(q1);              % Position of Link 1 y COM
p1x = L1*cos(q1);                   % Position of Link 1 x end point 
p1y = L1*sin(q1);                   % Position of Link 1 y end point
p2x_COM = p1x+L2c*cos(q1+q2);       % Position of Link 2 x COM
p2y_COM = p1y+L2c*sin(q1+q2);       % Position of Link 2 y COM
p2x = p1x+L2*cos(q1+q2);            % Position of Link 2 x end point 
p2y = p1y+L2*sin(q1+q2);            % Position of Link 2 y end point
p3x_COM = p2x+L3c*cos(q1+q2+q3);    % Position of Link 3 x COM
p3y_COM = p2y+L3c*sin(q1+q2+q3);    % Position of Link 3 y COM
p3x = p2x+L3*cos(q1+q2+q3);         % Position of Link 3 x end point 
p3y = p2y+L3*sin(q1+q2+q3);         % Position of Link 3 y end point

v1x_COM = -L1c*sin(q1)*dq1;                          % Velocity of Link 1 x COM
v1y_COM = +L1c*cos(q1)*dq1;                          % Velocity of Link 1 y COM
v1x = -L1*sin(q1)*dq1;                               % Velocity of Link 1 x end point
v1y = +L1*cos(q1)*dq1;                               % Velocity of Link 1 y end point
v2x_COM = v1x-L2c*sin(q1+q2)*(dq1+dq2);              % Velocity of Link 2 x COM
v2y_COM = v1y+L2c*cos(q1+q2)*(dq1+dq2);              % Velocity of Link 2 y COM
v2x = v1x-L2*sin(q1+q2)*(dq1+dq2);                   % Velocity of Link 2 x end point   
v2y = v1y+L2*cos(q1+q2)*(dq1+dq2);                   % Velocity of Link 2 y end point
v3x_COM = v2x-L3c*sin(q1+q2+q3)*(dq1+dq2+dq3);       % Velocity of Link 2 x COM
v3y_COM = v2y+L3c*cos(q1+q2+q3)*(dq1+dq2+dq3);       % Velocity of Link 2 y COM
v3x = v2x-L3*sin(q1+q2+q3)*(dq1+dq2+dq3);            % Velocity of Link 2 x end point   
v3y = v2y+L3*cos(q1+q2+q3)*(dq1+dq2+dq3);            % Velocity of Link 2 y end point

w1 = dq1;                     % Angular Velocity of Link 1 
w2 = dq1 + dq2;               % Angular Velocity of Link 2 
w3 = dq1 + dq2 + dq3;         % Angular Velocity of Link 3 

% Kinetic Energy
KE = 0.5*m1*(v1x_COM^2 + v1y_COM^2) + 0.5*m2*(v2x_COM^2 + v2y_COM^2) + 0.5*m3*(v3x_COM^2 + v3y_COM^2) + 0.5*I1*w1^2 + 0.5*I2*w2^2 + 0.5*I3*w3^2;
KE = simplify(KE);

% Potential Energy
PE = m1*g*p1y_COM + m2*g*p2y_COM + m3*g*p3y_COM;
PE = simplify(PE);

% Euler-Lagrange Equations of Motion
pKEpdq1 = diff(KE,dq1);
ddtpKEpdq1 = diff(pKEpdq1,q1)*dq1+ ...
             diff(pKEpdq1,dq1)*ddq1+ ...
             diff(pKEpdq1,q2)*dq2 + ...
             diff(pKEpdq1,dq2)*ddq2 + ...
             diff(pKEpdq1,q3)*dq3 + ...
             diff(pKEpdq1,dq3)*ddq3;
pKEpq1 = diff(KE,q1);
pPEpq1 = diff(PE,q1);

pKEpdq2 = diff(KE,dq2);
ddtpKEpdq2 = diff(pKEpdq2,q1)*dq1+ ...
             diff(pKEpdq2,dq1)*ddq1+ ...
             diff(pKEpdq2,q2)*dq2 + ...
             diff(pKEpdq2,dq2)*ddq2 + ...
             diff(pKEpdq2,q3)*dq3 + ...
             diff(pKEpdq2,dq3)*ddq3;
pKEpq2 = diff(KE,q2);
pPEpq2 = diff(PE,q2);

pKEpdq3 = diff(KE,dq3);
ddtpKEpdq3 = diff(pKEpdq3,q1)*dq1+ ...
             diff(pKEpdq3,dq1)*ddq1+ ...
             diff(pKEpdq3,q2)*dq2 + ...
             diff(pKEpdq3,dq2)*ddq2 + ...
             diff(pKEpdq3,q3)*dq3 + ...
             diff(pKEpdq3,dq3)*ddq3;
pKEpq3 = diff(KE,q3);
pPEpq3 = diff(PE,q3);

% Equations of Motion 
eqq1 = simplify(ddtpKEpdq1 - pKEpq1 + pPEpq1 - tau1);
eqq2 = simplify(ddtpKEpdq2 - pKEpq2 + pPEpq2 - tau2);
eqq3 = simplify(ddtpKEpdq3 - pKEpq3 + pPEpq3 - tau3);

% Inverse Dynamics
ID = solve(eqq1, eqq2, eqq3, tau1, tau2, tau3);

% Forward Dynamics
FD = solve(eqq1, eqq2, eqq3, ddq1, ddq2, ddq3);

ID.tau1 = simplify(ID.tau1);
ID.tau2 = simplify(ID.tau2);
ID.tau3 = simplify(ID.tau3);
FD.ddq1 = simplify(FD.ddq1);
FD.ddq2 = simplify(FD.ddq2);
FD.ddq3 = simplify(FD.ddq3);

% Print out Inverse and Forward Dynamics
tau1 = ID.tau1
tau2 = ID.tau2
tau3 = ID.tau3
ddq1 = FD.ddq1
ddq2 = FD.ddq2
ddq3 = FD.ddq3
