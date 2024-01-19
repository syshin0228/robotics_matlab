% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Analytically solving for Jacobian of 2-Link Planar Manipulator
syms L1 L2 q1 q2 dq1 dq2

% Forward Kinematics
P2(1,:) = L1*cos(q1) + L2*cos(q1+q2);
P2(2,:) = L1*sin(q1) + L2*sin(q1+q2);

% Analytical Jacobian
Ja = [-L1*sin(q1)-L2*sin(q1+q2) -L2*sin(q1+q2); L1*cos(q1)+L2*cos(q1+q2) L2*cos(q1+q2)]

% Calculate time derivative of Jacobian
dJa11 = diff(Ja(1,1),q1)*dq1 + diff(Ja(1,1),q2)*dq2;
dJa12 = diff(Ja(1,2),q1)*dq1 + diff(Ja(1,2),q2)*dq2;
dJa21 = diff(Ja(2,1),q1)*dq1 + diff(Ja(2,1),q2)*dq2;
dJa22 = diff(Ja(2,2),q1)*dq1 + diff(Ja(2,2),q2)*dq2;

dJa = [dJa11 dJa12; dJa21 dJa22]


