% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Forward Kinematics of 2 Link Planar Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;

% Joint angle input
th1 = 30.0 * deg2rad;
th2 = 45.0 * deg2rad;

% Forward Kinematics
P1(1) = L1*cos(th1);
P1(2) = L1*sin(th1);

P2(1) = L1*cos(th1) + L2*cos(th1+th2);
P2(2) = L1*sin(th1) + L2*sin(th1+th2);

figure(1)
plot(P1(1), P1(2),'ro','linewidth',3)
hold on
plot(P2(1), P2(2),'ro','linewidth',3)
line([origin(1) P1(1)],[origin(2) P1(2)], 'color', 'k', 'linewidth', 3)
line([P1(1) P2(1)],[P1(2) P2(2)], 'color','k', 'linewidth', 3)

% Draw global coordinate
plot(origin(1), origin(2), 'ro', 'linewidth',5)
line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 2)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 2)
axis equal
xlabel('x- axis')
ylabel('y- axis')
title('2-Link Planar Manipulator')
set(gca,'fontsize',13)
