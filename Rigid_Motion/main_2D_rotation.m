% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% 2D Rotation Transformation
deg2rad = pi/180;

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';

% Input Angle
theta = 45*deg2rad;

% Rotation matrix
Rz = [cos(theta) -sin(theta); sin(theta) cos(theta)];

% Get coordinate position
x = Rz(:,1);
y = Rz(:,2);

% Scale axis
axis_scale = 0.5;
x = axis_scale * x;
y = axis_scale * y;

figure(1)
% Draw global coordinate
plot(origin(1),origin(2),'ro','linewidth',2)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 1)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 1)

% Draw relative coordinate
line([origin(1) x(1)],[origin(2) x(2)], 'color', 'r', 'linewidth', 3)
line([origin(1) y(1)],[origin(2) y(2)], 'color','g', 'linewidth', 3)

axis equal
xlabel('x- axis')
ylabel('y- axis')
set(gca,'fontsize',15)