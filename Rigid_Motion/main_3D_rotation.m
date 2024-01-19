% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% 3D Rotation Transformation 
deg2rad = pi/180;

% Camera View (https://www.mathworks.com/help/matlab/ref/view.html)
az = 30;    % azimuth
el = 30;    % elevation

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Input Angles
alpha = 30*deg2rad;     % x- axis rotation
beta = 30*deg2rad;      % y- axis rotation
gamma = 45*deg2rad;     % z- axis rotation

% Rotation matrix
Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)]
Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)]
Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1]

% Get coordinate position (x- axis rotation)
x_Rx = Rx(:,1);
y_Rx = Rx(:,2);
z_Rx = Rx(:,3);

% Get coordinate position (y- axis rotation)
x_Ry = Ry(:,1);
y_Ry = Ry(:,2);
z_Ry = Ry(:,3);

% Get coordinate position (z- axis rotation)
x_Rz = Rz(:,1);
y_Rz = Rz(:,2);
z_Rz = Rz(:,3);

% Scale axis
axis_scale = 0.5;
x_Rx = axis_scale * x_Rx;
y_Rx = axis_scale * y_Rx;
z_Rx = axis_scale * z_Rx;

x_Ry = axis_scale * x_Ry;
y_Ry = axis_scale * y_Ry;
z_Ry = axis_scale * z_Ry;

x_Rz = axis_scale * x_Rz;
y_Rz = axis_scale * y_Rz;
z_Rz = axis_scale * z_Rz;

figure(1)
set(gcf,'Position', [0 0 1200 500])
subplot(1,3,1)
% Draw global coordinate
plot3(origin(1),origin(2),origin(3),'ro','linewidth',2)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 1)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 1)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 1)

line([origin(1) x_Rx(1)],[origin(2) x_Rx(2)],[origin(3) x_Rx(3)], 'color', 'r', 'linewidth', 3)
line([origin(1) y_Rx(1)],[origin(2) y_Rx(2)],[origin(3) y_Rx(3)], 'color','g', 'linewidth', 3)
line([origin(1) z_Rx(1)],[origin(2) z_Rx(2)],[origin(3) z_Rx(3)], 'color','b', 'linewidth', 3)
xlabel('x- axis')
ylabel('y- axis')
zlabel('z- axis')
title('Rotation about x- axis')
set(gca,'fontsize',13)
axis equal
grid on
view(az, el)

subplot(1,3,2)
% Draw global coordinate
plot3(origin(1),origin(2),origin(3),'ro','linewidth',2)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 1)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 1)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 1)

line([origin(1) x_Ry(1)],[origin(2) x_Ry(2)],[origin(3) x_Ry(3)], 'color', 'r', 'linewidth', 3)
line([origin(1) y_Ry(1)],[origin(2) y_Ry(2)],[origin(3) y_Ry(3)], 'color','g', 'linewidth', 3)
line([origin(1) z_Ry(1)],[origin(2) z_Ry(2)],[origin(3) z_Ry(3)], 'color','b', 'linewidth', 3)
xlabel('x- axis')
ylabel('y- axis')
zlabel('z- axis')
title('Rotation about y- axis')
set(gca,'fontsize',13)
axis equal
grid on
view(az, el)

subplot(1,3,3)
% Draw global coordinate
plot3(origin(1),origin(2),origin(3),'ro','linewidth',2)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 1)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 1)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 1)

line([origin(1) x_Rz(1)],[origin(2) x_Rz(2)],[origin(3) x_Rz(3)], 'color', 'r', 'linewidth', 3)
line([origin(1) y_Rz(1)],[origin(2) y_Rz(2)],[origin(3) y_Rz(3)], 'color','g', 'linewidth', 3)
line([origin(1) z_Rz(1)],[origin(2) z_Rz(2)],[origin(3) z_Rz(3)], 'color','b', 'linewidth', 3)
xlabel('x- axis')
ylabel('y- axis')
zlabel('z- axis')
title('Rotation about z- axis')
set(gca,'fontsize',13)
axis equal
grid on
view(az, el)

