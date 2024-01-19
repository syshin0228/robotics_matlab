% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% 3D Postion Transformation

% Camera View (https://www.mathworks.com/help/matlab/ref/view.html)
az = 30;    % azimuth
el = 30;    % elevation

origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Position vectors
P1 = [3 2 1]';
P2 = [1 1 1]';
P3 = P1 - P2;

figure(1)
hold on
plot3(P1(1),P1(2),P1(3),'ro')
plot3(P2(1),P2(2),P2(3),'bo')
plot3(P3(1),P3(2),P3(3),'ko')

% Draw global coordinate
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 2)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 2)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 2)
axis equal
grid on
view(az, el)
xlabel('x- axis')
ylabel('y- axis')
zlabel('z- axis')
set(gca,'fontsize',15)

