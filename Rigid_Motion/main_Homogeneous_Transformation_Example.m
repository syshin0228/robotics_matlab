% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Forward Kinematics using Homogeneous Transformation of 3 Link Planar Manipulator
deg2rad = pi/180;
rad2deg = 180/pi;

% Camera View (https://www.mathworks.com/help/matlab/ref/view.html)
az = 30;    % azimuth
el = 30;    % elevation

% Origin
origin = [0 0 0]'
x_axis = [1 0 0]'
y_axis = [0 1 0]'
z_axis = [0 0 1]'

% Joint angle input
theta = 45 * deg2rad
phi = 30 * deg2rad
psi = 30 * deg2rad

% Forward kinematics using Homogeneous transformation
R01 = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
d01 = R01*[1 0 0]'

R12 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
d12 = R12*[1 0 0]'

R23 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
d23 = R23*[1 0 0]'

T01 = [R01 d01; 0 0 0 1]
T12 = [R12 d12; 0 0 0 1]
T23 = [R23 d23; 0 0 0 1]

T02 = T01*T12
T03 = T02*T23

% Position and axis infomation
axis_scale = 0.3    % Scale axis
P1 = T01(1:3,4)
P1_x = P1 + axis_scale * T01(1:3,1) 
P1_y = P1 + axis_scale * T01(1:3,2) 
P1_z = P1 + axis_scale * T01(1:3,3) 

P2 = T02(1:3,4)
P2_x = P2 + axis_scale * T02(1:3,1) 
P2_y = P2 + axis_scale * T02(1:3,2) 
P2_z = P2 + axis_scale * T02(1:3,3)

P3 = T03(1:3,4)
P3_x = P3 + axis_scale * T03(1:3,1) 
P3_y = P3 + axis_scale * T03(1:3,2)
P3_z = P3 + axis_scale * T03(1:3,3)


figure(1)
% Draw global coordinate
plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 3)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 3)
line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 3)

plot3(P1(1),P1(2),P1(3),'ro','linewidth',2)
line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)
line([P1(1) P1_x(1)],[P1(2) P1_x(2)],[P1(3) P1_x(3)], 'color', 'r', 'linewidth', 1)
line([P1(1) P1_y(1)],[P1(2) P1_y(2)],[P1(3) P1_y(3)], 'color','g', 'linewidth', 1)
line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color','b', 'linewidth', 1)

plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
line([P2(1) P2_x(1)],[P2(2) P2_x(2)],[P2(3) P2_x(3)], 'color', 'r', 'linewidth', 1)
line([P2(1) P2_y(1)],[P2(2) P2_y(2)],[P2(3) P2_y(3)], 'color','g', 'linewidth', 1)
line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color','b', 'linewidth', 1)

plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
line([P3(1) P3_x(1)],[P3(2) P3_x(2)],[P3(3) P3_x(3)], 'color', 'r', 'linewidth', 1)
line([P3(1) P3_y(1)],[P3(2) P3_y(2)],[P3(3) P3_y(3)], 'color','g', 'linewidth', 1)
line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color','b', 'linewidth', 1)
axis equal
grid on
view(az, el)
xlabel('x- axis')
ylabel('y- axis')
zlabel('z- axis')
set(gca,'fontsize',15)

