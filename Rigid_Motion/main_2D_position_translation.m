% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% 2D Postion Transformation

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';

% Position vectors
P1 = [1 3]';
P2 = [2 5]';
P3 = P1 - P2;

figure(1)
% Draw global coordinate
plot(origin(1),origin(2),'ro','linewidth',2)
hold on
line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 2)
line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 2)

plot(P1(1),P1(2),'ro')
plot(P2(1),P2(2),'bo')
plot(P3(1),P3(2),'ko')

axis equal
xlabel('x- axis')
ylabel('y- axis')
set(gca,'fontsize',15)
