% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Axis/Angle Representation
syms kx ky kz th v

v = 1 - cos(th);
R = [kx^2*v+cos(th) kx*ky*v-kz*sin(th) kx*kz*v+ky*sin(th); kx*ky*v+kz*sin(th) ky^2*v+cos(th) ky*kz*v-kx*sin(th); kx*kz*v-ky*sin(th) ky*kz*v+kx*sin(th) kz^2*v+cos(th)]


% Axis/Angle Representation with given variables (k, theta)
% To calculate given variables (k, theta) from Axis/Angle Representation rotation matrix
k = [1 1 1]';
k_unit = k / norm(k)    % Convert to unit vector
th = 10      

% Given variables calculation
deg2rad = pi/180;
rad2deg = 180/pi;
th = th*deg2rad;
kx = k_unit(1);
ky = k_unit(2);
kz = k_unit(3);

v = 1 - cos(th);
R = [kx^2*v+cos(th) kx*ky*v-kz*sin(th) kx*kz*v+ky*sin(th); kx*ky*v+kz*sin(th) ky^2*v+cos(th) ky*kz*v-kx*sin(th); kx*kz*v-ky*sin(th) ky*kz*v+kx*sin(th) kz^2*v+cos(th)]

% Calcuated variables (k, theta) 
th_out = acos((R(1,1) + R(2,2) + R(3,3) -1) / 2) * rad2deg
k_out = 1/(2*sin(th))*[R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)]
