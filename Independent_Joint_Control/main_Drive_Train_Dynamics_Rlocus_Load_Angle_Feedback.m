% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% PD controller with load angle feedback - Root Locus

% System parameters
Jl = 1;     % Load inertia 
Bl = 1;     % Load damping constant
Jm = 10;    % Motor inertia 
Bm = 10;    % Motor damping constant
k = 20;     % Torsional stiffness b/w load and motor

% Control gain
% Kp + KdS = K(S+a)
Kp = 1;     % P gain
Kd = 5;     % D gain

% System Transfer function with load angle feedback --> Input: desired step angle, Output: load angle
den_cl = [Jl*Jm Jl*Bm+Jm*Bl (k*(Jl+Jm)+Bl*Bm) k*(Bl+Bm)+k*Kd Kp*k];
num_cl = [Kp*k];
sys_cl = tf(num_cl,den_cl)

% Draw Root Locus
figure(2)
rlocus(sys_cl)

