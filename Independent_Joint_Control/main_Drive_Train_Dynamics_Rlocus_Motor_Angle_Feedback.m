% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% PD controller with motor angle feedback - Root Locus

% System parameters
Jl = 1;     % Load inertia 
Bl = 1;     % Load damping constant
Jm = 10;    % Motor inertia 
Bm = 10;    % Motor damping constant
k = 20;     % Torsional stiffness b/w load and motor

% Control gain 
% Kp + KdS = K(S+a)
Kp = 1;   % P gain
Kd = 30;  % D gain

% System Transfer function with motor angle feedback --> Input: desired step angle, Output: motor angle
den_cm = [Jl*Jm Jl*Bm+Jm*Bl+Jl*Kd (k*(Jl+Jm)+Bl*Bm+Kd*Bl+Kp*Jl) k*(Bl+Bm)+k*Kd+Kp*Bl Kp*k];
num_cm = [Jl Bl k]*Kp;
sys_cm = tf(num_cm,den_cm)

% Draw Root Locus
figure(2)
rlocus(sys_cm)

