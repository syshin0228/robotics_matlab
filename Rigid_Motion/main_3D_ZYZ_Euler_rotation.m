% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% ZYZ- Euler angle transformation 
syms phi theta psi

Rz1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
Ry2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz3 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz1*Ry2*Rz3

% ZYZ- Euler angle transformation with given angle values in Degree
% To calculate given angles (phi, theta, psi) from ZYZ- Euler angle rotation matrix

phi = 45 
theta = -30
psi = 55 

% Given angle calculation
deg2rad = pi/180;
rad2deg = 180/pi;
phi = phi * deg2rad;
theta = theta * deg2rad;
psi = psi * deg2rad;

Rz1 = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
Ry2 = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz3 = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R = Rz1*Ry2*Rz3

if (R(1,3) ~= 0.0) && (R(2,3) ~= 0.0 && R(3,3) ~= 1 && R(3,3) ~= -1)
    % If sin(theta) > 0 
    theta_out(1) = atan2(sqrt(1-R(3,3)^2), R(3,3));
    phi_out(1) = atan2(R(2,3), R(1,3));
    psi_out(1) = atan2(R(3,2), -R(3,1));
    
    % If sin(theta) < 0 
    theta_out(2) = atan2(-sqrt(1-R(3,3)^2), R(3,3));
    phi_out(2) = atan2(-R(2,3), -R(1,3));
    psi_out(2) = atan2(-R(3,2), R(3,1));
else
    % if cos(theta) = 1, infinite number of solutions for psi and phi
    if R(3,3) == 1 
        theta_out = 0;
        phi_psi = atan2(-R(1,2), R(1,1));   % phi + psi
        phi_out = 0;
        psi_out = phi_psi;
    % if cos(theta) = -1, infinite number of solutions for psi and phi
    elseif R(3,3) == -1
        theta_out = pi;
        phi_psi = atan2(-R(1,2), -R(1,1));  % phi - psi
        phi_out = 0;
        psi_out = phi_psi;
    end  
end

% Calculated angles (Textbook page 54)
phi_out = phi_out*rad2deg
theta_out = theta_out*rad2deg
psi_out = psi_out*rad2deg

