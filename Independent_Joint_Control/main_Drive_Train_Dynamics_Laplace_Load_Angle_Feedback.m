% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% PD controller with load angle feedback in Laplace and Time domain

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System parameters
Jl = 1;     % Load inertia 
Bl = 1;     % Load damping constant
Jm = 10;    % Motor inertia 
Bm = 10;    % Motor damping constant
k = 20;     % Torsional stiffness b/w load and motor

% Control gain
Kp = 20;    % P gain
Kd = 5;     % D gain

% System Transfer function with load angle feedback --> Input: desired step angle, Output: load angle
den_cl = [Jl*Jm Jl*Bm+Jm*Bl (k*(Jl+Jm)+Bl*Bm) k*(Bl+Bm)+k*Kd Kp*k];
num_cl = [Kp*k];
sys_cl = tf(num_cl,den_cl)

% System Transfer function with load angle feedback --> Input: desired step angle, Output: motor angle
den_cm = [Jl*Jm Jl*Bm+Jm*Bl (k*(Jl+Jm)+Bl*Bm) k*(Bl+Bm)+k*Kd Kp*k];
num_cm = [Jl Bl k]*Kp;
sys_cm = tf(num_cm,den_cm)

% Simulation time
t0 = 0.0;   % Initial time
tf = 10.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Step input
[y_cl,t] = step(sys_cl,time);
[y_cm,t] = step(sys_cm,time);

figure(1)
h1 = plot(t, y_cl, 'r','linewidth',1);
hold on
h2 = plot(t, y_cm, 'b','linewidth',1);
ylabel('Joint Angle [deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

% Initialization
ddth_l = 0;
dth_l = 0;
th_l = 0;
ddth_m = 0;
dth_m = 0;
th_m = 0;
th_d = 1;  % Desired step input

% Simulation in time domain
for i = 1:1:length(time)
    t = time(i);
    u = Kp*(th_d - th_l) - Kd*dth_l;                        % PD controller with load angle feedback
    ddth_m = (1/Jm) * (u - Bm*dth_m + k*(th_l - th_m));     % Motor dynamics
    ddth_l = (1/Jl) * (- Bl*dth_l + k*(th_m - th_l));       % Load dynamics

    % Numerical integration
    dth_m = dth_m + ddth_m*dt;
    th_m = th_m + dth_m*dt;
    dth_l = dth_l + ddth_l*dt;
    th_l = th_l + dth_l*dt;
    
    th_d_out(i) = th_d;
    ddth_m_out(i) = ddth_m;
    dth_m_out(i) = dth_m;
    th_m_out(i) = th_m;
    u_m_out(i) = u;
    
    ddth_l_out(i) = ddth_l;
    dth_l_out(i) = dth_l;
    th_l_out(i) = th_l;
    
end

figure(1)
h3 = plot(time, th_l_out,'r:','linewidth',2);
h4 = plot(time, th_m_out,'b:','linewidth',2);
h5 = plot(time,th_d_out,'k:');
ylabel('Joint Angle [deg]')
xlabel('Time [Sec]')
legend([h1, h2, h3, h4, h5], 'Load Angle (Laplace)', 'Motor Angle (Laplace)', 'Load Angle (Time)', 'Motor Angle (Time)', 'Desired Motor Angle')
set(gca,'fontsize',13)
