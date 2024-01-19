% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Simulate PD control in time domain with various control gains

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System parameters
J = 1;          % Motor inertia = actuator inertia + gear inertia (Ja + Jg)
b = 1;          % Effective Damping term
tau_l = 10.0;   % Load torque
r = 1;          % gear ratio
d = tau_l/r;    % disturbance input

% Simulation time
t0 = 0.0;   % Initial time
tf = 2.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Natural frequencies of 2nd order system
w_n = [4 8 12];

th_d = 1;               % Desired step input
for k = 1:length(w_n)
    zeta = 1;   % Damping ratio (zeta = 1 is critical damping)
    w = w_n(k);
    
    % Control gain
    Kp = w^2*J;             % P gain
    Kd = 2*zeta*w*J - b;    % D gain

    ddth = 0.0;
    dth = 0.0;
    th = 0.0;

    for i = 1:1:length(time)
        t = time(i);
        u = Kp*(th_d - th) - Kd*dth;    % PD controller
        ddth = (u - d - b*dth);         % Actuator dynamics

        % Numerical integration
        dth = dth + ddth*dt;
        th = th + dth*dt;
        
        th_d_out(i) = th_d;
        ddth_out(i) = ddth;
        dth_out(i) = dth;
        th_out(i) = th;
        u_out(i) = u;
    end

    lcolor = ['k' 'r' 'b'];
    set(gcf,'position',[0 10 1200 700]);
    figure(1)
    subplot(2,2,1)
    h1{k} = plot(time, th_out,lcolor(k),'linewidth',2);
    hold on
    h2 = plot(time, th_d_out,'k:');
    ylabel('Joint Angle [deg]')
    xlabel('Time [sec]')
    ylim([0 th_d*1.2])
    set(gca,'fontsize',13)
    subplot(2,2,2)
    plot(time, dth_out,lcolor(k),'linewidth',2)
    hold on
    ylabel('Joint Velocity [deg/sec]')
    xlabel('Time [sec]')
    set(gca,'fontsize',13)
    
    subplot(2,2,3)
    plot(time, ddth_out,lcolor(k),'linewidth',2)
    hold on
    ylabel('Joint Acceleration [deg/sec^2]')
    xlabel('Time [sec]')
    set(gca,'fontsize',13)
    
    subplot(2,2,4)
    plot(time, u_out,lcolor(k),'linewidth',2)
    hold on
    ylabel('Input u(t)')
    xlabel('Time [sec]')
    set(gca,'fontsize',13)
end
legend([h1{1},h1{2},h1{3},h2],'\omega_n = 4', '\omega_n = 8', '\omega_n = 12', 'Desired Input')


