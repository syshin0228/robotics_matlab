% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Force control with 1 DoF linear system in Resistive environment

% Sampling rate info
Hz = 10000;
dt = 1/Hz;

% System Parameters
m = 1.0;      % mass

% Simulation time
t0 = 0.0;   % Initial time
tf = 1.0;   % Final time
time = [t0:1:tf*Hz]*dt; % Time

% Control gain
K = 100;
zeta = 1.0;
B = 2*zeta*sqrt(K);

% Desired Velocity
dx_d = 1.5;

% Environment model
Ke = 1000;      % Stiffness of environment
Be = 10;        % Viscosity of envirionment
Me = 1.0;       % Mass of environment
x_e = 1.0;      % Position of environment
x_ae = 1.0;     % Actual position of actual environment

% Forward Dynamics of 1DoF System
% Initialization
x = 0.8;
dx = 0;
ddx = 0;
F_ex = 0.0;
for i = 1:1:length(time)
    t = time(i);
    
    % Environment Model: Resistive
    if x > x_e
        F_ex = -(Me*ddx+Be*dx);
        x_ae = x;
    elseif x <= x_e
        F_ex = 0.0;
    end
    
    % Robot Dynamics
    u =  B*(dx_d - dx) - F_ex;       % Impedance controller
    ddx = (1/m)*(u + F_ex);         % Dynamics

    % Numerical integration
    dx = dx + ddx*dt;
    x = x + dx*dt;

    ddx_out(i) = ddx;
    dx_out(i) = dx;
    x_out(i) = x;
    x_ae_out(i) = x_ae;
    u_out(i) = u;
    Fex_out(i) = F_ex;
    
end

% Visualize Simulation
step = 100;
y = 0;
y_lim = [-1 1];
for i = 1:step:length(time)
    figure(100)    
    plot(x_out(i),y,'or','linewidth',5)
    hold on
    line([x_e x_e],[y_lim(1) y_lim(2)],'linewidth',1,'color','k','linestyle',':')
    line([x_ae_out(i) x_ae_out(i)],[y_lim(1) y_lim(2)],'linewidth',2,'color','k')
    
    % Draw Force vector
    scale = 1/max(2*abs(Fex_out));
    F_vec = scale*Fex_out(i);
    line([x_out(i) x_out(i)+F_vec],[0 0],'linewidth',2,'color','r')
    title_text = ['F_{ex} = ' sprintf('%0.2f',Fex_out(i)) ' N'];
    title(title_text)
    axis([0 1.2*max(x_out) -1 1])
    xlabel('x- axis')
    ylabel('y- axis')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off
end



figure(1)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
plot(time, x_out,'k','linewidth',2)
ylabel('Displacement [m]')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dx_out,'k','linewidth',2)
ylabel('Velocity [m/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddx_out,'k','linewidth',2)
ylabel('Acceleration [m/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
plot(time, u_out,'k','linewidth',2)
ylabel('Input Force [N]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(3)
plot(time, Fex_out,'k','linewidth',2)
ylabel('External Force [N]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)


