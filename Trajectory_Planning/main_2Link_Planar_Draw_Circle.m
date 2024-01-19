% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Path Planning of 2 Link Planar Manipulator (Draw Circle)
deg2rad = pi/180;
rad2deg = 180/pi;

% Origin
origin = [0 0]';
x_axis = [1 0]';
y_axis = [0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;

% Sampling rate info
Hz = 10000;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Get initial end-effector position
th1_init = -45 * deg2rad;
th2_init = 45 * deg2rad;

Px_i = L1*cos(th1_init) + L2*cos(th1_init+th2_init);
Py_i = L1*sin(th1_init) + L2*sin(th1_init+th2_init);

% Input parameter for drawing circle
A1 = 0.5;
A2 = 0.5;
w1 = 2*pi;
w2 = 2*pi;
offset1 = 1.0;
offset2 = 1.0;

t = [1:1:Hz*Rep]*dt;
Px = A1*sin(w1*t) + offset1;
Py = A2*cos(w2*t) + offset2;
dPx = A1*w1*cos(w1*t);
dPy = -A2*w2*sin(w2*t);
ddPx = -A1*w1^2*sin(w1*t);
ddPy = -A1*w2^2*cos(w2*t);

% Initial position of circle motion = final position of initial motion
Px_f = Px(1);
Py_f = Py(1);
dPx_f = dPx(1);
dPy_f = dPy(1);
ddPx_f = ddPx(1);
ddPy_f = ddPy(1);

% Given constraint variables
t_init = [0:1:Hz]*dt;
t0 = t_init(1);
tf = t_init(length(t_init));

Px_i = Px_i;
Px_f = Px_f;
vx_i = 0;
vx_f = 0;
ax_i = 0;
ax_f = 0;
Vx = 1.5*(Px_f - Px_i) / tf;

Py_i = Py_i;
Py_f = Py_f;
vy_i = 0;
vy_f = 0;
ay_i = 0;
ay_f = 0;
Vy = 1.5*(Py_f - Py_i) / tf;

% For smooth motion - cubic and quintic only
vx_f = dPx_f;
ax_f = ddPx_f;
vy_f = dPy_f;
ay_f = ddPy_f;

% % Cubic Polynomial Trajectory
% [Px_init, dPx_init, ddPx_init] = cubic_traj(t0, tf, Px_i, Px_f, vx_i, vx_f, Hz); 
% [Py_init, dPy_init, ddPy_init] = cubic_traj(t0, tf, Py_i, Py_f, vy_i, vy_f, Hz); 

% Quintic Polynomial Trajectory
[Px_init, dPx_init, ddPx_init] = quintic_traj(t0, tf, Px_i, Px_f, vx_i, vx_f, ax_i, ax_f, Hz); 
[Py_init, dPy_init, ddPy_init] = quintic_traj(t0, tf, Py_i, Py_f, vy_i, vy_f, ay_i, ay_f, Hz); 

% % LSPB
% [Px_init, dPx_init, ddPx_init] = LSPB_traj(t0, tf, Px_i, Px_f, Vx, Hz); 
% [Py_init, dPy_init, ddPy_init] = LSPB_traj(t0, tf, Py_i, Py_f, Vy, Hz); 

% % Minimum Time Trajectory
% [Px_init, dPx_init, ddPx_init] = min_time_traj(t0, tf, Px_i, Px_f, Hz); 
% [Py_init, dPy_init, ddPy_init] = min_time_traj(t0, tf, Py_i, Py_f, Hz); 


% Full Motion Planning
t = [t_init t_init(length(t_init))+t];
Px = [Px_init Px];
Py = [Py_init Py];
dPx = [dPx_init dPx];
dPy = [dPy_init dPy];

figure(1)
plot(Px,Py,'k:','linewidth',1)
hold on
axis equal
xlabel('x- axis [m]')
ylabel('y- axis [m]')
set(gca,'fontsize',13)

figure(2)
subplot(2,1,1)
plot(t,Px,'r','linewidth',1)
hold on
plot(t,Py,'g','linewidth',1)
ylabel('Displacement [m]')
set(gca,'fontsize',13)
subplot(2,1,2)
plot(t,dPx,'r','linewidth',1)
hold on
plot(t,dPy,'g','linewidth',1)
ylabel('Velocity [m/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

th = [th1_init th2_init];
for i = 1:1:length(t)
    J = [-L1*sin(th(1))-L2*sin(th(1)+th(2)) -L2*sin(th(1)+th(2)); L1*cos(th(1))+L2*cos(th(1)+th(2)) L2*cos(th(1)+th(2))];
    dth(i,:) = inv(J)*[dPx(i) dPy(i)]';
    
    th = th + dth(i,:)*dt;
    
    th1_deg(i) = th(1) * rad2deg;
    th2_deg(i) = th(2) * rad2deg;
    
end

figure(3)
subplot(2,1,1)
plot(t,th1_deg,'r','linewidth',1)
hold on
plot(t,th2_deg,'b','linewidth',1)
ylabel('Joint Angle [Deg]')
set(gca,'fontsize',13)
subplot(2,1,2)
plot(t,dth(:,1),'r','linewidth',1)
hold on
plot(t,dth(:,2),'b','linewidth',1)
ylabel('Angular Velocity [Deg/s]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)


step = 200
% Forward Kinematics
for i = 1:step:length(t)
    t = i*dt;
        
    % Sinusoidal Input Joint Angle with given theta
    th1 = th1_deg(i) * deg2rad;
    th2 = th2_deg(i) * deg2rad;

    P1(1) = L1*cos(th1);
    P1(2) = L1*sin(th1);

    P2(1) = L1*cos(th1) + L2*cos(th1+th2);
    P2(2) = L1*sin(th1) + L2*sin(th1+th2);
    

    figure(1)
    % Given theta
    plot(P1(1), P1(2),'ko','linewidth',2)
    hold on
    plot(P2(1), P2(2),'ko','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)], 'color', 'k', 'linewidth', 2)
    line([P1(1) P2(1)],[P1(2) P2(2)], 'color','k', 'linewidth', 2)

    plot(Px,Py,'k:','linewidth',1)
    % Draw global coordinate
    plot(origin(1), origin(2), 'ro', 'linewidth',5)
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)], 'color', 'r', 'linewidth', 2)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)], 'color','g', 'linewidth', 2)
    axis equal
    axis([-2 2 -2 2])
    xlabel('x- axis')
    ylabel('y- axis')
    title('2-Link Planar Manipulator')
    set(gca,'fontsize',13)
    pause(0.001)
    hold off
    
end