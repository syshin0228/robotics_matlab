% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Forward Kinematics of 2 Link Planar Manipulator
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
Hz = 100;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 30.0;
A2 = 60.0;
w1 = 2*pi;
w2 = 2*pi;

% Forward Kinematics
for i = 1:1:Hz*Rep
    
    t = i*dt;
        
    % Sinusoidal Input Joint Angle
    th1 = A1*sin(w1*t) * deg2rad;
    th2 = A2*sin(w2*t) * deg2rad;

    P1(1) = L1*cos(th1);
    P1(2) = L1*sin(th1);

    P2(1) = L1*cos(th1) + L2*cos(th1+th2);
    P2(2) = L1*sin(th1) + L2*sin(th1+th2);

    figure(1)
    plot(P1(1), P1(2),'ro','linewidth',3)
    hold on
    plot(P2(1), P2(2),'ro','linewidth',3)
    line([origin(1) P1(1)],[origin(2) P1(2)], 'color', 'k', 'linewidth', 3)
    line([P1(1) P2(1)],[P1(2) P2(2)], 'color','k', 'linewidth', 3)

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
    
    % Output
    t_out(i) = t;
    th1_out(i) = th1 * rad2deg;
    th2_out(i) = th2 * rad2deg;
    
    P2x_out(i) = P2(1);
    P2y_out(i) = P2(2);

end

figure(2)
subplot(2,1,1)
plot(t_out, th1_out,'r','linewidth',2)
hold on
plot(t_out, th2_out,'k','linewidth',2)
xlabel('Time [Sec]')
ylabel('Joint Angle [Deg]')
legend('th1','th2')
set(gca,'fontsize',13)

subplot(2,1,2)
plot(t_out, P2x_out,'r','linewidth',2)
hold on
plot(t_out, P2y_out,'g','linewidth',2)
xlabel('Time [Sec]')
ylabel('End-Effector Position [m]')
legend('x- axis','y- axis')
set(gca,'fontsize',13)
