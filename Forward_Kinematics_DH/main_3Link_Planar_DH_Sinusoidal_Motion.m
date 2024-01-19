% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Forward Kinematics of 3-Link Planar Manipulator using DH Parameter
deg2rad = pi/180;
rad2deg = 180/pi;

% Camera View
az = 0;
el = 90;

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Constant link lengths
L1 = 1.0;
L2 = 1.0;
L3 = 1.0;

% Sampling rate info
Hz = 100;
dt = 1/Hz;
Rep = 2; % Number of repeat

% Sinusoidal input parameters
A1 = 30.0;
A2 = 60.0;
A3 = 90.0;
w1 = 2*pi;
w2 = 2*pi;
w3 = 2*pi;

% Forward Kinematics
for i = 1:1:Hz*Rep
     
    t = i*dt;
    
    % Sinusoidal Input Joint Angle
    th1 = A1*sin(w1*t) * deg2rad;
    th2 = A2*sin(w2*t) * deg2rad;
    th3 = A3*sin(w3*t) * deg2rad;
    
    % Forward Kinematics using DH convention
    % DH parameter Link 1
    a1 = L1; alpha1 = 0; d1 = 0; q1 = th1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = L2; alpha2 = 0; d2 = 0; q2 = th2; 
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = L3; alpha3 = 0; d3 = 0; q3 = th3;
    T23 = DH(q3, d3, alpha3, a3);

    T02 = T01*T12;
    T03 = T02*T23;

    % Position and axis infomation
    axis_scale = 0.3;
    P1 = T01(1:3,4);
    P1_x = P1 + axis_scale * T01(1:3,1);
    P1_y = P1 + axis_scale * T01(1:3,2);
    P1_z = P1 + axis_scale * T01(1:3,3); 

    P2 = T02(1:3,4);
    P2_x = P2 + axis_scale * T02(1:3,1);
    P2_y = P2 + axis_scale * T02(1:3,2); 
    P2_z = P2 + axis_scale * T02(1:3,3);

    P3 = T03(1:3,4);
    P3_x = P3 + axis_scale * T03(1:3,1);
    P3_y = P3 + axis_scale * T03(1:3,2);
    P3_z = P3 + axis_scale * T03(1:3,3);

    figure(1)
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 3)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 3)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 3)

    plot3(P1(1),P1(2),P1(3),'ro','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)
    line([P1(1) P1_x(1)],[P1(2) P1_x(2)],[P1(3) P1_x(3)], 'color', 'r', 'linewidth', 1)
    line([P1(1) P1_y(1)],[P1(2) P1_y(2)],[P1(3) P1_y(3)], 'color','g', 'linewidth', 1)
    line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color','b', 'linewidth', 1)

    plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)
    line([P2(1) P2_x(1)],[P2(2) P2_x(2)],[P2(3) P2_x(3)], 'color', 'r', 'linewidth', 1)
    line([P2(1) P2_y(1)],[P2(2) P2_y(2)],[P2(3) P2_y(3)], 'color','g', 'linewidth', 1)
    line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color','b', 'linewidth', 1)

    plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)
    line([P3(1) P3_x(1)],[P3(2) P3_x(2)],[P3(3) P3_x(3)], 'color', 'r', 'linewidth', 1)
    line([P3(1) P3_y(1)],[P3(2) P3_y(2)],[P3(3) P3_y(3)], 'color','g', 'linewidth', 1)
    line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color','b', 'linewidth', 1)

    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title('3-Link Planar Manipulator')
    set(gca,'fontsize',13)
    axis equal
    axis([-3 3 -3 3 -3 3])
    grid on
    view(az, el)
    pause(0.001)
    hold off
    
    % Output
    t_out(i) = t;
    th1_out(i) = th1 * rad2deg;
    th2_out(i) = th2 * rad2deg;
    th3_out(i) = th3 * rad2deg;
    
    P3x_out(i) = P3(1);
    P3y_out(i) = P3(2);
    P3z_out(i) = P3(3);
end

figure(2)
subplot(2,1,1)
plot(t_out, th1_out,'r','linewidth',2)
hold on
plot(t_out, th2_out,'k','linewidth',2)
plot(t_out, th3_out,'b','linewidth',2)
xlabel('Time [Sec]')
ylabel('Joint Angle [Deg]')
legend('th1','th2','th3')
set(gca,'fontsize',13)

subplot(2,1,2)
plot(t_out, P3x_out,'r','linewidth',2)
hold on
plot(t_out, P3y_out,'g','linewidth',2)
plot(t_out, P3z_out,'b','linewidth',2)
xlabel('Time [Sec]')
ylabel('End-Effector Position [m]')
legend('x- axis','y- axis','z- axis')
set(gca,'fontsize',13)

