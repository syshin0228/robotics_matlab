% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Force control with 3 Link Planar Manipulator in Resitive environment

% Angle coversion
deg2rad = pi/180;
rad2deg = 180/pi;

% Sampling rate info
Hz = 1000;
dt = 1/Hz;

% System Parameters
L1 = 0.5;       % Link1 length
L2 = 0.5;       % Link2 length
L3 = 0.5;       % Link3 length
L1c = 0.25;     % Link1 CoM length
L2c = 0.25;     % Link2 CoM length
L3c = 0.25;     % Link3 CoM length
m1 = 1;         % Mass 1
m2 = 1;         % Mass 2
m3 = 1;         % Mass 3
I1 = 1;         % Moment of inertia of link 1
I2 = 1;         % Moment of inertia of link 2
I3 = 1;         % Moment of inertia of link 3
g = 9.81;       % Gravity acceleration
B = 0.0;        % Damping coefficient

% Simulation time
t0 = 0.0;                    % Initial time
tf = 5.0;                    % Final time
time = [t0:1:tf*Hz]*dt;      % Time

% Motion Control Gain
zeta = 1.0;
Kp = 200.0;
Kd = 2*zeta*sqrt(Kp);

% Force Control Gain
Kf = 1000.0;
Bf = 300.0;

% Desired step position
P_d = [0.8 0.8]';       % Desired Px and Py
alpha_d = 45 * pi/180;  % Desired orientation of end-effector
P = [0 0]';           % Initialize current end-effector position

% Desired velocity
dP_d = [-0.2 0.1];    
dalpha_d = 0.0;

% Environment model
Be = 100.0;             % Viscosity of environment
P_e = [0.8 0.8]';       % Position of environment
P_ae = P_e;             % Actual position of actual environment
F_ex = [0 0 0]';        % External force
 
% Forward Dynamics 
% Initialization
th = [-30 30 30]'*deg2rad;
dth = [0 0 0]';
ddth = [0 0 0]';
for i = 1:1:length(time)
    
    t = time(i);
    
    % Contact: if distance between end-effector and environment is less
    % than detect
    detect = 0.001;
    dist = norm(P-P_ae);
    
    % Environment Model
    if dist < detect
        F_ex(1,:) = -Be*dP(1);              % Resisitve in x- axis      
        F_ex(2,:) = -Be*dP(2);              % Resisitve in y- axis
        F_ex(3,:) = 0.0;                    % Free in orientaion
        P_ae = P;
    else
        F_ex = [0 0 0]';
    end
    
    % Current variables (joint positions, joint velocities)
    q1 = th(1);
    q2 = th(2);
    q3 = th(3);
    dq1 = dth(1);
    dq2 = dth(2);
    dq3 = dth(3);

    % Jacobian 
    J = [ - L1*sin(q1) - L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), - L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L2*cos(q1)*sin(q2) - L2*cos(q2)*sin(q1), - L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) - L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2));    L1*cos(q1) + L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2),   L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2),   L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)); 1 1 1];

    % Time derivative of Jacobian
    dJ = [ - dq2*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)) - dq3*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - dq1*(L1*cos(q1) + L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)), - dq1*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)) - dq2*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)) - dq3*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))), - dq1*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - dq2*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1))) - dq3*(L3*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)));  - dq2*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) - dq3*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - dq1*(L1*sin(q1) + L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)), - dq1*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) - dq2*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) - dq3*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))), - dq1*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - dq2*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))) - dq3*(L3*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2))); 0 0 0];    
    
    % Forward Kinematics using DH convention
    % DH parameter Link 1
    a1 = L1; alpha1 = 0; d1 = 0; q1;
    T01 = DH(q1, d1, alpha1, a1);

    % DH parameter Link 2
    a2 = L2; alpha2 = 0; d2 = 0; q2; 
    T12 = DH(q2, d2, alpha2, a2);

    % DH parameter Link 3
    a3 = L3; alpha3 = 0; d3 = 0; q3;
    T23 = DH(q3, d3, alpha3, a3);

    T02 = T01*T12;
    T03 = T02*T23;

    % Position of End-effector
    P = T03(1:2,4);         % [Px Py]'
    alpha = q1 + q2 + q3;   % Orientation alpha
    
    % Velocity of End-effector
    dq = [dq1 dq2 dq3]';
    dP = J * dq;
    
    % Inertial Matrix
    M = [I1 + I2 + I3 + m3*(L1*cos(q1) + L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2))^2 + m3*(L1*sin(q1) + L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1))^2 + m2*(L1*cos(q1) + L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2))^2 + m2*(L1*sin(q1) + L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1))^2 + L1c^2*m1*cos(q1)^2 + L1c^2*m1*sin(q1)^2, I2 + I3 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1))*(L1*sin(q1) + L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) + m2*(L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2))*(L1*cos(q1) + L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2)) + m2*(L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1))*(L1*sin(q1) + L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1)) + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2))*(L1*cos(q1) + L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)), I3 + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(L1*cos(q1) + L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)) + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(L1*sin(q1) + L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)); I2 + I3 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1))*(L1*sin(q1) + L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) + m2*(L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2))*(L1*cos(q1) + L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2)) + m2*(L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1))*(L1*sin(q1) + L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1)) + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2))*(L1*cos(q1) + L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)), I2 + I3 + m2*(L2c*cos(q1)*sin(q2) + L2c*cos(q2)*sin(q1))^2 + m2*(L2c*cos(q1)*cos(q2) - L2c*sin(q1)*sin(q2))^2 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1))^2 + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2))^2, I3 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2));  I3 + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(L1*cos(q1) + L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)) + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(L1*sin(q1) + L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)), I3 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) + L2*cos(q1)*sin(q2) + L2*cos(q2)*sin(q1)) + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L2*cos(q1)*cos(q2) - L2*sin(q1)*sin(q2)), I3 + m3*(L3c*cos(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)) + L3c*sin(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)))^2 + m3*(L3c*cos(q3)*(cos(q1)*cos(q2) - sin(q1)*sin(q2)) - L3c*sin(q3)*(cos(q1)*sin(q2) + cos(q2)*sin(q1)))^2];

    % Control Input
    % Outer Loop Controller
    if t < 1.0  % Position control
        a_x(1,:) = -Kd*dP(1) + Kp*(P_d(1) - P(1));
        a_x(2,:) = -Kd*dP(2) + Kp*(P_d(2) - P(2));
        a_x(3,:) = -Kd*dP(3) + Kp*(alpha_d - alpha);

    elseif t >= 1.0 && t < 4.0 % Start velocity control with resistive environment
        
        a_x(1,:) = Bf*(dP_d(1) - dP(1)) - F_ex(1);
        a_x(2,:) = Bf*(dP_d(2) - dP(2)) - F_ex(2);                 
        a_x(3,:) = Bf*(dalpha_d - dP(3)) - F_ex(3);

        W = J*inv(M)*J';    % Mobility tensor
        a_x = W*a_x;
        
    else % Stop
        
        a_x(1,:) = Bf*(0.0 - dP(1)) - F_ex(1);
        a_x(2,:) = Bf*(0.0 - dP(2)) - F_ex(2);                 
        a_x(3,:) = Bf*(0.0 - dP(3)) - F_ex(3);

        W = J*inv(M)*J';    % Mobility tensor
        a_x = W*a_x;
        
    end
    
    a_q = inv(J)*(a_x - dJ*dq);
    
    % Inner Loop Controller
    ddq1 = a_q(1);
    ddq2 = a_q(2);
    ddq3 = a_q(3);
    
    u(1) = I1*ddq1 + I2*ddq1 + I2*ddq2 + I3*ddq1 + I3*ddq2 + I3*ddq3 + L1^2*ddq1*m2 + L1^2*ddq1*m3 + L2^2*ddq1*m3 + L2^2*ddq2*m3 + L1c^2*ddq1*m1 + L2c^2*ddq1*m2 + L2c^2*ddq2*m2 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L2*g*m3*cos(q1 + q2) + L2c*g*m2*cos(q1 + q2) + L1*g*m2*cos(q1) + L1*g*m3*cos(q1) + L1c*g*m1*cos(q1) + L3c*g*m3*cos(q1 + q2 + q3) - L1*L2*dq2^2*m3*sin(q2) - L1*L2c*dq2^2*m2*sin(q2) - L2*L3c*dq3^2*m3*sin(q3) + 2*L1*L3c*ddq1*m3*cos(q2 + q3) + L1*L3c*ddq2*m3*cos(q2 + q3) + L1*L3c*ddq3*m3*cos(q2 + q3) + 2*L1*L2*ddq1*m3*cos(q2) + L1*L2*ddq2*m3*cos(q2) + 2*L1*L2c*ddq1*m2*cos(q2) + L1*L2c*ddq2*m2*cos(q2) + 2*L2*L3c*ddq1*m3*cos(q3) + 2*L2*L3c*ddq2*m3*cos(q3) + L2*L3c*ddq3*m3*cos(q3) - L1*L3c*dq2^2*m3*sin(q2 + q3) - L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2*dq1*dq2*m3*sin(q2) - 2*L1*L2c*dq1*dq2*m2*sin(q2) - 2*L2*L3c*dq1*dq3*m3*sin(q3) - 2*L2*L3c*dq2*dq3*m3*sin(q3) - 2*L1*L3c*dq1*dq2*m3*sin(q2 + q3) - 2*L1*L3c*dq1*dq3*m3*sin(q2 + q3) - 2*L1*L3c*dq2*dq3*m3*sin(q2 + q3);
    u(2) = I2*ddq1 + I2*ddq2 + I3*ddq1 + I3*ddq2 + I3*ddq3 + L2^2*ddq1*m3 + L2^2*ddq2*m3 + L2c^2*ddq1*m2 + L2c^2*ddq2*m2 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L2*g*m3*cos(q1 + q2) + L2c*g*m2*cos(q1 + q2) + L3c*g*m3*cos(q1 + q2 + q3) + L1*L2*dq1^2*m3*sin(q2) + L1*L2c*dq1^2*m2*sin(q2) - L2*L3c*dq3^2*m3*sin(q3) + L1*L3c*ddq1*m3*cos(q2 + q3) + L1*L2*ddq1*m3*cos(q2) + L1*L2c*ddq1*m2*cos(q2) + 2*L2*L3c*ddq1*m3*cos(q3) + 2*L2*L3c*ddq2*m3*cos(q3) + L2*L3c*ddq3*m3*cos(q3) + L1*L3c*dq1^2*m3*sin(q2 + q3) - 2*L2*L3c*dq1*dq3*m3*sin(q3) - 2*L2*L3c*dq2*dq3*m3*sin(q3);
    u(3) = I3*ddq1 + I3*ddq2 + I3*ddq3 + L3c^2*ddq1*m3 + L3c^2*ddq2*m3 + L3c^2*ddq3*m3 + L3c*g*m3*cos(q1 + q2 + q3) + L2*L3c*dq1^2*m3*sin(q3) + L2*L3c*dq2^2*m3*sin(q3) + L1*L3c*ddq1*m3*cos(q2 + q3) + L2*L3c*ddq1*m3*cos(q3) + L2*L3c*ddq2*m3*cos(q3) + L1*L3c*dq1^2*m3*sin(q2 + q3) + 2*L2*L3c*dq1*dq2*m3*sin(q3);

    % External Force
    tau_ex = J'*F_ex;
    
    tau1 = u(1) -B*dth(1) + tau_ex(1);
    tau2 = u(2) -B*dth(2) + tau_ex(2);
    tau3 = u(3) -B*dth(3) + tau_ex(3);
    
    % Dynamic equations of motion
    ddq1 = (4*I2*I3*tau1 - 4*I2*I3*tau2 + 4*I3*L2^2*m3*tau1 - 4*I3*L2^2*m3*tau2 + 4*I3*L2c^2*m2*tau1 + 4*I2*L3c^2*m3*tau1 - 4*I3*L2c^2*m2*tau2 - 4*I2*L3c^2*m3*tau2 + 2*L2^2*L3c^2*m3^2*tau1 - 2*L2^2*L3c^2*m3^2*tau2 + 4*L2c^2*L3c^2*m2*m3*tau1 - 4*L2c^2*L3c^2*m2*m3*tau2 - 2*L2^2*L3c^2*m3^2*tau1*cos(2*q3) + 2*L2^2*L3c^2*m3^2*tau2*cos(2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1*L2^3*dq1^2*m3^2*sin(q2) + 4*I3*L1*L2^3*dq2^2*m3^2*sin(q2) + 4*I3*L1*L2c^3*dq1^2*m2^2*sin(q2) + 4*I3*L1*L2c^3*dq2^2*m2^2*sin(q2) - 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 + q3) - 2*I3*L1*L2^2*g*m3^2*cos(q1) - 2*I3*L1*L2c^2*g*m2^2*cos(q1) - 2*I2*L1*L3c^2*g*m3^2*cos(q1) + 2*I2*L1*L3c^2*g*m3^2*cos(q1 + 2*q2 + 2*q3) - 2*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau3*cos(q2) - 4*I2*L1*L3c*m3*tau3*cos(q2 + q3) + 2*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) + 2*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + 2*I3*L1*L2^2*g*m3^2*cos(q1 + 2*q2) + 2*I3*L1*L2c^2*g*m2^2*cos(q1 + 2*q2) + 4*I2*L1*L3c^3*dq1^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq2^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq3^2*m3^2*sin(q2 + q3) - 4*I2*I3*L1*g*m2*cos(q1) - 4*I2*I3*L1*g*m3*cos(q1) - 4*I2*I3*L1c*g*m1*cos(q1) + 2*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau3*cos(q2 + 2*q3) - 4*I3*L1*L2*m3*tau2*cos(q2) + 4*I3*L1*L2*m3*tau3*cos(q2) - 4*I3*L1*L2c*m2*tau2*cos(q2) + 4*I3*L1*L2c*m2*tau3*cos(q2) + 8*I3*L1*L2^3*dq1*dq2*m3^2*sin(q2) + 8*I3*L1*L2c^3*dq1*dq2*m2^2*sin(q2) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 + q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2) + 4*I2*I3*L1*L3c*dq1^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq2^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1) + 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 - q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 - q3) - 4*L1*L2c^2*L3c*m2*m3*tau3*cos(q2 + q3) + 4*I2*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq2^2*m3*sin(q2) + 4*I2*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq2^2*m2*sin(q2) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1 + 2*q2) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 - 2*q3) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 + 2*q3) + 4*L1*L2c^2*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2^2*g*m2*m3*cos(q1) - 4*I3*L2^2*L1c*g*m1*m3*cos(q1) - 4*I2*L1*L3c^2*g*m2*m3*cos(q1) - 4*I3*L1*L2c^2*g*m2*m3*cos(q1) - 4*I3*L1c*L2c^2*g*m1*m2*cos(q1) - 4*I2*L1c*L3c^2*g*m1*m3*cos(q1) - 4*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau3*cos(q2) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1*L2c^3*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L1*L2c^3*L3c^2*dq2^2*m2^2*m3*sin(q2) + 8*I2*L1*L3c^3*dq1*dq2*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq1*dq3*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq2*dq3*m3^2*sin(q2 + q3) + 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq2^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq3^2*m2*m3*sin(q2 + q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2) + 4*I3*L1*L2*L2c^2*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq2^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq2^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq2^2*m2*m3*sin(q2) + 8*I2*I3*L1*L3c*dq1*dq2*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq1*dq3*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq2*dq3*m3*sin(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 + q3) - 4*L1c*L2c^2*L3c^2*g*m1*m2*m3*cos(q1) + 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2 + 2*q3) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 - q3) - 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1) + 8*I2*I3*L1*L2*dq1*dq2*m3*sin(q2) + 8*I2*I3*L1*L2c*dq1*dq2*m2*sin(q2) + 8*L1*L2c^2*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1 + 2*q2) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 - q3) + 8*L1*L2c^3*L3c^2*dq1*dq2*m2^2*m3*sin(q2) + 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 - q3) - 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 8*I3*L1*L2c^2*L3c*dq1*dq2*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq1*dq3*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) - 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) + 8*I3*L1*L2*L2c^2*dq1*dq2*m2*m3*sin(q2) + 8*I3*L1*L2^2*L2c*dq1*dq2*m2*m3*sin(q2) + 8*I2*L1*L2c*L3c^2*dq1*dq2*m2*m3*sin(q2) + 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 - q3) - 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 - q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2))); 
    ddq2 = -(4*I2*I3*tau1 - 4*I1*I3*tau2 + 4*I1*I3*tau3 - 4*I2*I3*tau2 - 4*I3*L1^2*m2*tau2 + 4*I3*L1^2*m2*tau3 - 4*I3*L1^2*m3*tau2 + 4*I3*L2^2*m3*tau1 + 4*I3*L1^2*m3*tau3 - 4*I3*L2^2*m3*tau2 - 4*I3*L1c^2*m1*tau2 + 4*I3*L1c^2*m1*tau3 + 4*I3*L2c^2*m2*tau1 - 4*I1*L3c^2*m3*tau2 + 4*I2*L3c^2*m3*tau1 - 4*I3*L2c^2*m2*tau2 + 4*I1*L3c^2*m3*tau3 - 4*I2*L3c^2*m3*tau2 - 2*L1^2*L3c^2*m3^2*tau2 + 2*L2^2*L3c^2*m3^2*tau1 + 2*L1^2*L3c^2*m3^2*tau3 - 2*L2^2*L3c^2*m3^2*tau2 - 4*L1^2*L3c^2*m2*m3*tau2 + 4*L1^2*L3c^2*m2*m3*tau3 - 4*L1c^2*L3c^2*m1*m3*tau2 + 4*L1c^2*L3c^2*m1*m3*tau3 + 4*L2c^2*L3c^2*m2*m3*tau1 - 4*L2c^2*L3c^2*m2*m3*tau2 - 2*L2^2*L3c^2*m3^2*tau1*cos(2*q3) + 2*L2^2*L3c^2*m3^2*tau2*cos(2*q3) + 2*L1^2*L3c^2*m3^2*tau2*cos(2*q2 + 2*q3) - 2*L1^2*L3c^2*m3^2*tau3*cos(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1*L2^3*dq1^2*m3^2*sin(q2) + 4*I3*L1^3*L2*dq1^2*m3^2*sin(q2) + 4*I3*L1*L2^3*dq2^2*m3^2*sin(q2) + 4*I3*L1*L2c^3*dq1^2*m2^2*sin(q2) + 4*I3*L1^3*L2c*dq1^2*m2^2*sin(q2) + 4*I3*L1*L2c^3*dq2^2*m2^2*sin(q2) - 4*I1*L2*L3c^3*dq1^2*m3^2*sin(q3) - 4*I1*L2*L3c^3*dq2^2*m3^2*sin(q3) - 4*I1*L2*L3c^3*dq3^2*m3^2*sin(q3) - 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2 + 2*q3) + 2*I3*L1^2*L2*g*m3^2*cos(q1 + q2) + 2*I3*L1^2*L2c*g*m2^2*cos(q1 + q2) + 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2) - 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 + q3) - 2*I3*L1*L2^2*g*m3^2*cos(q1) - 2*I3*L1*L2c^2*g*m2^2*cos(q1) - 2*I2*L1*L3c^2*g*m3^2*cos(q1) + 2*I2*L1*L3c^2*g*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I1*I3*L2*g*m3*cos(q1 + q2) + 4*I1*I3*L2c*g*m2*cos(q1 + q2) + 2*L1*L2*L3c^2*m3^2*tau1*cos(q2) - 4*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau3*cos(q2) + 2*L1^2*L2*L3c*m3^2*tau3*cos(q3) - 4*I2*L1*L3c*m3*tau3*cos(q2 + q3) + 4*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) + 2*I3*L1^2*L2^2*dq2^2*m3^2*sin(2*q2) + 4*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) + 2*I3*L1^2*L2c^2*dq2^2*m2^2*sin(2*q2) - 2*I1*L2^2*L3c^2*dq1^2*m3^2*sin(2*q3) - 2*I1*L2^2*L3c^2*dq2^2*m3^2*sin(2*q3) - 2*I3*L1^2*L2*g*m3^2*cos(q1 - q2) + 2*I3*L1*L2^2*g*m3^2*cos(q1 + 2*q2) - 2*I3*L1^2*L2c*g*m2^2*cos(q1 - q2) + 2*I3*L1*L2c^2*g*m2^2*cos(q1 + 2*q2) + 4*I2*L1*L3c^3*dq1^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq2^2*m3^2*sin(q2 + q3) + 4*I2*L1*L3c^3*dq3^2*m3^2*sin(q2 + q3) - 4*I2*I3*L1*g*m2*cos(q1) - 4*I2*I3*L1*g*m3*cos(q1) - 4*I2*I3*L1c*g*m1*cos(q1) - 2*L1*L2*L3c^2*m3^2*tau1*cos(q2 + 2*q3) + 4*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau3*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau3*cos(q2 + 2*q3) - 2*L1^2*L2*L3c*m3^2*tau3*cos(2*q2 + q3) + 4*I3*L1*L2*m3*tau1*cos(q2) - 8*I3*L1*L2*m3*tau2*cos(q2) + 4*I3*L1*L2*m3*tau3*cos(q2) + 4*I3*L1*L2c*m2*tau1*cos(q2) - 8*I3*L1*L2c*m2*tau2*cos(q2) + 4*I3*L1*L2c*m2*tau3*cos(q2) + 4*I1*L2*L3c*m3*tau3*cos(q3) - 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(2*q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) - 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) + 8*I3*L1*L2^3*dq1*dq2*m3^2*sin(q2) + 8*I3*L1*L2c^3*dq1*dq2*m2^2*sin(q2) - 8*I1*L2*L3c^3*dq1*dq2*m3^2*sin(q3) - 8*I1*L2*L3c^3*dq1*dq3*m3^2*sin(q3) - 8*I1*L2*L3c^3*dq2*dq3*m3^2*sin(q3) + 4*I3*L1^3*L2*dq1^2*m2*m3*sin(q2) + 4*I3*L1^3*L2c*dq1^2*m2*m3*sin(q2) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 + q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2) + 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2) + 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 + q2) + 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2) - 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) - 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) - 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(q3) + 4*I2*I3*L1*L3c*dq1^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq2^2*m3*sin(q2 + q3) + 4*I2*I3*L1*L3c*dq3^2*m3*sin(q2 + q3) - 2*L1*L2^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1) - 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) + 2*L1*L2c^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 2*I3*L1^2*L2*g*m2*m3*cos(q1 + q2) + 4*I3*L2*L1c^2*g*m1*m3*cos(q1 + q2) + 2*I3*L1^2*L2c*g*m2*m3*cos(q1 + q2) + 4*I3*L1c^2*L2c*g*m1*m2*cos(q1 + q2) + 4*I1*L2c*L3c^2*g*m2*m3*cos(q1 + q2) - 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I2*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I2*L1*L2*L3c^2*dq2^2*m3^2*sin(q2 + 2*q3) + 2*I3*L1*L2^2*L3c*dq2^2*m3^2*sin(q2 - q3) + 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) + 2*I3*L1*L2^2*L3c*dq3^2*m3^2*sin(q2 - q3) + 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) + 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(2*q2 + q3) - 4*L1*L2c^2*L3c*m2*m3*tau3*cos(q2 + q3) + 4*I1*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq1^2*m3*sin(q2) + 4*I2*I3*L1*L2*dq2^2*m3*sin(q2) + 4*I1*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I2*I3*L1*L2c*dq2^2*m2*sin(q2) - 4*I1*I3*L2*L3c*dq1^2*m3*sin(q3) - 4*I1*I3*L2*L3c*dq2^2*m3*sin(q3) - 4*I1*I3*L2*L3c*dq3^2*m3*sin(q3) - 2*L1^2*L2^2*L3c^2*dq1^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2^2*L3c^2*dq2^2*m2*m3^2*sin(2*q3) + 4*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) - 2*L2^2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(2*q3) + 2*L1^2*L2c^2*L3c^2*dq2^2*m2^2*m3*sin(2*q2) - 2*L2^2*L1c^2*L3c^2*dq2^2*m1*m3^2*sin(2*q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 - q2) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) + L1*L2^2*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 - q2) - 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 - q2) + 2*L1*L2c^2*L3c^2*g*m2^2*m3*cos(q1 + 2*q2) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 - 2*q3) + L2^2*L1c*L3c^2*g*m1*m3^2*cos(q1 + 2*q3) + 4*L1*L2c^2*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) + 4*L1*L2c^2*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) - 4*I3*L1*L2^2*g*m2*m3*cos(q1) - 4*I3*L2^2*L1c*g*m1*m3*cos(q1) - 4*I2*L1*L3c^2*g*m2*m3*cos(q1) - 4*I3*L1*L2c^2*g*m2*m3*cos(q1) - 4*I3*L1c*L2c^2*g*m1*m2*cos(q1) - 4*I2*L1c*L3c^2*g*m1*m3*cos(q1) + 4*L1^2*L2*L3c*m2*m3*tau3*cos(q3) + 4*L1*L2c*L3c^2*m2*m3*tau1*cos(q2) - 8*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau3*cos(q2) + 4*L2*L1c^2*L3c*m1*m3*tau3*cos(q3) - 2*L1^2*L2c*L3c*m2*m3*tau3*cos(q3) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*I3*L1^2*L2^2*dq1*dq2*m3^2*sin(2*q2) + 4*I3*L1^2*L2c^2*dq1*dq2*m2^2*sin(2*q2) - 4*I1*L2^2*L3c^2*dq1*dq2*m3^2*sin(2*q3) + 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2) - 4*L1^2*L2*L3c^3*dq1^2*m2*m3^2*sin(q3) - 4*L1^2*L2*L3c^3*dq2^2*m2*m3^2*sin(q3) - 4*L1^2*L2*L3c^3*dq3^2*m2*m3^2*sin(q3) + 4*L1*L2c^3*L3c^2*dq1^2*m2^2*m3*sin(q2) - 4*L2*L1c^2*L3c^3*dq1^2*m1*m3^2*sin(q3) + 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 4*L1^3*L2c*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L1*L2c^3*L3c^2*dq2^2*m2^2*m3*sin(q2) - 4*L2*L1c^2*L3c^3*dq2^2*m1*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q3) - 4*L2*L1c^2*L3c^3*dq3^2*m1*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q3) + 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q3) - 2*I3*L1^2*L2*g*m2*m3*cos(q1 - q2) - 2*I3*L1^2*L2c*g*m2*m3*cos(q1 - q2) + 8*I2*L1*L3c^3*dq1*dq2*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq1*dq3*m3^2*sin(q2 + q3) + 8*I2*L1*L3c^3*dq2*dq3*m3^2*sin(q2 + q3) - 2*L1^2*L2c*L3c*m2*m3*tau3*cos(2*q2 + q3) + 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 - q3) + 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 - q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(2*q2 + q3) + 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2) + 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq2^2*m2*m3*sin(q2 + q3) + 4*I3*L1*L2c^2*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1c^2*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) - 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1*L2^2*L2c*L3c^2*dq2^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q2 - q1 + 2*q3) - 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2) - 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) - 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(q3) - 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(q3) + 4*I3*L1*L2*L1c^2*dq1^2*m1*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq1^2*m2*m3*sin(q2) + 4*I3*L1*L2*L2c^2*dq2^2*m2*m3*sin(q2) + 4*I3*L1*L2^2*L2c*dq2^2*m2*m3*sin(q2) - 4*I3*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) - 4*I3*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) - 4*I3*L1^2*L2*L3c*dq3^2*m2*m3*sin(q3) + 4*I3*L1*L1c^2*L2c*dq1^2*m1*m2*sin(q2) + 4*I1*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L1*L2c*L3c^2*dq2^2*m2*m3*sin(q2) - 4*I3*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) - 4*I3*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) - 4*I3*L2*L1c^2*L3c*dq3^2*m1*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) + 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(q3) - 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 + q2) - 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 + q2) + 8*I2*I3*L1*L3c*dq1*dq2*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq1*dq3*m3*sin(q2 + q3) + 8*I2*I3*L1*L3c*dq2*dq3*m3*sin(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 + q3) - 4*L1c*L2c^2*L3c^2*g*m1*m2*m3*cos(q1) + 4*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) + 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q3) + 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2) + 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 - q2) + 2*L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q2) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 - 2*q3) - L1*L2*L2c*L3c^2*g*m2*m3^2*cos(q1 + 2*q3) + 4*I2*L1*L2*L3c^2*dq1*dq2*m3^2*sin(q2 + 2*q3) + 4*I3*L1*L2^2*L3c*dq1*dq2*m3^2*sin(q2 - q3) + 4*I3*L1*L2^2*L3c*dq1*dq3*m3^2*sin(q2 - q3) + 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) + 4*I3*L1*L2^2*L3c*dq2*dq3*m3^2*sin(q2 - q3) + 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(2*q2 + q3) + 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) + 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(2*q2 + q3) - 2*L1*L2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q2 + q3) - 2*L1*L2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q2 + q3) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1) + 8*I2*I3*L1*L2*dq1*dq2*m3*sin(q2) + 8*I2*I3*L1*L2c*dq1*dq2*m2*sin(q2) - 8*I1*I3*L2*L3c*dq1*dq2*m3*sin(q3) - 8*I1*I3*L2*L3c*dq1*dq3*m3*sin(q3) - 8*I1*I3*L2*L3c*dq2*dq3*m3*sin(q3) - 4*L1^2*L2^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) + 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2^2*m3*sin(2*q2) - 4*L2^2*L1c^2*L3c^2*dq1*dq2*m1*m3^2*sin(2*q3) + 8*L1*L2c^2*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) + 8*L1*L2c^2*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 8*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) + 4*I3*L1^2*L2*L2c*dq2^2*m2*m3*sin(2*q2) - 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 - q2) + 4*I3*L1*L2*L2c*g*m2*m3*cos(q1 + 2*q2) - 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 - q2) + 2*L1*L2*L2c*L3c*m2*m3*tau3*cos(q2 - q3) - 8*L1^2*L2*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 8*L1^2*L2*L3c^3*dq1*dq3*m2*m3^2*sin(q3) - 8*L1^2*L2*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 8*L1*L2c^3*L3c^2*dq1*dq2*m2^2*m3*sin(q2) - 8*L2*L1c^2*L3c^3*dq1*dq2*m1*m3^2*sin(q3) - 8*L2*L1c^2*L3c^3*dq1*dq3*m1*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 8*L2*L1c^2*L3c^3*dq2*dq3*m1*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q3) + 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 - q3) + 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 - q3) - 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 + q3) - 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2) + 4*L1*L1c^2*L2c*L3c^2*dq1^2*m1*m2*m3*sin(q2) - 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) + 8*I3*L1*L2c^2*L3c*dq1*dq2*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq1*dq3*m2*m3*sin(q2 + q3) + 8*I3*L1*L2c^2*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*L1*L2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) - 4*L1*L2^2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(q2 + 2*q3) + 8*I3*L1*L2*L2c^2*dq1*dq2*m2*m3*sin(q2) + 8*I3*L1*L2^2*L2c*dq1*dq2*m2*m3*sin(q2) - 8*I3*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) - 8*I3*L1^2*L2*L3c*dq1*dq3*m2*m3*sin(q3) - 8*I3*L1^2*L2*L3c*dq2*dq3*m2*m3*sin(q3) + 8*I2*L1*L2c*L3c^2*dq1*dq2*m2*m3*sin(q2) - 8*I3*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) - 8*I3*L2*L1c^2*L3c*dq1*dq3*m1*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) - 8*I3*L2*L1c^2*L3c*dq2*dq3*m1*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(q3) + 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(q3) + 2*I3*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq2^2*m2*m3*sin(q2 - q3) + 2*I3*L1*L2*L2c*L3c*dq3^2*m2*m3*sin(q2 - q3) + 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2) + 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) - 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 - q2) + 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) + 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(2*q2 + q3) + 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(2*q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q2 + q3) - 4*L1*L2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q2 + q3) + 8*I3*L1^2*L2*L2c*dq1*dq2*m2*m3*sin(2*q2) - 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 + q3) - 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 + q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq2*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq1*dq3*m2*m3*sin(q2 - q3) + 4*I3*L1*L2*L2c*L3c*dq2*dq3*m2*m3*sin(q2 - q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2))); 
    ddq3 = -(4*I1*I3*tau2 - 4*I1*I2*tau3 - 4*I1*I3*tau3 - 4*I2*L1^2*m2*tau3 + 4*I3*L1^2*m2*tau2 - 4*I1*L2^2*m3*tau3 - 4*I2*L1^2*m3*tau3 - 4*I3*L1^2*m2*tau3 + 4*I3*L1^2*m3*tau2 - 4*I3*L1^2*m3*tau3 - 4*I2*L1c^2*m1*tau3 + 4*I3*L1c^2*m1*tau2 - 4*I1*L2c^2*m2*tau3 - 4*I3*L1c^2*m1*tau3 + 4*I1*L3c^2*m3*tau2 - 4*I1*L3c^2*m3*tau3 - 2*L1^2*L2^2*m3^2*tau3 - 2*L1^2*L2c^2*m2^2*tau3 + 2*L1^2*L3c^2*m3^2*tau2 - 2*L1^2*L3c^2*m3^2*tau3 - 4*L1^2*L2^2*m2*m3*tau3 - 4*L2^2*L1c^2*m1*m3*tau3 - 4*L1^2*L2c^2*m2*m3*tau3 + 4*L1^2*L3c^2*m2*m3*tau2 - 4*L1^2*L3c^2*m2*m3*tau3 - 4*L1c^2*L2c^2*m1*m2*tau3 + 4*L1c^2*L3c^2*m1*m3*tau2 - 4*L1c^2*L3c^2*m1*m3*tau3 + 2*L1^2*L2^2*m3^2*tau3*cos(2*q2) + 2*L1^2*L2c^2*m2^2*tau3*cos(2*q2) - 2*L1^2*L3c^2*m3^2*tau2*cos(2*q2 + 2*q3) + 2*L1^2*L3c^2*m3^2*tau3*cos(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq1^2*m3^2*sin(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq2^2*m3^2*sin(2*q2 + 2*q3) + 2*I2*L1^2*L3c^2*dq3^2*m3^2*sin(2*q2 + 2*q3) - 4*I3*L1^3*L2*dq1^2*m3^2*sin(q2) - 4*I3*L1^3*L2c*dq1^2*m2^2*sin(q2) + 4*I1*L2*L3c^3*dq1^2*m3^2*sin(q3) + 4*I1*L2^3*L3c*dq1^2*m3^2*sin(q3) + 4*I1*L2*L3c^3*dq2^2*m3^2*sin(q3) + 4*I1*L2^3*L3c*dq2^2*m3^2*sin(q3) + 4*I1*L2*L3c^3*dq3^2*m3^2*sin(q3) + 4*I1*I2*L3c*g*m3*cos(q1 + q2 + q3) - 2*I1*L2^2*L3c*g*m3^2*cos(q1 + q2 - q3) - 2*I2*L1^2*L3c*g*m3^2*cos(q2 - q1 + q3) + 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2 + 2*q3) - 2*I3*L1^2*L2*g*m3^2*cos(q1 + q2) - 2*I3*L1^2*L2c*g*m2^2*cos(q1 + q2) - 2*I1*L2*L3c^2*g*m3^2*cos(q1 + q2) + 2*L1*L2^2*L3c*m3^2*tau1*cos(q2 + q3) - 2*L1*L2^2*L3c*m3^2*tau2*cos(q2 + q3) - 4*I1*I3*L2*g*m3*cos(q1 + q2) - 4*I1*I3*L2c*g*m2*cos(q1 + q2) - 2*L1*L2*L3c^2*m3^2*tau1*cos(q2) + 2*L1*L2*L3c^2*m3^2*tau2*cos(q2) + 2*L1^2*L2*L3c*m3^2*tau2*cos(q3) - 4*L1^2*L2*L3c*m3^2*tau3*cos(q3) + 4*I2*L1*L3c*m3*tau1*cos(q2 + q3) - 4*I2*L1*L3c*m3*tau2*cos(q2 + q3) - 2*I3*L1^2*L2^2*dq1^2*m3^2*sin(2*q2) - 2*I3*L1^2*L2^2*dq2^2*m3^2*sin(2*q2) - 2*I3*L1^2*L2c^2*dq1^2*m2^2*sin(2*q2) - 2*I3*L1^2*L2c^2*dq2^2*m2^2*sin(2*q2) + 4*I1*L2^2*L3c^2*dq1^2*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq2^2*m3^2*sin(2*q3) + 2*I1*L2^2*L3c^2*dq3^2*m3^2*sin(2*q3) + 2*I3*L1^2*L2*g*m3^2*cos(q1 - q2) + 2*I3*L1^2*L2c*g*m2^2*cos(q1 - q2) + 4*I2*L1^3*L3c*dq1^2*m3^2*sin(q2 + q3) - 2*L1*L2^2*L3c*m3^2*tau1*cos(q2 - q3) + 2*L1*L2*L3c^2*m3^2*tau1*cos(q2 + 2*q3) + 2*L1*L2^2*L3c*m3^2*tau2*cos(q2 - q3) - 2*L1*L2*L3c^2*m3^2*tau2*cos(q2 + 2*q3) - 2*L1^2*L2*L3c*m3^2*tau2*cos(2*q2 + q3) + 4*L1^2*L2*L3c*m3^2*tau3*cos(2*q2 + q3) + 2*I1*L2^2*L3c*g*m3^2*cos(q1 + q2 + q3) + 2*I2*L1^2*L3c*g*m3^2*cos(q1 + q2 + q3) + 4*L1^2*L2*L2c*m2*m3*tau3 - 4*I3*L1*L2*m3*tau1*cos(q2) + 4*I3*L1*L2*m3*tau2*cos(q2) - 4*I3*L1*L2c*m2*tau1*cos(q2) + 4*I3*L1*L2c*m2*tau2*cos(q2) + 4*I1*L2*L3c*m3*tau2*cos(q3) - 8*I1*L2*L3c*m3*tau3*cos(q3) + 4*I2*L1^2*L3c^2*dq1*dq2*m3^2*sin(2*q2 + 2*q3) + 4*I2*L1^2*L3c^2*dq1*dq3*m3^2*sin(2*q2 + 2*q3) + 4*I2*L1^2*L3c^2*dq2*dq3*m3^2*sin(2*q2 + 2*q3) - 2*L1^3*L2^2*L3c*dq1^2*m2*m3^2*sin(q2 - q3) + 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) + 2*L1^3*L2c^2*L3c*dq1^2*m2^2*m3*sin(q2 - q3) - 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c^3*L3c*dq1^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2 + 2*q3) - 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2c^3*L3c*dq2^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(2*q2 + q3) - L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 + q2 - q3) + L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 - q2 + q3) - L1^2*L2^2*L3c*g*m2*m3^2*cos(q2 - q1 + q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) - 2*L2^2*L1c^2*L3c*g*m1*m3^2*cos(q1 + q2 - q3) + 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) - 2*L1^2*L2c^2*L3c*g*m2*m3^2*cos(q2 - q1 + q3) + L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 + q2 - q3) - L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 - q2 + q3) - L1^2*L2c^2*L3c*g*m2^2*m3*cos(q2 - q1 + q3) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2 + 2*q3) + 8*I1*L2*L3c^3*dq1*dq2*m3^2*sin(q3) + 8*I1*L2^3*L3c*dq1*dq2*m3^2*sin(q3) + 8*I1*L2*L3c^3*dq1*dq3*m3^2*sin(q3) + 8*I1*L2*L3c^3*dq2*dq3*m3^2*sin(q3) - 4*I3*L1^3*L2*dq1^2*m2*m3*sin(q2) - 4*I3*L1^3*L2c*dq1^2*m2*m3*sin(q2) + 4*L1^2*L2*L2c*m2*m3*tau3*cos(2*q2) + 2*I1*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 + q2) - 2*L2*L1c^2*L3c^2*g*m1*m3^2*cos(q1 + q2) - L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 + q2) - 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 + q2) - 2*I2*L1^2*L3c*g*m2*m3*cos(q2 - q1 + q3) - 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2) + 2*I2*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) + 2*I2*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(q3) + 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(q3) + 4*I1*I2*L1*L3c*dq1^2*m3*sin(q2 + q3) - L1^2*L2*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q2 - q1 + 2*q3) - 2*I3*L1^2*L2*g*m2*m3*cos(q1 + q2) - 4*I3*L2*L1c^2*g*m1*m3*cos(q1 + q2) - 2*I3*L1^2*L2c*g*m2*m3*cos(q1 + q2) - 4*I3*L1c^2*L2c*g*m1*m2*cos(q1 + q2) - 4*I1*L2c*L3c^2*g*m2*m3*cos(q1 + q2) - 2*I1*L1*L2^2*L3c*dq1^2*m3^2*sin(q2 - q3) + 2*I1*L1*L2*L3c^2*dq1^2*m3^2*sin(q2 + 2*q3) + 2*I2*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) + 2*I2*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq1^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq2^2*m3^2*sin(2*q2 + q3) - 2*I3*L1^2*L2*L3c*dq3^2*m3^2*sin(2*q2 + q3) + 4*L1*L2c^2*L3c*m2*m3*tau1*cos(q2 + q3) - 4*L1*L2c^2*L3c*m2*m3*tau2*cos(q2 + q3) - 4*I1*I3*L1*L2*dq1^2*m3*sin(q2) - 4*I1*I3*L1*L2c*dq1^2*m2*sin(q2) + 4*I1*I2*L2*L3c*dq1^2*m3*sin(q3) + 4*I1*I2*L2*L3c*dq2^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq1^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq2^2*m3*sin(q3) + 4*I1*I3*L2*L3c*dq3^2*m3*sin(q3) + 4*L1^2*L2^2*L3c^2*dq1^2*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq2^2*m2*m3^2*sin(2*q3) + 2*L1^2*L2^2*L3c^2*dq3^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2c^2*L3c^2*dq1^2*m2^2*m3*sin(2*q2) + 4*L2^2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(2*q3) - 2*L1^2*L2c^2*L3c^2*dq2^2*m2^2*m3*sin(2*q2) + 4*L2^2*L1c^2*L3c^2*dq2^2*m1*m3^2*sin(2*q3) + 2*L2^2*L1c^2*L3c^2*dq3^2*m1*m3^2*sin(2*q3) + L1^2*L2*L3c^2*g*m2*m3^2*cos(q1 - q2) + L1^2*L2c*L3c^2*g*m2*m3^2*cos(q1 - q2) + 2*L1^2*L2c*L3c^2*g*m2^2*m3*cos(q1 - q2) + 2*L1^3*L2^2*L3c*dq1^2*m2*m3^2*sin(q2 + q3) + 4*L1^3*L2c^2*L3c*dq1^2*m2*m3^2*sin(q2 + q3) + 2*L1^3*L2c^2*L3c*dq1^2*m2^2*m3*sin(q2 + q3) + L1^2*L2^2*L3c*g*m2*m3^2*cos(q1 + q2 + q3) + 2*L2^2*L1c^2*L3c*g*m1*m3^2*cos(q1 + q2 + q3) + 2*L1^2*L2c^2*L3c*g*m2*m3^2*cos(q1 + q2 + q3) + L1^2*L2c^2*L3c*g*m2^2*m3*cos(q1 + q2 + q3) + 4*L1^2*L2*L3c*m2*m3*tau2*cos(q3) - 8*L1^2*L2*L3c*m2*m3*tau3*cos(q3) - 4*L1*L2c*L3c^2*m2*m3*tau1*cos(q2) + 4*L1*L2c*L3c^2*m2*m3*tau2*cos(q2) + 4*L2*L1c^2*L3c*m1*m3*tau2*cos(q3) - 8*L2*L1c^2*L3c*m1*m3*tau3*cos(q3) - 2*L1^2*L2c*L3c*m2*m3*tau2*cos(q3) + 4*L1^2*L2c*L3c*m2*m3*tau3*cos(q3) + 2*L1^2*L2c^2*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) + 2*L1^2*L2c^2*L3c^2*dq2^2*m2*m3^2*sin(2*q2 + 2*q3) + 2*L1^2*L2c^2*L3c^2*dq3^2*m2*m3^2*sin(2*q2 + 2*q3) - 4*I3*L1^2*L2^2*dq1*dq2*m3^2*sin(2*q2) - 4*I3*L1^2*L2c^2*dq1*dq2*m2^2*sin(2*q2) + 8*I1*L2^2*L3c^2*dq1*dq2*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq1*dq3*m3^2*sin(2*q3) + 4*I1*L2^2*L3c^2*dq2*dq3*m3^2*sin(2*q3) - 2*L1^3*L2*L3c^2*dq1^2*m2*m3^2*sin(q2) + 4*L1^2*L2*L3c^3*dq1^2*m2*m3^2*sin(q3) + 4*L1^2*L2^3*L3c*dq1^2*m2*m3^2*sin(q3) + 4*L1^2*L2*L3c^3*dq2^2*m2*m3^2*sin(q3) + 4*L1^2*L2^3*L3c*dq2^2*m2*m3^2*sin(q3) + 4*L1^2*L2*L3c^3*dq3^2*m2*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq1^2*m1*m3^2*sin(q3) - 2*L1^3*L2c*L3c^2*dq1^2*m2*m3^2*sin(q2) - 4*L1^3*L2c*L3c^2*dq1^2*m2^2*m3*sin(q2) + 4*L2^3*L1c^2*L3c*dq1^2*m1*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq2^2*m1*m3^2*sin(q3) - 2*L1^2*L2c*L3c^3*dq1^2*m2*m3^2*sin(q3) - 2*L1^2*L2c^3*L3c*dq1^2*m2^2*m3*sin(q3) + 4*L2^3*L1c^2*L3c*dq2^2*m1*m3^2*sin(q3) + 4*L2*L1c^2*L3c^3*dq3^2*m1*m3^2*sin(q3) - 2*L1^2*L2c*L3c^3*dq2^2*m2*m3^2*sin(q3) - 2*L1^2*L2c^3*L3c*dq2^2*m2^2*m3*sin(q3) - 2*L1^2*L2c*L3c^3*dq3^2*m2*m3^2*sin(q3) + 2*I3*L1^2*L2*g*m2*m3*cos(q1 - q2) + 2*I3*L1^2*L2c*g*m2*m3*cos(q1 - q2) + 4*I2*L1^3*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*L1^2*L2c*L3c*m2*m3*tau2*cos(2*q2 + q3) + 4*L1^2*L2c*L3c*m2*m3*tau3*cos(2*q2 + q3) + 2*I2*L1^2*L3c*g*m2*m3*cos(q1 + q2 + q3) + 4*I2*L1c^2*L3c*g*m1*m3*cos(q1 + q2 + q3) + 4*I1*L2c^2*L3c*g*m2*m3*cos(q1 + q2 + q3) + 2*L1^3*L2*L2c*L3c*dq1^2*m2*m3^2*sin(q2 - q3) - 2*L1^3*L2*L2c*L3c*dq1^2*m2^2*m3*sin(q2 - q3) + 2*L1*L2^2*L1c^2*L3c*dq1^2*m1*m3^2*sin(q2 + q3) + L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 + q2 - q3) + L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 - q2 + q3) - L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q2 - q1 + q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2 + 2*q3) + L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 + q2 - q3) - L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 - q2 + q3) + 3*L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q2 - q1 + q3) - L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 + q2 - q3) + L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 - q2 + q3) + L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q2 - q1 + q3) - 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 4*L1^2*L2c^3*L3c*dq1*dq2*m2^2*m3*sin(2*q2 + q3) - 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(2*q2 + q3) - 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(2*q2 + q3) - 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2^2*m3*sin(q3) - 6*L1^2*L2^2*L2c*L3c*dq1^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2*m3^2*sin(q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2^2*m3*sin(q3) - 6*L1^2*L2^2*L2c*L3c*dq2^2*m2*m3^2*sin(q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 + q2) + 4*I2*L1*L1c^2*L3c*dq1^2*m1*m3*sin(q2 + q3) + 4*I1*L1*L2c^2*L3c*dq1^2*m2*m3*sin(q2 + q3) - 2*I2*L1*L1c*L3c*g*m1*m3*cos(q2 - q1 + q3) - 2*I1*L2*L2c*L3c*g*m2*m3*cos(q1 + q2 - q3) - 4*L1c^2*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) - 2*L1*L2^2*L1c^2*L3c*dq1^2*m1*m3^2*sin(q2 - q3) + 2*L1*L2*L1c^2*L3c^2*dq1^2*m1*m3^2*sin(q2 + 2*q3) + 2*L1^2*L2*L2c^2*L3c*dq1^2*m2*m3^2*sin(2*q2 + q3) - 2*L1^2*L2*L2c^2*L3c*dq1^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2^2*L2c*L3c*dq1^2*m2*m3^2*sin(2*q2 + q3) + 2*L1^2*L2*L2c^2*L3c*dq2^2*m2*m3^2*sin(2*q2 + q3) - 2*L1^2*L2*L2c^2*L3c*dq2^2*m2^2*m3*sin(2*q2 + q3) - 2*L1^2*L2^2*L2c*L3c*dq2^2*m2*m3^2*sin(2*q2 + q3) - L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q2 - q1 + 2*q3) + 4*I2*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(q3) + 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(q3) - 4*I3*L1*L2*L1c^2*dq1^2*m1*m3*sin(q2) + 4*I2*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) + 4*I2*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L1^2*L2*L3c*dq3^2*m2*m3*sin(q3) - 4*I3*L1*L1c^2*L2c*dq1^2*m1*m2*sin(q2) - 4*I1*L1*L2c*L3c^2*dq1^2*m2*m3*sin(q2) + 4*I2*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) + 4*I1*L2*L2c^2*L3c*dq1^2*m2*m3*sin(q3) + 4*I2*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) - 2*I2*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq1^2*m1*m3*sin(q3) + 4*I1*L2*L2c^2*L3c*dq2^2*m2*m3*sin(q3) - 2*I2*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq2^2*m1*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(q3) + 4*I3*L2*L1c^2*L3c*dq3^2*m1*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(q3) - 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(q3) + 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 + q2) + 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 + q2) - 2*L1*L2*L2c*L3c*m2*m3*tau1*cos(q2 + q3) + 2*L1*L2*L2c*L3c*m2*m3*tau2*cos(q2 + q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2) - 4*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2) - 4*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq3^2*m2*m3^2*sin(2*q3) + L1*L2*L1c*L3c^2*g*m1*m3^2*cos(q1 - q2) + 4*I2*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq1*dq2*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq1*dq3*m3^2*sin(2*q2 + q3) - 4*I3*L1^2*L2*L3c*dq2*dq3*m3^2*sin(2*q2 + q3) + 2*I2*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) + 2*I2*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq1^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq2^2*m2*m3*sin(2*q2 + q3) - 2*I3*L1^2*L2c*L3c*dq3^2*m2*m3*sin(2*q2 + q3) - 6*L1^3*L2*L2c*L3c*dq1^2*m2*m3^2*sin(q2 + q3) - 2*L1^3*L2*L2c*L3c*dq1^2*m2^2*m3*sin(q2 + q3) + 8*I1*I2*L2*L3c*dq1*dq2*m3*sin(q3) + 8*I1*I3*L2*L3c*dq1*dq2*m3*sin(q3) + 8*I1*I3*L2*L3c*dq1*dq3*m3*sin(q3) + 8*I1*I3*L2*L3c*dq2*dq3*m3*sin(q3) - L1*L2^2*L1c*L3c*g*m1*m3^2*cos(q1 + q2 + q3) - 3*L1^2*L2*L2c*L3c*g*m2*m3^2*cos(q1 + q2 + q3) - L1^2*L2*L2c*L3c*g*m2^2*m3*cos(q1 + q2 + q3) + 8*L1^2*L2^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq1*dq3*m2*m3^2*sin(2*q3) + 4*L1^2*L2^2*L3c^2*dq2*dq3*m2*m3^2*sin(2*q3) - 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2^2*m3*sin(2*q2) + 8*L2^2*L1c^2*L3c^2*dq1*dq2*m1*m3^2*sin(2*q3) + 4*L2^2*L1c^2*L3c^2*dq1*dq3*m1*m3^2*sin(2*q3) + 4*L2^2*L1c^2*L3c^2*dq2*dq3*m1*m3^2*sin(2*q3) - 2*L1^2*L2*L2c*L3c^2*dq1^2*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1^2*L2*L2c*L3c^2*dq2^2*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1^2*L2*L2c*L3c^2*dq3^2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1c^2*L2c^2*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 4*I3*L1^2*L2*L2c*dq1^2*m2*m3*sin(2*q2) - 4*I3*L1^2*L2*L2c*dq2^2*m2*m3*sin(2*q2) + 2*I3*L1*L2*L1c*g*m1*m3*cos(q1 - q2) + 2*I3*L1*L1c*L2c*g*m1*m2*cos(q1 - q2) + 4*L1^2*L2c^2*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1^2*L2c^2*L3c^2*dq1*dq3*m2*m3^2*sin(2*q2 + 2*q3) + 4*L1^2*L2c^2*L3c^2*dq2*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 2*L1*L2*L2c*L3c*m2*m3*tau1*cos(q2 - q3) + 2*L1*L2*L2c*L3c*m2*m3*tau2*cos(q2 - q3) - 2*I2*L1*L1c*L3c*g*m1*m3*cos(q1 + q2 + q3) - 2*I1*L2*L2c*L3c*g*m2*m3*cos(q1 + q2 + q3) + 8*L1^2*L2*L3c^3*dq1*dq2*m2*m3^2*sin(q3) + 8*L1^2*L2^3*L3c*dq1*dq2*m2*m3^2*sin(q3) + 8*L1^2*L2*L3c^3*dq1*dq3*m2*m3^2*sin(q3) + 8*L1^2*L2*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 8*L2*L1c^2*L3c^3*dq1*dq2*m1*m3^2*sin(q3) + 8*L2^3*L1c^2*L3c*dq1*dq2*m1*m3^2*sin(q3) + 8*L2*L1c^2*L3c^3*dq1*dq3*m1*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq1*dq2*m2*m3^2*sin(q3) - 4*L1^2*L2c^3*L3c*dq1*dq2*m2^2*m3*sin(q3) + 8*L2*L1c^2*L3c^3*dq2*dq3*m1*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq1*dq3*m2*m3^2*sin(q3) - 4*L1^2*L2c*L3c^3*dq2*dq3*m2*m3^2*sin(q3) + 4*L1*L1c^2*L2c^2*L3c*dq1^2*m1*m2*m3*sin(q2 + q3) - 2*L1*L1c*L2c^2*L3c*g*m1*m2*m3*cos(q2 - q1 + q3) - 2*L2*L1c^2*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 - q3) - 2*I1*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 + q3) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2*m3^2*sin(q3) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2^2*m3*sin(q3) - 12*L1^2*L2^2*L2c*L3c*dq1*dq2*m2*m3^2*sin(q3) - 4*L1*L1c^2*L2c*L3c^2*dq1^2*m1*m2*m3*sin(q2) + 4*L2*L1c^2*L2c^2*L3c*dq1^2*m1*m2*m3*sin(q3) + 4*L2*L1c^2*L2c^2*L3c*dq2^2*m1*m2*m3*sin(q3) + 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 + q2) + 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2*m3^2*sin(2*q2 + q3) - 4*L1^2*L2*L2c^2*L3c*dq1*dq2*m2^2*m3*sin(2*q2 + q3) - 4*L1^2*L2^2*L2c*L3c*dq1*dq2*m2*m3^2*sin(2*q2 + q3) + 8*I2*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq1*dq3*m2*m3*sin(q3) + 8*I3*L1^2*L2*L3c*dq2*dq3*m2*m3*sin(q3) + 8*I2*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) + 8*I1*L2*L2c^2*L3c*dq1*dq2*m2*m3*sin(q3) - 4*I2*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq1*dq2*m1*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq1*dq3*m1*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(q3) + 8*I3*L2*L1c^2*L3c*dq2*dq3*m1*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(q3) - 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(q3) - 2*I1*L1*L2*L2c*L3c*dq1^2*m2*m3*sin(q2 - q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2) - 8*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq3*m2*m3^2*sin(2*q3) - 4*L1^2*L2*L2c*L3c^2*dq2*dq3*m2*m3^2*sin(2*q3) + 2*L1*L1c*L2c*L3c^2*g*m1*m2*m3*cos(q1 - q2) + 4*I2*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq1*dq2*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq1*dq3*m2*m3*sin(2*q2 + q3) - 4*I3*L1^2*L2c*L3c*dq2*dq3*m2*m3*sin(2*q2 + q3) - 2*L1*L1c*L2c^2*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 2*L2*L1c^2*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq2*m2*m3^2*sin(2*q2 + 2*q3) - 4*L1^2*L2*L2c*L3c^2*dq1*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 4*L1^2*L2*L2c*L3c^2*dq2*dq3*m2*m3^2*sin(2*q2 + 2*q3) - 8*I3*L1^2*L2*L2c*dq1*dq2*m2*m3*sin(2*q2) + 8*L2*L1c^2*L2c^2*L3c*dq1*dq2*m1*m2*m3*sin(q3) - 2*L1*L2*L1c^2*L2c*L3c*dq1^2*m1*m2*m3*sin(q2 - q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 + q3) - 2*L1*L2*L1c^2*L2c*L3c*dq1^2*m1*m2*m3*sin(q2 + q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 + q2 - q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q1 - q2 + q3) + L1*L2*L1c*L2c*L3c*g*m1*m2*m3*cos(q2 - q1 + q3))/(2*(2*I1*I2*I3 + 2*I2*I3*L1^2*m2 + 2*I1*I3*L2^2*m3 + 2*I2*I3*L1^2*m3 + 2*I2*I3*L1c^2*m1 + 2*I1*I3*L2c^2*m2 + 2*I1*I2*L3c^2*m3 + I3*L1^2*L2^2*m3^2 + I3*L1^2*L2c^2*m2^2 + I1*L2^2*L3c^2*m3^2 + I2*L1^2*L3c^2*m3^2 - I2*L1^2*L3c^2*m3^2*cos(2*q2 + 2*q3) + L1^2*L2^2*L3c^2*m2*m3^2 + L2^2*L1c^2*L3c^2*m1*m3^2 + L1^2*L2c^2*L3c^2*m2*m3^2 + L1^2*L2c^2*L3c^2*m2^2*m3 + 2*I3*L1^2*L2^2*m2*m3 + 2*I3*L2^2*L1c^2*m1*m3 + 2*I2*L1^2*L3c^2*m2*m3 + 2*I3*L1^2*L2c^2*m2*m3 + 2*I3*L1c^2*L2c^2*m1*m2 + 2*I2*L1c^2*L3c^2*m1*m3 + 2*I1*L2c^2*L3c^2*m2*m3 - I3*L1^2*L2^2*m3^2*cos(2*q2) - I3*L1^2*L2c^2*m2^2*cos(2*q2) - I1*L2^2*L3c^2*m3^2*cos(2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2 + 2*L1c^2*L2c^2*L3c^2*m1*m2*m3 - 2*I3*L1^2*L2*L2c*m2*m3 - L1^2*L2^2*L3c^2*m2*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2^2*m3*cos(2*q2) - L2^2*L1c^2*L3c^2*m1*m3^2*cos(2*q3) - L1^2*L2c^2*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q3) + L1^2*L2*L2c*L3c^2*m2*m3^2*cos(2*q2 + 2*q3) - 2*I3*L1^2*L2*L2c*m2*m3*cos(2*q2)));
    ddth = [ddq1 ddq2 ddq3]';
    
    % Numerical integration
    dth = dth + ddth*dt;
    th = th + dth*dt;
    
    ddth_out(i,:) = ddth;
    dth_out(i,:) = dth;
    th_out(i,:) = th;
    u_out(i,:) = u;
    P_out(i,:) = P;
    alpha_out(i,:) = alpha;
    P_ae_out(i,:) = P_ae;
    F_ex_out(i,:) = F_ex;
    dP_out(i,:) = dP;
    
end

figure(1)
subplot(2,1,1)
h1 = plot(time, P_out(:,1),'k','linewidth',2);
hold on
h2 = plot(time, P_out(:,2),'r','linewidth',2);
h3 = plot(time, ones(length(time),1)*P_d(1),'k:','linewidth',1);
h4 = plot(time, ones(length(time),1)*P_d(2),'r:','linewidth',1);
legend([h1, h2, h3, h4],'Px','Py','desired Px','desired Py')
set(gca,'fontsize',13)
ylabel('Position [m]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

subplot(2,1,2)
h5 = plot(time, alpha_out*rad2deg,'k','linewidth',2);
hold on
h6 = plot(time, ones(length(time),1)*alpha_d*rad2deg,'k:','linewidth',1);
legend([h5, h6],'alpha', 'desired alpha')
set(gca,'fontsize',13)
ylabel('Orientation [Deg]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(2)
set(gcf,'position',[0 10 600 700])
subplot(3,1,1)
h7 = plot(time, th_out(:,1)*rad2deg,'k','linewidth',2);
hold on
h8 = plot(time, th_out(:,2)*rad2deg,'r','linewidth',2);
h9 = plot(time, th_out(:,3)*rad2deg,'b','linewidth',2);
ylabel('Angle [Deg]')
legend([h7, h8, h9],'J1', 'J2', 'J3')
set(gca,'fontsize',13)
subplot(3,1,2)
plot(time, dth_out(:,1)*rad2deg,'k','linewidth',2)
hold on
plot(time, dth_out(:,2)*rad2deg,'r','linewidth',2)
plot(time, dth_out(:,3)*rad2deg,'b','linewidth',2)
ylabel('Angular Velocity [Deg/s]')
set(gca,'fontsize',13)
subplot(3,1,3)
plot(time, ddth_out(:,1)*rad2deg,'k','linewidth',2)
hold on
plot(time, ddth_out(:,2)*rad2deg,'r','linewidth',2)
plot(time, ddth_out(:,3)*rad2deg,'b','linewidth',2)
ylabel('Acceleration [Deg/s^2]')
xlabel('Time [Sec]')
set(gca,'fontsize',13)

figure(3)
h10 = plot(time, u_out(:,1),'k','linewidth',2);
hold on
h11 = plot(time, u_out(:,2),'r','linewidth',2);
h12 = plot(time, u_out(:,3),'b','linewidth',2);
ylabel('Input Torque [Nm]')
xlabel('Time [Sec]')
legend([h10, h11, h12],'J1', 'J2', 'J3')
set(gca,'fontsize',13)

figure(4)
h100 = plot(time, F_ex_out(:,1),'k','linewidth',2);
hold on
h101 = plot(time, F_ex_out(:,2),'r','linewidth',2);
ylabel('External Force [N]')
xlabel('Time [Sec]')
legend([h100, h101],'Fx', 'Fy')
set(gca,'fontsize',13)

figure(5)
h102 = plot(time, dP_out(:,1),'k','linewidth',2);
hold on
h103 = plot(time, dP_out(:,2),'r','linewidth',2);
xlim([1 tf])
ylabel('Velocity [m/s]')
xlabel('Time [Sec]')
legend([h102, h103],'Vx', 'Vy')
set(gca,'fontsize',13)


% Visualize Simulation with Forward Kinematics
step = 20;

% Camera View
az = 0;
el = 90;

% Origin
origin = [0 0 0]';
x_axis = [0.5 0 0]';
y_axis = [0 0.5 0]';
z_axis = [0 0 0.5]';

Coordinate = 0; % To draw Coordinate --> 1
% Forward Kinematics
for i = 1:step:length(time)
    
    % Sinusoidal Input Joint Angle
    th1 = th_out(i,1);
    th2 = th_out(i,2);
    th3 = th_out(i,3);
    
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
    axis_scale = 0.2;  
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
 
    figure(100)
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',5)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 3)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 3)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 3)

    plot3(P1(1),P1(2),P1(3),'ro','linewidth',2)
    line([origin(1) P1(1)],[origin(2) P1(2)],[origin(3) P1(3)], 'color', 'k', 'linewidth', 2)

    plot3(P2(1),P2(2),P2(3),'ro','linewidth',2)
    line([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)], 'color', 'k', 'linewidth', 2)

    plot3(P3(1),P3(2),P3(3),'ro','linewidth',2)
    line([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)], 'color', 'k', 'linewidth', 2)

    if Coordinate == 1
        line([P1(1) P1_x(1)],[P1(2) P1_x(2)],[P1(3) P1_x(3)], 'color', 'r', 'linewidth', 1)
        line([P1(1) P1_y(1)],[P1(2) P1_y(2)],[P1(3) P1_y(3)], 'color','g', 'linewidth', 1)
        line([P1(1) P1_z(1)],[P1(2) P1_z(2)],[P1(3) P1_z(3)], 'color','b', 'linewidth', 1)
        line([P2(1) P2_x(1)],[P2(2) P2_x(2)],[P2(3) P2_x(3)], 'color', 'r', 'linewidth', 1)
        line([P2(1) P2_y(1)],[P2(2) P2_y(2)],[P2(3) P2_y(3)], 'color','g', 'linewidth', 1)
        line([P2(1) P2_z(1)],[P2(2) P2_z(2)],[P2(3) P2_z(3)], 'color','b', 'linewidth', 1)
        line([P3(1) P3_x(1)],[P3(2) P3_x(2)],[P3(3) P3_x(3)], 'color', 'r', 'linewidth', 1)
        line([P3(1) P3_y(1)],[P3(2) P3_y(2)],[P3(3) P3_y(3)], 'color','g', 'linewidth', 1)
        line([P3(1) P3_z(1)],[P3(2) P3_z(2)],[P3(3) P3_z(3)], 'color','b', 'linewidth', 1)
    end
    
    x_lim = [-0.6 1.6];
    y_lim = [-0.6 1.6];
    
    % Draw Environment 
    plot3(P_ae_out(i,1),P_ae_out(i,2),0.0,'bo','linewidth',10)
    
    % Draw applied force vector
    scale = 1/max(max(2*abs(F_ex_out)));
    F_vec = -scale*F_ex_out(i,:);
    line([P_ae_out(i,1) P_ae_out(i,1)+F_vec(1)],[P_ae_out(i,2) P_ae_out(i,2)+F_vec(2)],'linewidth',2,'color','r')

    xlabel('x- axis','fontsize',13)
    ylabel('y- axis','fontsize',13)
    zlabel('z- axis','fontsize',13)
    title_text = ['y- axis F_{ex} = ' sprintf('%0.2f',F_ex_out(i,2)) ' N'];
    title(title_text)
    set(gca,'fontsize',13)
    axis equal
    xlim(x_lim)
    ylim(y_lim)
    grid on
    view(az, el)
    pause(0.001)
    hold off
end

