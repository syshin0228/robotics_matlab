% This code is programmed by Sung Yul Shin (syshin0228@utexas.edu) for Robotics Modeling and Control course by Dr. James Sulzer from University of Texas at Austin 
clc
clear all
close all

% Composition of 3D Rotation with respect to Current Frame (x-y-z axis sequence)
deg2rad = pi/180;

% Camera View (https://www.mathworks.com/help/matlab/ref/view.html)
az = 120;    % azimuth
el = 30;    % elevation

% Origin
origin = [0 0 0]';
x_axis = [1 0 0]';
y_axis = [0 1 0]';
z_axis = [0 0 1]';

% Input Angle
x_rotation = 90
y_rotation = 90
z_rotation = 90

% Object on/off
% 0 -> nothing, 1 -> object, 2 -> object with a position mark, 3 -> a position mark without object  
object = 0;

% Rotation Angles
x_rotation = x_rotation*deg2rad;
y_rotation = y_rotation*deg2rad;
z_rotation = z_rotation*deg2rad;

% Sampling Rate Info
Hz = 100;
dt = 1/Hz;

figure(1)
set(gcf,'Position', [0 100 800 600])
for i = 1:1:300
    
    t = i * dt;
 
    if t < 1.0  
        % x- axis rotation
        alpha = t * x_rotation;
        beta = 0.0;
        gamma = 0.0;
        Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        R = Rx*Ry*Rz;   % Composition of 3D Rotation with respect to Current Frame (x- axis rotation)


    elseif (t > 1.0) && (t <= 2.0)
        % y- axis rotation
        alpha = x_rotation;
        beta = (t - 1.0) * y_rotation;
        gamma = 0.0;
        Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        R = Rx*Ry*Rz;   % Composition of 3D Rotation with respect to Current Frame (y- axis rotation)
        
    elseif (t > 2.0) && (t <= 3.0)
        % z- axis rotation
        alpha = x_rotation;
        beta = y_rotation;
        gamma = (t - 2.0) * z_rotation;
        Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        R = Rx*Ry*Rz;   % Composition of 3D Rotation with respect to Current Frame (z- axis rotation)
    end
    
    % Get coordinate position
    x_R = R(:,1);
    y_R = R(:,2);
    z_R = R(:,3);
    
    % Scale axis
    axis_scale = 0.5;
    x_R = axis_scale * x_R;
    y_R = axis_scale * y_R;
    z_R = axis_scale * z_R;
    
    % Draw global coordinate
    plot3(origin(1),origin(2),origin(3),'ro','linewidth',3)
    hold on
    line([origin(1) x_axis(1)],[origin(2) x_axis(2)],[origin(3) x_axis(3)], 'color', 'r', 'linewidth', 1)
    line([origin(1) y_axis(1)],[origin(2) y_axis(2)],[origin(3) y_axis(3)], 'color','g', 'linewidth', 1)
    line([origin(1) z_axis(1)],[origin(2) z_axis(2)],[origin(3) z_axis(3)], 'color','b', 'linewidth', 1)
    
    line([origin(1) x_R(1)],[origin(2) x_R(2)],[origin(3) x_R(3)], 'color', 'r', 'linewidth', 3)
    line([origin(1) y_R(1)],[origin(2) y_R(2)],[origin(3) y_R(3)], 'color','g', 'linewidth', 3)
    line([origin(1) z_R(1)],[origin(2) z_R(2)],[origin(3) z_R(3)], 'color','b', 'linewidth', 3)
    
    % Object Info
    alpha_val = 0.05;
    l = 0.8;    % Length
    w = 0.6;    % Width
    h = 0.4;    % Height
    P0 = [0 0 0]';
    P1 = R*[w 0 0]';
    P2 = R*[w l 0]';
    P3 = R*[0 l 0]';
    P4 = R*[0 0 h]';
    P5 = R*[w 0 h]';
    P6 = R*[w l h]';
    P7 = R*[0 l h]';
    
    if object == 1
        % Object
        x1 = [P0(1) P1(1) P2(1) P3(1)];
        y1 = [P0(2) P1(2) P2(2) P3(2)];
        z1 = [P0(3) P1(3) P2(3) P3(3)];
        h1 = patch(x1,y1,z1,'b');
        set(h1,'FaceAlpha',alpha_val,'edgecolor','k');

        x2 = [P4(1) P5(1) P6(1) P7(1)]; 
        y2 = [P4(2) P5(2) P6(2) P7(2)];
        z2 = [P4(3) P5(3) P6(3) P7(3)];
        h2 = patch(x2,y2,z2,'b');
        set(h2,'FaceAlpha',alpha_val,'edgecolor','k');

        x3 = [P0(1) P1(1) P5(1) P4(1)];
        y3 = [P0(2) P1(2) P5(2) P4(2)];
        z3 = [P0(3) P1(3) P5(3) P4(3)];
        h3 = patch(x3,y3,z3,'b');
        set(h3,'FaceAlpha',alpha_val,'edgecolor','k');

        x4 = [P3(1) P2(1) P6(1) P7(1)]; 
        y4 = [P3(2) P2(2) P6(2) P7(2)];
        z4 = [P3(3) P2(3) P6(3) P7(3)];
        h4 = patch(x4,y4,z4,'b');
        set(h4,'FaceAlpha',alpha_val,'edgecolor','k');

        x5 = [P1(1) P2(1) P6(1) P5(1)]; 
        y5 = [P1(2) P2(2) P6(2) P5(2)];
        z5 = [P1(3) P2(3) P6(3) P5(3)];
        h5 = patch(x5,y5,z5,'b');
        set(h5,'FaceAlpha',alpha_val,'edgecolor','k');

        x6 = [P0(1) P3(1) P7(1) P4(1)]; 
        y6 = [P0(2) P3(2) P7(2) P4(2)];
        z6 = [P0(3) P3(3) P7(3) P4(3)];
        h6 = patch(x6,y6,z6,'b');
        set(h6,'FaceAlpha',alpha_val,'edgecolor','k');
        
    elseif object == 2
        % Object with position mark  
        plot3(P6(1),P6(2),P6(3),'bo','linewidth',2)
        
        x1 = [P0(1) P1(1) P2(1) P3(1)];
        y1 = [P0(2) P1(2) P2(2) P3(2)];
        z1 = [P0(3) P1(3) P2(3) P3(3)];
        h1 = patch(x1,y1,z1,'b');
        set(h1,'FaceAlpha',alpha_val,'edgecolor','k');

        x2 = [P4(1) P5(1) P6(1) P7(1)]; 
        y2 = [P4(2) P5(2) P6(2) P7(2)];
        z2 = [P4(3) P5(3) P6(3) P7(3)];
        h2 = patch(x2,y2,z2,'b');
        set(h2,'FaceAlpha',alpha_val,'edgecolor','k');

        x3 = [P0(1) P1(1) P5(1) P4(1)];
        y3 = [P0(2) P1(2) P5(2) P4(2)];
        z3 = [P0(3) P1(3) P5(3) P4(3)];
        h3 = patch(x3,y3,z3,'b');
        set(h3,'FaceAlpha',alpha_val,'edgecolor','k');

        x4 = [P3(1) P2(1) P6(1) P7(1)]; 
        y4 = [P3(2) P2(2) P6(2) P7(2)];
        z4 = [P3(3) P2(3) P6(3) P7(3)];
        h4 = patch(x4,y4,z4,'b');
        set(h4,'FaceAlpha',alpha_val,'edgecolor','k');

        x5 = [P1(1) P2(1) P6(1) P5(1)]; 
        y5 = [P1(2) P2(2) P6(2) P5(2)];
        z5 = [P1(3) P2(3) P6(3) P5(3)];
        h5 = patch(x5,y5,z5,'b');
        set(h5,'FaceAlpha',alpha_val,'edgecolor','k');

        x6 = [P0(1) P3(1) P7(1) P4(1)]; 
        y6 = [P0(2) P3(2) P7(2) P4(2)];
        z6 = [P0(3) P3(3) P7(3) P4(3)];
        h6 = patch(x6,y6,z6,'b');
        set(h6,'FaceAlpha',alpha_val,'edgecolor','k');
        
    elseif object == 3
        % Position mark without object
        plot3(P6(1),P6(2),P6(3),'ko','linewidth',2)
        line([origin(1) P6(1)],[origin(2) P6(2)],[origin(3) P6(3)], 'color','k', 'linewidth', 1)

    else
        % Nothing
        
    end

    xlabel('x- axis')
    ylabel('y- axis')
    zlabel('z- axis')
    title('Composition of Rotation about Current Frame (x-y-z)')
    set(gca,'fontsize',13)
    axis equal
    axis([-1 1 -1 1 -1 1])
    grid on
    view(az, el)
    pause(0.001)
    hold off
end    


