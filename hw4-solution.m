%===============================================================
function hwk5
%===============================================================
% Note that it is considered bad practice to use global variables when
% programming.  Don't do this at home!!!
global V_ROBOT OMEGA_MAX DT_ODOM WHEELBASE SIGMA_WHEEL SIGMA_BEARING Q;

% Initialize the robot parameters
V_ROBOT=5;          % Robot velocity in m/s
DT_ODOM = 0.1;      % Update rate for odometry (10 Hz)    
OMEGA_MAX=pi/4;     % Angular velocity bound in radians/s 
WHEELBASE = 1.0;    % Wheelbase of the robot in meters
SIGMA_WHEEL = 0.15;  % This is the variance of the wheel velocity in m/s

% Camera measurement parameters
SIGMA_BEARING = 3*pi/180;   % Std dev for bearing estimates in radians
RANGE         = 20;         % Sets the max range in meters for camera

% Set up the figure 
close all; figure; axis equal; axis([0 120 0 80]); hold on;

% Initial robot pose
x0 = 15;  y0 = 15;  theta0 = 0;

% This is the actual robot position.  The robot is green.
robot = make_robot( x0, y0, theta0, 'size', 1.5, 'color', 'g', 'make_trail', 1 );
%robot_hat = make_robot( x0, y0, theta0, 'size', 1.5, 'color', 'g', 'make_trail', 1 );

% This is the position of the robot as estimated by your EKF.  It is red
robot_hat = make_robot( x0+10*randn, y0+10*randn, theta0+pi/8*randn, 'size', 1.5, 'color', 'r', 'make_trail', 1 );

% Make the path for the robot to follow
rectangle( 'position', [15 15 90 50],'linestyle',':', 'edgecolor', 'k' );

%Plot the initial covariance
p_cov  =  plot(0,0,'erasemode','xor');


% This is our initial Covariance estimate.  Note it is NOT correct.  You
% can play with this as you see fit based upon other parameters given. 
P = [10^2  0  0 
     0   10^2 0
     0   0   (pi/8)^2];
 
%Control Noise
sigmaV= .15; 
sigmaG= 10*pi/180;
Q= [sigmaV^2 0 
    0 sigmaG^2];

ellipse = make_covariance_ellipse([robot_hat.x robot_hat.y],P);
set(p_cov, 'xdata', ellipse(1,:),'ydata', ellipse(2,:));

num_cameras = 0;
button = 1;

while 1    
    % Gets a single mouse input from the user.  The corresponding x-y
    % position will be the location of the camera
    [ x, y, button ] = ginput(1);
    % If you hit the right mouse button, the loop will terminate
    if button==3
        break; 
    end
    num_cameras = num_cameras+1;
    % This is the position of the camera, the range it is capable of 
    % transmitting and the color it is initially set to.  
    camera(num_cameras) = make_camera( x, y, RANGE, [0.5 0.5 0.5] );
end

%******************************************
% ADD OTHER CODE HERE
%******************************************
 my_ellipse=[];
 current_leg=1;

 last_robot = robot;
 dist = 0;
 angle = 0;
 v = 0;
 w = 0;
 while current_leg~=9
          
    % Measurement Update Phase
    for i=1:num_cameras
        % Each camera is tested to see if it can be seen by the robot.  
        % Visible cameras are set to green, and a line reflecting its
        % measured position as estimated by the robot is also plotted.  
        [camera(i), bearing] = test_camera( camera(i), robot );
        
        if ~isempty( bearing )
            delete(camera(i).h);
            %Turn the camera green
            camera(i) = make_camera(camera(i).x, camera(i).y, RANGE, [0 1 0]);
            [ robot_hat, P ] = MeasurementUpdate( robot_hat, P, camera(i), bearing );
        else
            %Turn the camera gray
            delete(camera(i).h);
            camera(i) = make_camera(camera(i).x, camera(i).y, RANGE, [0.5 0.5 0.5]);
        end
    end

    % Time Update Phase.  Note we do multiple time updates for each
    % measurement update because the update rate of the odometry is higher
    for j=1:1/DT_ODOM
        
        if current_leg == 1 || current_leg == 5
            if dist < 90
                v=5;
                w=0;
            else
                v=0;
                w=0;
                dist=0;
                angle=0;
                current_leg = current_leg + 1;
                break;
            end
        elseif current_leg == 3 || current_leg == 7
            if dist < 50
                v=5;
                w=0;
            else
                v=0;
                w=0;
                dist=0;
                angle=0;
                current_leg = current_leg + 1;
                break;
            end
        else
            if angle < pi/2
                v=0;
                w=pi/4;
            else
                v=0;
                w=0;
                dist=0;
                angle=0;
                current_leg = current_leg + 1;
                break;
            end
        end
        
        position_error = sqrt((robot.x-robot_hat.x)^2+(robot.y-robot_hat.y)^2);
        [V D] = eig(P);
        covariance = D(1,1)*D(2,2);
        fprintf('%f %f \n',position_error, covariance);
        
        %Update robot position
        [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, v, w );
            
        %Plot the robot ellipse
        ellipse = make_covariance_ellipse([robot_hat.x robot_hat.y],P);
        set(p_cov, 'xdata', ellipse(1,:),'ydata', ellipse(2,:));
        
        %Calculate the distance traveled
        dist = dist + sqrt((robot.x - last_robot.x)^2+(robot.y-last_robot.y)^2);
        angle = angle + robot.theta - last_robot.theta;
        last_robot = robot;
        
        drawnow;
        
    end

end
 
end

%==================================================================================
function [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, v, w)
%==================================================================================
global DT_ODOM WHEELBASE SIGMA_WHEEL Q;

    %Useful mappings
    x       =  robot.x;
    y       =  robot.y;
    th      =  robot.theta;
    x_hat   =  robot_hat.x;
    y_hat   =  robot_hat.y;
    th_hat  =  robot_hat.theta;

    %Corrupt the control input with noise
    v_p = v + Q(1,1)*randn(1);
    w_p = w + Q(2,2)*randn(1);

    %Update the robot position
    x_p   = x  + v*cos(th)*DT_ODOM;
    y_p   = y  + v*sin(th)*DT_ODOM;
    th_p  = th + w*DT_ODOM;

    %Update the hypothesis position
    x_hat_p  = x_hat  + v_p*cos(th_hat)*DT_ODOM;
    y_hat_p  = y_hat  + v_p*sin(th_hat)*DT_ODOM;
    th_hat_p = th_hat + w_p*DT_ODOM;

    robot = move_robot( robot, x_p, y_p, th_p); 
    robot_hat = move_robot( robot_hat, x_hat_p, y_hat_p, th_hat_p); 
    
    %Update the uncertainty
    P = update_pose_cov(v,P,th_p, DT_ODOM, Q);
    
end

%==================================================================================
function [ robot_hat, P ] = MeasurementUpdate(robot_hat, P, camera, z )
%==================================================================================
global SIGMA_BEARING;

%Robot Pose 
mu = [robot_hat.x robot_hat.y robot_hat.theta]';

%Expected measurement
z_p = atan2(camera.y-robot_hat.y,camera.x-robot_hat.x)-robot_hat.theta;
z_p = pi_to_pi(z_p);

%Measurement Jacobian
q = (camera.x-robot_hat.x)^2 + (camera.y-robot_hat.y)^2;
H = [(camera.y-robot_hat.y)/q -(camera.x-robot_hat.x)/q -1];

%Measurement covariance
Z = H*P*H' + SIGMA_BEARING;

%Kalman gain
K = P*H'/Z;

%Update position
mu = mu + K*(z-z_p);
P = (eye(3)-K*H)*P;

robot_hat.x = mu(1);
robot_hat.y = mu(2);
robot_hat.theta = mu(3);

end


function camera = make_camera(x, y, range, color)
     
    fig_coords = [x-1 x+1 x
                  y   y   y+2];

    camera.x     = x;
    camera.y     = y;
    camera.range = range;
    camera.h     = patch(fig_coords(1,:), fig_coords(2,:), color); 
              
              
end

function [camera, bearing] = test_camera( camera, robot )

    %First we calculate the difference between the robot and the camera
    dx = camera.x - robot.x;
    dy = camera.y - robot.y;
    
    dist = sqrt(dx^2 + dy^2);
    
    if dist > camera.range
        bearing = [];
        return;
    else
        bearing = atan2(dy,dx)-robot.theta;
        bearing = pi_to_pi(bearing);
        return;
    end
    
end

function P = update_pose_cov(v,P_tm1, th, dt, Q)

Gv = [1  0 -v*sin(th)*dt;
      0  1  v*cos(th)*dt;
      0  0  1];
 
Gu = [cos(th)*dt  0
      sin(th)*dt  0
      0           dt];

%Predict covariance
P = Gv*P_tm1*Gv' + Gu*Q*Gu';

end

function p = make_covariance_ellipse(xf,Pf)
% part of plotting routines
    p= [];

    N= 20;
    inc= 2*pi/N;
    phi= 0:inc:2*pi;
    circ= 2*[cos(phi); sin(phi)];
    
    p= zeros (2, N+2);

    ctr= 1;
    
    ii= ctr:(ctr+N+1);
    p(:,ii)= make_ellipse(xf(1:2), Pf(1:2,1:2), circ);
    ctr= ctr+N+2;
end    
        
function p = make_ellipse(x,P,circ)
% make a single 2-D ellipse 
r= sqrtm_2by2(P);
a= r*circ;
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

end

function X = sqrtm_2by2(A)

[Q, T] = schur(A);        % T is real/complex according to A.
%[Q, T] = rsf2csf(Q, T);   % T is now complex Schur form.

R = zeros(2);

R(1,1) = sqrt(T(1,1));
R(2,2) = sqrt(T(2,2));
R(1,2) = T(1,2) / (R(1,1) + R(2,2));

X = Q*R*Q';

end


