%===============================================================
function hwk5_skeleton
%===============================================================
% Note that it is considered bad practice to use global variables when
% programming.  Don't do this at home!!!
global V_ROBOT OMEGA_MAX DT_ODOM WHEELBASE SIGMA_WHEEL SIGMA_BEARING;

% Initialize the robot parameters
V_ROBOT=5;          % Robot velocity in m/s
DT_ODOM = 0.1;      % Update rate for odometry (10 Hz)    
OMEGA_MAX=pi/4;     % Angular velocity bound in radians/s 
WHEELBASE = 1.0;    % Wheelbase of the robot in meters
SIGMA_WHEEL = 0.1;  % This is the std dev of the wheel velocity in m/s

% Camera measurement parameters
SIGMA_BEARING = 3*pi/180;   % Std dev for bearing estimates in radians

% Set up the figure 
close all; figure; axis equal; axis([0 120 0 80]); hold on;

% Initial robot pose
x0 = 15;  y0 = 15;  theta0 = 0;

% This is the actual robot position.  The robot is green.
robot = make_robot( x0, y0, theta0, 'size', 1.5, 'color', 'g', 'make_trail', 1 );
% This is the position of the robot as estimated by your EKF.  It is redd
robot_hat = make_robot( x0+10*randn, y0+10*randn, theta0+pi/8*randn, 'size', 1.5, 'color', 'r', 'make_trail', 1 );
% Make the path for the robot to follow
rectangle( 'position', [15 15 90 50],'linestyle',':', 'edgecolor', 'k' );

% This is our initial Covariance estimate.  Note is is NOT correct.  You
% can play with this as you see fit based upon other parameters given. 
P = [100 0 0; 0 100 0; 0 0 (pi/18)^2];

num_cameras = 0;
button = 1;
while 1    
    % Gets a single mouse input from the user.  The corresponding x-y
    % position will be the location of the camera
    [ x, y, button ] = ginput(1);
    % If you hit the right mouse button, the loop will terminate
    if button==3, break; end
    num_cameras = num_cameras+1;
    % This is the position of the camera, the range it is capable of 
    % transmitting and the color it is initially set to.  
    camera(num_cameras) = make_camera( x, y, 20, [0.5 0.5 0.5] );
end

% This is just here for demonstration purposes showing how the robots will
% move.  You will need to comment out this loop, and comment in the while
% loop below where you will add the EKF code.  
%for i=1:50
 %   robot = move_robot( robot, robot.x+cos(robot.theta), robot.y+sin(robot.theta), robot.theta+0.05*randn );
  %  robot_hat = move_robot( robot_hat, robot_hat.x+cos(robot_hat.theta), robot_hat.y+sin(robot_hat.theta), robot_hat.theta+0.05*randn );
   % pause(0.5);
%end


% %******************************************
% % ADD OTHER CODE HERE
% %******************************************


current_leg=1;
 while current_leg~=9
     if (current_leg == 1)
         robot = move_robot( robot, 105, 15, 0 );
         robot_hat = move_robot( robot_hat, 105, 15, 0 );
     end
     if (current_leg == 2)
         robot = move_robot( robot, 105, 15, pi/2 );
         robot_hat = move_robot( robot_hat, 105, 15, pi/2 );
     end
     if (current_leg == 3)
         robot = move_robot( robot, 105, 65, pi/2 );
         robot_hat = move_robot( robot_hat, 105, 65, pi/2 );
     end
     if (current_leg == 4)
         robot = move_robot( robot, 105, 65, pi );
         robot_hat = move_robot( robot_hat, 105, 65, pi );
     end
     if (current_leg == 5)
         robot = move_robot( robot, 15, 65, pi );
         robot_hat = move_robot( robot_hat, 15, 65, pi );
     end
     if (current_leg == 6)
         robot = move_robot( robot, 15, 65, 3*pi/2 );
         robot_hat = move_robot( robot_hat, 15, 65, 3*pi/2 );
     end
     if (current_leg == 7)
         robot = move_robot( robot, 15, 15, 3*pi/2 );
         robot_hat = move_robot( robot_hat, 15, 15, 3*pi/2 );
     end
     if (current_leg == 8)
         robot = move_robot( robot, 15, 15, 0 );
         robot_hat = move_robot( robot_hat, 15, 15, 0 );
     end     
     ekf = extendedKalmanFilter(robot_hat, MeasurementFcn ,P);
     ekf.run();
%     % Measurement Update Phase
     for i=1:num_cameras
         % Each camera is tested to see if it can be seen by the robot.  
         % Visible cameras are set to green, and a line reflecting its
         % measured position as estimated by the robot is also plotted.  
         [ camera(i), bearing ] = test_camera( camera(i), robot );        
         if ~isempty( bearing )
             [ robot_hat, P ] = MeasurementUpdate( robot_hat, P, camera(i), bearing );

             
         end
     end
% 
%     % Time Update Phase.  Note we do multiple time updates for each
%     % measurement update because the update rate of the odometry is higher
     for j=1:1/DT_ODOM
         [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, legV, legOmega );
         h = plot_cov( mu, P );
     end
% 
end
function h = plot_cov( mu, P )

e = eig(P);

ellipse = pdellip(mu, P, 0);
plot ellipse;

%==================================================================================
function [ robot, robot_hat, P ] = TimeUpdate( robot, robot_hat, P, v, omega )
%==================================================================================
global DT_ODOM WHEELBASE SIGMA_WHEEL;
robot_hat = make_robot( x, y, bearing, 'size', 1.5, 'color', 'r', 'make_trail', 1 );
%******************************************
% ADD OTHER CODE HERE
%******************************************


%==================================================================================
function [ robotHat, P ] = MeasurementUpdate( robot_hat, P, camera, range )
%==================================================================================
global SIGMA_BEARING;
%******************************************

bearing = atan2( robot.y-camera.y, robot.x-camera.x );
plot ellipse;

%******************************************
