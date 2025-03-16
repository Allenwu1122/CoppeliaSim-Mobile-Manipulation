clear; close all; clc;  % Clear workspace, close figures, and clear command window
addpath('../mr')        % Add the 'mr' (Modern Robotics) library path for essential functions

%% --------------------------- Initialization --------------------------- %%

% Reference end-effector configuration in space frame
Tse_given = [0 0 1 0;
             0 1 0 0;
            -1 0 0 0.5;
             0 0 0 1]; 

% Initial block configuration relative to space frame
Tsc_initial = [1 0 0 1;
               0 1 0 1;
               0 0 1 0.025;
               0 0 0 1];

% Final block configuration relative to space frame
Tsc_final = [0 1 0 -2;
            -1 0 0 -2;
             0 0 1 0.025;
             0 0 0 1];

% End-effector grasp configuration relative to block frame
Tce_g = [0 0 1 0.02;
         0 1 0 0;
        -1 0 0 0.005;
         0 0 0 1];

% Standoff configuration above the block relative to block frame
Tce_s = [0 0 1 0;
         0 1 0 0;
        -1 0 0 0.3;
         0 0 0 1];

% Final placement configuration
Tce_place = [0 0 1 0.02;
             0 1 0 0;
            -1 0 0 0;
             0 0 0 1];

% Duration of each movement phase
motion_durations = [3, 1, 1, 1, 6, 1, 1, 1];  % Seconds

%% --------------------------- Trajectory Generation --------------------------- %%

% Generate trajectory
trajectory = TrajectoryGenerator(Tse_given, Tsc_initial, Tsc_final, Tce_g, Tce_s, Tce_place, motion_durations);
writematrix(trajectory, 'newtasktrajector.csv');  % Save trajectory to CSV

% Retrieve number of trajectory points
[lines, ~] = size(trajectory);

%% --------------------------- Initial Setup --------------------------- %%

% Initial robot configuration
chassis = [0, -0.5, 0];  % Chassis initial pose
joint_angles_init = [0, 0, 0.2, -1.6, 0]';  % Initial joint angles
[current, success] = TseToCurrent(Tse_given, chassis, joint_angles_init);

% Apply an initial error
current = current + [30*(pi/180) -0.2 zeros(1,10)];

%% --------------------------- Feedback Control --------------------------- %%

% Define control gains
Kp = eye(6)*1.4;    % Proportional gain matrix
Ki = zeros(6);  % Integral gain matrix

dt = 1;        % Time step (1 second)
maxVel = 50;   % Maximum velocity constraint

% Initialize global integrated error for PID control
global Xerrdt_int
Xerrdt_int = zeros(6,1);

% Preallocation for efficiency
wheel_vel = zeros(lines,4);
joints_vel = zeros(lines,5);
X_err = zeros(lines,6);
new_state = zeros(lines,12);
robot_config = zeros(lines,13);
mu_w = zeros(1,lines);
mu_v = zeros(1,lines);

for i = 1:lines-1
    % Compute control actions
    [wheel_vel(i,:), joints_vel(i,:), X_err(i,:), mu_w(i), mu_v(i)] = FeedbackControl(current, trajectory(i,:), trajectory(i+1,:), Kp, Ki, dt);
    
    % Compute the new robot state
    jointWheelVels = [joints_vel(i,:) wheel_vel(i,:)];
    new_state(i,:) = NextState(current, jointWheelVels, dt, maxVel);
    current = new_state(i,:);
    robot_config(i,:) = [new_state(i,:) 0];  % Default: gripper open
    
    % Close gripper during block transport
    if i >= sum(motion_durations(1:2))*100 && i <= sum(motion_durations(1:6))*100-1  
        robot_config(i,:) = [new_state(i,:) 1];  % Gripper closed
    end
end

% Save robot configuration data
writematrix(robot_config, 'newtask_robo_config.csv');
