clear; close all; clc;  % Clear workspace, close figures, and clear command window
addpath('../mr')        % Add the 'mr' (Modern Robotics) library path for functions like FKinBody, etc.

% For Testing
% Paths to example CSV files (commented out for reference):
% C:\Documents\MATLAB\MAE 204\Final Project\Scene6_example.csv
% C:\Documents\MATLAB\MAE 204\Final Project\milestone2.csv

%% Tse Calculation
% Compute the initial end-effector configuration (Tse) using chassis, base, and arm transformations
% Inputs: T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k

% This vector represents:
% phi=0, x=0, y=0, joint angles=[0, 0, 0.2, -1.6, 0], wheel velocities=[0, 0, 0, 0]
% The current configuration includes the chassis orientation (phi), position (x, y), 
% joint angles for the arm, and wheel velocities.

% Tse = CurrentToTse(current);  % Uncomment to calculate Tse if needed

%% Trajectory Generator
% Generate a trajectory for the robot to move a block from initial to final configuration

%---------------------------------------------------------------------------
% Given Tse for Reference End-Effector Configuration (Tse)
%---------------------------------------------------------------------------
Tse_given = [0 0 1 0;   % End-effector configuration in space frame
             0 1 0 0;   
            -1 0 0 0.5; % Position 0.5 meters below the space frame
             0 0 0 1]; 

%---------------------------------------------------------------------------
% Define Tsc_initial: Initial block configuration relative to space frame {s}
%---------------------------------------------------------------------------
Tsc_initial = [1 0 0 1;       % Block at x=1 m, y=0 m
               0 1 0 0;       % Aligned with space frame
               0 0 1 0.025;   % Height of 0.025 m
               0 0 0 1];      % Homogeneous coordinate

%---------------------------------------------------------------------------
% Define Tsc_final: Final block configuration relative to space frame {s}
%---------------------------------------------------------------------------
Tsc_final = [0 1 0 0;         % Rotated 90 degrees, at x=0 m, y=-1 m
            -1 0 0 -1;
             0 0 1 0.025;    % Same height (0.025 m)
             0 0 0 1];

%---------------------------------------------------------------------------
% Define Tce_g: Grasp configuration of end-effector relative to block frame {c}
%---------------------------------------------------------------------------
Tce_g = [0 0 1 0.02;           % End-effector aligned to grasp block (rotated)
         0 1 0 0;
        -1 0 0 0.005;           % At block's origin
         0 0 0 1];

%---------------------------------------------------------------------------
% Define Tce_s: Standoff configuration above the block relative to block frame {c}
%---------------------------------------------------------------------------
Tce_s = [0 0 1 0;             % Same orientation as grasp
         0 1 0 0;
        -1 0 0 0.3;           % Standoff height of 0.3 m above block
         0 0 0 1];

Tce_place = [0 0 1 0.02;           % Same orientation as grasp
             0 1 0 0;
            -1 0 0 0;            % Place at block origin
             0 0 0 1];

%---------------------------------------------------------------------------
% Generate the trajectory using TrajectoryGenerator function
%---------------------------------------------------------------------------
% Operations for generating the trajectory:
% 1. Move to standoff above initial position
% 2. Move down to grasp position
% 3. Close gripper (1 = closed)
% 4. Move back to standoff position
% 5. Move to standoff above final position
% 6. Move down to final grasp position
% 7. Open gripper (0 = open)
% 8. Move back to standoff above final position

durations = [3, 1, 1, 1, 3, 1, 1, 1]; % Define durations for each operation in the trajectory
trajectory = TrajectoryGenerator(Tse_given, Tsc_initial, Tsc_final, Tce_g, Tce_s, Tce_place, durations);
% Inputs: Initial end-effector config, initial/final block configs, grasp/standoff configs, k=1 (time scaling factor)
writematrix(trajectory, 'trajectory.csv');  % Save trajectory to CSV file for later use

% Reference path (commented out):
% C:\Documents\MATLAB\MAE 204\Final Project\trajectory.csv

% Read the generated trajectory for further processing
[lines, c] = size(trajectory);

% Initial Guess
chassis = [0, -0.5, 0];  % Chassis initial position
thetalist0 = [0, 0, 0.2, -1.6, 0]';  % Initial joint angles

% Inverse Kinematics of Tse_given
[current, success] = TseToCurrent(Tse_given, chassis, thetalist0);  % Compute the robot configuration to match Tse

% Initial Error (set error for testing) %% No Error For Feed Forward
% current = current + [30*(pi/180) -0.2 zeros(1,10)];  % Set error in configuration

%% Feedback Control Setup and Test
% Test the FeedbackControl function with sample inputs

% Initialize the global integrated error vector
global Xerrdt_int
Xerrdt_int = zeros(6,1);  % Initialize error vector for control integration

%--------------------------------------------------------------------------
% Define control gains and time step
%--------------------------------------------------------------------------
Kp = zeros(6);    % 6×6 proportional gain matrix (zero for testing)
Ki = zeros(6);   % 6×6 integral gain matrix (zero for testing)
dt = 1;          % Time step (1 second for testing)
maxVel = 50;     % Maximum velocity

% Initialize storage for velocities and errors
wheel_vel = zeros(lines,4);      % 4×1 array for wheel velocities
joints_vel = zeros(lines,5);     % 5×1 array for joint velocities
X_err = zeros(lines,6);          % 6×1 array for configuration error twist
new_state = zeros(lines,12);     % 12×1 array for robot state updates
robot_config = zeros(lines,13);  % 13×1 array for robot configuration history

% Run control loop for each trajectory step
for i = 1:lines-1
    % Perform feedback control to get wheel and joint velocities, and error twist
    [wheel_vel(i,:), joints_vel(i,:), X_err(i,:)] = FeedbackControl(current, trajectory(i,:), trajectory(i+1,:), Kp, Ki, dt);
    % Outputs:
    %   wheel_vel:  4×1 wheel velocities
    %   joints_vel: 5×1 joint velocities
    %   X_err:      6×1 configuration error twist
    
    % Combine joint and wheel velocities into a single vector
    jointWheelVels = [joints_vel(i,:) wheel_vel(i,:)];

    % Compute the new state of the robot based on current velocities and time step
    new_state(i,:) = NextState(current, jointWheelVels, dt, maxVel);
    
    % Update the current configuration for the next iteration
    current = new_state(i,:);

    % Store the robot configuration, including gripper state (0 or 1)
    robot_config(i,:) = [new_state(i,:) 0];  

    % If within a specified range of iterations, set gripper state to 1 (closed)
    if i >= sum(durations(1:2))*100 && i <= sum(durations(1:6))*100-1  
        robot_config(i,:) = [new_state(i,:) 1];  
    end
end

% Write the robot configuration to a CSV file
writematrix(robot_config,'Feedforward.csv');
% C:\Documents\MATLAB\MAE 204\Final Project\robo_config.csv
