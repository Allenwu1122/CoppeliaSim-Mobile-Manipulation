function [current, success] = TseToCurrent(Tse, chassis, thetalist0)
% TseToCurrent Computes the robot's current configuration from the end-effector configuration.
%
%   [current, success] = TseToCurrent(Tse, chassis, thetalist0) returns the
%   configuration vector "current" for the mobile manipulator given:
%
%       Tse         - 4×4 homogeneous transformation matrix of the end-effector in the space frame.
%       chassis     - 1×3 vector [phi, x, y] representing the chassis orientation and position.
%       thetalist0  - 5×1 initial guess for the arm joint angles for inverse kinematics.
%
%   The configuration vector "current" is organized as:
%       [phi, x, y, theta1, theta2, theta3, theta4, theta5, u1, u2, u3, u4],
%   where the wheel velocities (u1 to u4) are set to zero.
%
%   The function uses the relation:
%       T0e = inv(Tb0_traj) * inv(Tsb) * Tse,
%   and then calls IKinBody to compute the joint angles that yield T0e.
%
%   Note: This function requires the Modern Robotics library function IKinBody.
%
%   Example:
%       Tse = ...;              % Desired end-effector configuration
%       chassis = [0, 0, 0];      % Known chassis configuration [phi, x, y]
%       thetalist0 = zeros(5,1);  % Initial guess for the joint angles
%       [current, success] = TseToCurrent(Tse, chassis, thetalist0);
%
%   If IKinBody fails to converge, a warning is issued and "success" is set to 0.

    %----------------------------------------------------------------------
    % Define Tb0_traj: Transformation from chassis frame {b} to arm base frame {0}
    %----------------------------------------------------------------------
    Tb0_traj = [1, 0, 0, 0.1622;
                0, 1, 0, 0;
                0, 0, 1, 0.0026;
                0, 0, 0, 1];

    %----------------------------------------------------------------------
    % Build Tsb from the given chassis configuration [phi, x, y]
    %----------------------------------------------------------------------
    phi = chassis(1);
    x = chassis(2);
    y = chassis(3);
    Tsb = [cos(phi), -sin(phi), 0, x;
           sin(phi),  cos(phi), 0, y;
           0,         0,        1, 0.0963;
           0,         0,        0, 1];

    %----------------------------------------------------------------------
    % Compute the transformation T0e (arm base to end-effector) from Tse
    %----------------------------------------------------------------------
    % T0e = inv(Tb0_traj) * inv(Tsb) * Tse
    T0e = inv(Tb0_traj) * inv(Tsb) * Tse;

    %----------------------------------------------------------------------
    % Define body screw axes (Blist) for the manipulator in the arm base frame
    %----------------------------------------------------------------------
    B1 = [0; 0; 1; 0; 0.033; 0];      % Joint 1 screw axis
    B2 = [0; -1; 0; -0.5076; 0; 0];    % Joint 2 screw axis
    B3 = [0; -1; 0; -0.3526; 0; 0];    % Joint 3 screw axis
    B4 = [0; -1; 0; -0.2176; 0; 0];    % Joint 4 screw axis
    B5 = [0; 0; 1; 0; 0; 0];           % Joint 5 screw axis
    Blist = [B1, B2, B3, B4, B5];

    %----------------------------------------------------------------------
    % Define the home configuration (M) of the end-effector in the arm base frame
    %----------------------------------------------------------------------
    M = [1, 0, 0, 0.033;
         0, 1, 0, 0;
         0, 0, 1, 0.6546;
         0, 0, 0, 1];

    %----------------------------------------------------------------------
    % Set tolerances for inverse kinematics
    %----------------------------------------------------------------------
    eomg = 0.0001;
    ev = 0.0001;

    %----------------------------------------------------------------------
    % Compute inverse kinematics to obtain the joint angles
    %----------------------------------------------------------------------
    [thetalist, success] = IKinBody(Blist, M, T0e, thetalist0, eomg, ev);
    if ~success
        warning('IKinBody did not converge to a solution.');
    end

    %----------------------------------------------------------------------
    % Construct the complete current configuration vector
    %----------------------------------------------------------------------
    % Wheel velocities (u1, u2, u3, u4) are not determined by Tse; set them to zero.
    current = [phi, x, y, thetalist', 0, 0, 0, 0];
end
