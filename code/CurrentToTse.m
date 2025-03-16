function Tse = CurrentToTse(current)
% CurrentToTse Computes the end-effector configuration in the space frame.
%
%   Tse = CurrentToTse(current) returns the 4×4 homogeneous transformation
%   matrix Tse (end-effector configuration in the space frame) given the
%   current configuration vector of the mobile manipulator.
%
%   The input "current" is assumed to be a 1×12 vector containing:
%       current = [phi, x, y, theta1, theta2, theta3, theta4, theta5, u1, u2, u3, u4]
%
%   where:
%       - phi: chassis orientation
%       - x, y: chassis position in the space frame
%       - theta1 to theta5: arm joint angles
%       - u1 to u4: wheel velocities (not used in Tse computation)
%
%   The function computes Tse as:
%       Tse = Tsb * Tb0_traj * T0e
%
%   where:
%       - Tsb: Transformation from the space frame to the chassis frame.
%       - Tb0_traj: Fixed transformation from the chassis frame to the arm base frame.
%       - T0e: End-effector configuration in the arm base frame, computed using
%              forward kinematics via FKinBody.

    %----------------------------------------------------------------------
    % Define Tb0_traj: Transformation from chassis frame {b} to arm base frame {0}
    %----------------------------------------------------------------------
    Tb0_traj = [1, 0, 0, 0.1622;
                0, 1, 0, 0;
                0, 0, 1, 0.0026;
                0, 0, 0, 1];

    %----------------------------------------------------------------------
    % Compute Tsb: Transformation from space frame to chassis frame
    %----------------------------------------------------------------------
    % current(1) = phi (chassis orientation)
    % current(2) = x, current(3) = y (chassis position)
    Tsb = [cos(current(1)), -sin(current(1)), 0, current(2);
           sin(current(1)),  cos(current(1)), 0, current(3);
           0,                0,               1, 0.0963;  % Fixed chassis height
           0,                0,               0, 1];

    %----------------------------------------------------------------------
    % Define body screw axes (Blist) for the manipulator in the arm base frame
    %----------------------------------------------------------------------
    B1 = [0; 0; 1; 0; 0.033; 0];      % Joint 1 screw axis
    B2 = [0; -1; 0; -0.5076; 0; 0];    % Joint 2 screw axis
    B3 = [0; -1; 0; -0.3526; 0; 0];    % Joint 3 screw axis
    B4 = [0; -1; 0; -0.2176; 0; 0];    % Joint 4 screw axis
    B5 = [0; 0; 1; 0; 0; 0];           % Joint 5 screw axis
    Blist = [B1, B2, B3, B4, B5];      % 6×5 matrix of body screw axes

    %----------------------------------------------------------------------
    % Define the home configuration (M) of the end-effector in the arm base frame
    %----------------------------------------------------------------------
    M = [1, 0, 0, 0.033;
         0, 1, 0, 0;
         0, 0, 1, 0.6546;
         0, 0, 0, 1];

    %----------------------------------------------------------------------
    % Extract joint angles from the current configuration
    %----------------------------------------------------------------------
    thetalist = current(4:8)';  % Joint angles as a column vector

    %----------------------------------------------------------------------
    % Compute forward kinematics to get T0e (arm base to end-effector frame)
    %----------------------------------------------------------------------
    T0e = FKinBody(M, Blist, thetalist);

    %----------------------------------------------------------------------
    % Compute Tse: End-effector configuration in the space frame
    %----------------------------------------------------------------------
    Tse = Tsb * Tb0_traj * T0e;
end
