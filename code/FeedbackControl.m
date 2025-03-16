function [wheel_vel, joints_vel, X_err, mu_det_w, mu_det_v] = FeedbackControl(current, Tse_d_row, Tse_d_next_row, Kp, Ki, dt)
%--------------------------------------------------------------------------
% FeedbackControl:
%   Computes the commanded end-effector twist V(t) in the end-effector frame,
%   using a feedforward + feedback law from Modern Robotics Equations (11.16)/(13.37).
%
% INPUTS:
%   current     : current states config (1x12) - [phi; x; y; theta1; theta2; ...; theta5; u1; u2; u3; u4]
%   T_se_d      : desired end-effector config at time t (4×4)
%   T_se_d_next : desired end-effector config at time t + dt (4×4)
%   Kp, Ki      : proportional and integral gains (scalars or 6×6 if decoupled gains)
%   dt          : time step, Δt
%
% OUTPUTS:
%   wheel_vel   : 4×1 vector of wheel velocities (u1, u2, u3, u4)
%   joints_vel  : 5×1 vector of joint velocities (theta_dot1, ..., theta_dot5)
%   X_err       : 6×1 end-effector configuration error twist
%--------------------------------------------------------------------------

    global Xerrdt_int  % Global variable to store the integrated error across timesteps
    
    %--------------------------------------------------------------------------
    % Define the chassis configuration transformation (T: odom to chassis frame)
    %--------------------------------------------------------------------------
    Tsb = [cos(current(1)), -sin(current(1)), 0,  current(2);  % current(1) = phi (chassis orientation)
         sin(current(1)),  cos(current(1)), 0,  current(3);  % current(2) = x, current(3) = y
                0,           0,           1,    0.0963;       % Fixed height of chassis
                0,           0,           0,     1];

    %--------------------------------------------------------------------------
    % Define body screw axes (Blist) for the manipulator in the arm base frame
    %--------------------------------------------------------------------------
    B1 = [0; 0; 1; 0; 0.033; 0];    % Screw axis for joint 1
    B2 = [0; -1; 0; -0.5076; 0; 0]; % Screw axis for joint 2
    B3 = [0; -1; 0; -0.3526; 0; 0]; % Screw axis for joint 3
    B4 = [0; -1; 0; -0.2176; 0; 0]; % Screw axis for joint 4
    B5 = [0; 0; 1; 0; 0; 0];        % Screw axis for joint 5
    Blist = [B1, B2, B3, B4, B5];    % 6×5 matrix of body screw axes

    %--------------------------------------------------------------------------
    % Define the home configuration (M) of the end-effector in the arm base frame
    %--------------------------------------------------------------------------
    M = [1, 0, 0, 0.033;  % Home configuration (T_0e at zero joint angles)
         0, 1, 0, 0;
         0, 0, 1, 0.6546;
         0, 0, 0, 1];

    %--------------------------------------------------------------------------
    % Define constants and wheel geometry for the mobile base
    %--------------------------------------------------------------------------
    w = 0.15;       % Width parameter of the chassis
    l = 0.235;      % Length parameter of the chassis
    r = 0.0475;     % Wheel radius
    lw_forF = 1/(l+w);  % Helper term for wheel geometry matrix F
    F = (r/4) * [-lw_forF, lw_forF, lw_forF, -lw_forF;  % Wheel contribution to chassis twist
                  1, 1, 1, 1;
                 -1, 1, -1, 1];

    %--------------------------------------------------------------------------
    % Define transformation from chassis frame to arm base frame (Tb0)
    %--------------------------------------------------------------------------
    Tb0 = [1, 0, 0, 0.1662;  % Fixed transformation between chassis and arm base
           0, 1, 0, 0;
           0, 0, 1, 0.0026;
           0, 0, 0, 1];

    %--------------------------------------------------------------------------
    % Extract joint angles from the current configuration
    %--------------------------------------------------------------------------
    thetalist = current(4:8)';  % Joint angles [theta1, theta2, theta3, theta4, theta5]

    %--------------------------------------------------------------------------
    % Extend F to a 6×4 matrix (F6x6) for base Jacobian computation
    %--------------------------------------------------------------------------
    F6x6 = [zeros(2, 4); F; zeros(1, 4)];  % Maps wheel velocities to chassis twist in 6D

    %--------------------------------------------------------------------------
    % Compute forward kinematics to get T0e (arm base to end-effector frame)
    %--------------------------------------------------------------------------
    T0e = FKinBody(M, Blist, thetalist);  % Current end-effector config relative to arm base
    
    Tse = CurrentToTse(current);

    Tse_d = RowToTransform(Tse_d_row);
    Tse_d_next = RowToTransform(Tse_d_next_row);


    %-----------------------------
    % 1) Compute feedforward twist V_d
    %-----------------------------
    % Compute the desired twist from T_se_d to T_se_d_next over dt
    X_d_mat = MatrixLog6(TransInv(Tse_d) * Tse_d_next);  % 4×4 matrix logarithm (se(3))
    X_d     = se3ToVec(X_d_mat);                           % 6×1 desired configuration change
    V_d     = X_d / dt;                                    % Feedforward twist (desired velocity)

    %-----------------------------
    % 2) Compute error twist X_err
    %-----------------------------
    % Compute the error between current and desired end-effector configuration
    X_err_mat = MatrixLog6(TransInv(Tse) * Tse_d);  % 4×4 matrix logarithm of error
    X_err     = se3ToVec(X_err_mat);                  % 6×1 error twist in end-effector frame

    %-----------------------------
    % 3) Update integral of error
    %-----------------------------
    % Integrate the error over time using Euler integration
    Xerrdt_int = Xerrdt_int + X_err * dt;  % Update global integrated error

    %-----------------------------
    % 4) Transform feedforward twist into end-effector frame & add feedback
    %-----------------------------
    % Compute the adjoint transformation to transform V_d into the current frame
    Ad_term = Adjoint(TransInv(Tse) * Tse_d);  % 6×6 adjoint matrix
    % Combine feedforward (V_d) and feedback (proportional + integral) terms
    V = Ad_term * V_d + Kp * X_err + Ki * Xerrdt_int;  % Commanded end-effector twist

    %--------------------------------------------------------------------------
    % Compute the arm Jacobian in the body frame
    %--------------------------------------------------------------------------
    Jarm = JacobianBody(Blist, thetalist);  % 6×5 Jacobian for the manipulator

    %--------------------------------------------------------------------------
    % Compute the base Jacobian (Jbase) for the mobile base
    %--------------------------------------------------------------------------
    Jbase = Adjoint(TransInv(T0e) * TransInv(Tb0)) * F6x6;  % 6×4 base Jacobian

    %--------------------------------------------------------------------------
    % Combine arm and base Jacobians into the full end-effector Jacobian (Je)
    %--------------------------------------------------------------------------
    Je = [Jbase Jarm];  % 6×9 full Jacobian (5 arm joints + 4 wheels)

    [mu_det_w, mu_det_v] = compute_manipulability(Je);

    %--------------------------------------------------------------------------
    % Compute pseudoinverse of Je and solve for velocities
    %--------------------------------------------------------------------------
    Jeinv = pinv(Je, 1e-4);  % Compute pseudoinverse with tolerance 1e-4
    result = Jeinv * V;      % Solve for wheel and joint velocities

    %--------------------------------------------------------------------------
    % Extract wheel and joint velocities from the result
    %--------------------------------------------------------------------------
    wheel_vel  = result(1:4)';   % Wheel velocities (u1, u2, u3, u4)
    joints_vel = result(5:9)';   % Joint velocities (theta_dot1, ..., theta_dot5)
    X_err = X_err';

end

function T = RowToTransform(row)
    R_flat = row(1:9);
    R = reshape(R_flat, 3, 3)';
    p = row(10:12)';  
    T = [R, p;
         0, 0, 0, 1];
end

function [mu_w, mu_v] = compute_manipulability(J)
    % COMPUTE_MANIPULABILITY - Computes manipulability measures for a given Jacobian
    % 
    % Inputs:
    %   J - The manipulator Jacobian (6×n matrix)
    %
    % Outputs:
    %   mu_det_w - Determinant-based manipulability index for angular velocity
    %   mu_det_v - Determinant-based manipulability index for linear velocity
    %   mu_svd_w - Smallest singular value-based manipulability index for angular velocity
    %   mu_svd_v - Smallest singular value-based manipulability index for linear velocity

    % Extract angular and linear velocity Jacobian parts
    A_w = J(1:3, :);  % First 3 rows (angular velocity part)
    A_v = J(4:6, :);  % Last 3 rows (linear velocity part)

    % Eigenvalue-based manipulability (use A_w * A_w' to get a 3x3 matrix)
    [v_Aw, D_Aw] = eig(A_w * A_w'); % 3x3 matrix
    [v_Av, D_Av] = eig(A_v * A_v'); % 3x3 matrix
    lambda_Aw_max = max(diag(D_Aw));
    lambda_Aw_min = min(diag(D_Aw));
    lambda_Av_max = max(diag(D_Av));
    lambda_Av_min = min(diag(D_Av));
    
    mu_w = sqrt(lambda_Aw_max / lambda_Aw_min); % Angular condition number
    mu_v = sqrt(lambda_Av_max / lambda_Av_min); % Linear condition number
    % Determinant-based manipulability measure
    mu_det_w = sqrt(det(A_w * A_w'));  % Angular manipulability
    mu_det_v = sqrt(det(A_v * A_v'));  % Linear manipulability

end
