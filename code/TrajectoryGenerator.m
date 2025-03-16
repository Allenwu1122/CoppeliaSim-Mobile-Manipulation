function trajectory = TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, Tce_place, k)
    % Define trajectory segments
    % 1. Move to standoff above initial position
    T_se_standoff_initial = T_sc_initial * T_ce_standoff;
    traj1 = GenerateTrajectory(T_se_initial, T_se_standoff_initial, k(1));
    
    % 2. Move down to grasp position
    T_se_grasp = T_sc_initial * T_ce_grasp;
    traj2 = GenerateTrajectory(T_se_standoff_initial, T_se_grasp, k(2));
%     
    % 3. Close gripper (1 = closed)
    traj3 = AppendGripperState(traj2(end, :), 1, k(3));
    
    % 4. Move back to standoff position
    traj4 = GenerateTrajectory(T_se_grasp, T_se_standoff_initial, k(4));
    traj4(:, end) = 1;
    
    % 5. Move to standoff above final position
    T_se_standoff_final = T_sc_final * T_ce_standoff;
    traj5 = GenerateTrajectory(T_se_standoff_initial, T_se_standoff_final, k(5));
    traj5(:, end) = 1;
    
    % 6. Move down to final grasp position
    T_se_final_grasp = T_sc_final * Tce_place;
    traj6 = GenerateTrajectory(T_se_standoff_final, T_se_final_grasp, k(6));
    traj6(:, end) = 1;
    
    % 7. Open gripper (0 = open)
    traj7 = AppendGripperState(traj6(end, :), 0, k(7));
    
    % 8. Move back to standoff above final position
    traj8 = GenerateTrajectory(T_se_final_grasp, T_se_standoff_final, k(8));
    
    % Concatenate all trajectory segments
    %trajectory = [traj1; traj2; traj3; traj4; traj5; traj6];
    trajectory = [traj1; traj2; traj3; traj4; traj5; traj6; traj7; traj8];

end

function traj = GenerateTrajectory(T_start, T_end, k)
    % Interpolate transformation matrices
    numSteps = k * 100; % Assuming 1 sec for each motion
    traj = zeros(numSteps, 13);
    for i = 1:numSteps
        s = i / numSteps;
        T_interp = T_start * expm(logm(T_start \ T_end) * s);
        traj(i, :) = TransformToRow(T_interp, 0);
    end
end

function traj = AppendGripperState(lastRow, gripperState, k)
    numSteps = k * 100;
    traj = repmat(lastRow, numSteps, 1);
    traj(:, end) = gripperState;
end

function row = TransformToRow(T, gripperState)
    R = T(1:3, 1:3);
    p = T(1:3, 4)';
    row = [reshape(R', 1, []) p gripperState];
end