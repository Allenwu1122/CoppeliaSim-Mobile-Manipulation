function nextstate = NextState(currState, jointWheelVels, dt, maxVel)
    
    % Extract current state variables
    chassisConfig = currState(1:3); % Chassis configuration (theta, x, y)
    armAngles = currState(4:8); % Arm joint angles
    wheelAngles = currState(9:12); % Wheel angles
    
    armSpeeds = jointWheelVels(1:5); % Arm joint speeds
    wheelSpeeds = jointWheelVels(6:9); % Wheel speeds

    newArmAngles = armAngles + armSpeeds * dt;
    newWheelAngles = wheelAngles + wheelSpeeds * dt;

    % Update chassis configuration (assuming a simple integration)
    % Odometry Calculations
    w = 0.15;
    l = 0.235;
    r = 0.0475;
    lw_forF = 1/(l+w);
    F = (r/4)*  [-lw_forF, lw_forF, lw_forF, -lw_forF;
                1, 1, 1, 1;
               -1,1,-1, 1];

    Vb = F*wheelSpeeds';

    if Vb(1) == 0
        delta_qb = Vb;
    else
        delta_qb = [Vb(1); (Vb(2)*sin(Vb(1))+Vb(3)*(cos(Vb(1))-1))/Vb(1);
                    (Vb(3)*sin(Vb(1))+Vb(2)*(1-cos(Vb(1))))/Vb(1)]; 
    end

    phi_k = chassisConfig(1);
    delta_q = [1  0   0;
       0  cos(phi_k) -sin(phi_k);
       0  sin(phi_k)  cos(phi_k)]*delta_qb;
    
    % Updated Odometry
    newChassisConfig = chassisConfig'+delta_q; % Modify based on kinematics if needed
    
    % Concatenate next state
    nextstate = [newChassisConfig', newArmAngles, newWheelAngles];
end