function mysegment(my_robot)
    % Define Fixed Parameters
    vel = 0.04;
    x_bar = 620; 
    z_bar = -3; % account for z-offset (determined in part 4.3) to ensure contact with paper
    
    % create X_workspace
    X_workspace = [ones(1, 100)*x_bar; linspace(-100,100,100); ones(1, 100)*z_bar];
    
    % Define rotation matrix for end effector
    R0_6 = [0  0  1;
            0 -1  0;
            1  0  0];
    
    % Record path for optional plotting of robot motion
    q_movement = zeros(100, 6);
    x_baseframe_movement = zeros(100, 3); 
    
    % iterate through the movement
    for i = 1:100
        X_baseframe = FrameTransformation(X_workspace(:, i)); % Determine base frame
        x_baseframe_movement(i, :) = X_baseframe'; % Optional plotting of robot trajectory 
        H = [R0_6 X_baseframe;0 0 0 1]; % Define homogenous transformation
        q = inverse_kuka(H, my_robot); % Calculate joint angles using inverse kinematics
        setAngles(q, vel) % move robot
        q_movement(i, :) = q'; % Optional plotting of robot motion
    end
    
    %Uncomment Section to Plot robot motion
    %figure(1)
    %zlim([0 1550]);
    %plot3(x_baseframe_movement(:, 1), x_baseframe_movement(:, 2), x_baseframe_movement(:, 3), "r");
    %holdon
    %plot(my_robot, q_movement); % plot robot movemnet since we were unable to complete this in lab 
end