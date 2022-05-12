function mycircle(my_robot)
    % Define parameters
    r = 50;
    c = [620 0 -1]';

    %offsety = 9 ;
   
    % create X_workspace
    X_workspace = [zeros(1, 100); zeros(1, 100); zeros(1, 100)];
    for i = 1:100
        theta = 2*pi/99*(i-1);
        X_workspace(1,i) = c(1) + r * cos(theta);
        X_workspace(2,i) = c(2) + r * sin(theta);
        X_workspace(3,i) = c(3);
    end
    
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
    %figure()
    %plot3(x_baseframe_movement(:, 1), x_baseframe_movement(:, 2), x_baseframe_movement(:, 3), "r");
    %hold on 
    %plot(my_robot, q_movement); % plot robot movement since we were unable to complete this in lab 
    
end