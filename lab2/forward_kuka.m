function H = forward_kuka(q, myrobot)  % Forward Kinematics
    H = eye(4,4); % base case for iteration
    for i = 1:6 % iterate through the 6 joints
        % extract parameters from input
        theta = q(i);
        alpha = myrobot.alpha(i);
        a = myrobot.a(i);
        d = myrobot.d(i);
        
        % define homogenous matrix for frame i 
        H_i = [ cos(theta) -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)   a*cos(theta);
                sin(theta)  cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)   a*sin(theta);
                0               sin(alpha)               cos(alpha)               d;
                0                   0                         0                   1];
        H = H * H_i; % multiply with previous homogenous matrix
    end
end