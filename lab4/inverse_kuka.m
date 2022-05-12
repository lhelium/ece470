function q = inverse_kuka(H, myrobot)
    elbow = 1;  % +1 for elbow up, -1 for elbow down
    
    % Extract parameters 
    a = myrobot.a;
    d = myrobot.d;
    
    % extract Rd from H
    Rd = H(1:3, 1:3);
    o_d = H(1:3, 4);
    
    % calculate origin of wrist center frame
    o_c = o_d - Rd*[0 0 d(6)]';
    
    xc = o_c(1);
    yc = o_c(2);
    zc = o_c(3);
    
    % Calculate theta 1
    theta_1 = atan2(yc, xc) - atan2(-d(2), real(sqrt(xc^2 + yc^2 - d(2)^2)));    

    % Calculate sin(theta_3) = D
    D = (xc^2 + yc^2 - d(2)^2 + (zc - d(1))^2 - a(2)^2 - d(4)^2)/(2 * a(2) * d(4));
    % Calculate theta_3
    theta_3 = atan2(D, elbow*real(sqrt(1-D^2)));
    
    % Calculate theta_2
    theta_2 = atan2(zc - d(1), real(sqrt(xc^2 + yc^2 - d(2)^2))) - atan2(-d(4)*cos(theta_3), a(2) + d(4)*D);
    
    %compute R_3_0 with same method and forward kinematics but with
    %rotational matrices instead of homogenous matrices
    R = eye(3, 3);
    thetas = [theta_1 theta_2 theta_3]; 
    for i = 1:3 % iterate through the 3 joints
        theta = thetas(i);
        alpha = myrobot.alpha(i);
        
        % Define Rotational Matrix
        R_i = [ cos(theta) -sin(theta)*cos(alpha)   sin(theta)*sin(alpha);
                sin(theta)  cos(theta)*cos(alpha)  -cos(theta)*sin(alpha);
                    0            sin(alpha)                  cos(alpha)];
        R = R * R_i;
    end
    
    R_3_0 = R;
    
    % calculate R_6_3: R_6_3 = R_3_0T * Rd
    R_6_3 = R_3_0' * Rd;
    M = R_6_3;
    
    if (M(1, 3)^2 + M(2, 3)^2) ~= 0  % check that it is not a singularity
        % thetas 4, 5, and 6 correspond to the ZYZ Euler Angles, use the
        % equations derrived for the Euler angles
        theta_4 = atan2(elbow * M(2, 3), elbow * M(1, 3));
        theta_5 = atan2(elbow * real(sqrt(1-M(3, 3)^2)), elbow * M(3, 3));
        theta_6 = atan2(elbow * M(3, 2), -1*elbow * M(3, 1));
        q = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]';
    else
        disp("Error: Singularity! Yikes!")
        q = [0 0 0 0 0 0]';
    end
    
end