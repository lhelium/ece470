function myrobot = mykuka(DH)
    % theta d a alpha
    myrobot = SerialLink(DH); % robot object with kinematics defined by the matrix dh which has one row per joint and each row is [theta d a alpha]
    
end
