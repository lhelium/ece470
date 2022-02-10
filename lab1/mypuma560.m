function myrobot = mypuma560(DH)
    % theta d a alpha
    myrobot = SerialLink(DH); % robot object with kinematics defined by the matrix dh which has one row per joint and each row is [theta d a alpha]
    %L(1) = Link(DH[1:], 'standard');
    %L(2) = Link(DH[2:], 'standard');
    %L(3) = Link(DH[3:], 'standard');
    %L(4) = Link(DH[4:], 'standard');
    %L(5) = Link(DH[5:], 'standard');
    %L(6) = Link(DH[6:], 'standard');
    
end
