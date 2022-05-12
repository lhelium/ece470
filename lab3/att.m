function tau = att(q,q2,myrobot)
    % zet := spring constant
    zet_metre = 1;
    zet = zet_metre/100;
    % n := number of joints
    q_size = size(q);
    n = q_size(1);
    % tau := a ROW vector
    tau = zeros(1,n);

    for i = 1:6
        % J calculation
        J = get_J_oi(q,myrobot,i);

        % F calculation
        F = get_F_i_att(q,q2,myrobot,i,Inf,zet);

        % tau calculation
        J_F = J' * F;
        tau = tau + J_F';
    end
    
    if tau == zeros(1,n)
        %disp('tau is zeros');
    else
        tau = tau / norm(tau);
    end
end