function tau = rep(q,myrobot,obs)
    % zet := repulsive constant
    eta_metre = 1;
    eta = eta_metre/100;
    % n := number of joints
    q_size = size(q);
    n = q_size(1);
    
    % tau := a ROW vector
    tau = zeros(1,n);
    if ~isempty(obs)
        for i = 1:6
            % J calculation
            J = get_J_oi(q,myrobot,i);

            % F calculation
            F = get_F_i_rep(q,obs,myrobot,i,eta);

            % tau calculation
            J_F = J' * F;
            tau = tau + J_F';
        end
    end
    
    if tau == zeros(1,n)
        %disp('tau is zeros'); % for debugging
    else
        tau = tau / norm(tau);
    end
end