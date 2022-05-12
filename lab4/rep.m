function tau = rep(q, myrobot, obs)
    % zet := repulsive constant
    %eta_metre = 0.1;
    %eta = eta_metre*100;
    eta = 0.1;
    % n := number of joints
    % n = size(q, 1);
    n = 6;
    
    if ~isempty(obs)
        
        % F calculation
        Frep_1 = get_F_i_rep(q, obs, myrobot, 1, eta);
        Frep_2 = get_F_i_rep(q, obs, myrobot, 2, eta);
        Frep_3 = get_F_i_rep(q, obs, myrobot, 3, eta);
        Frep_4 = get_F_i_rep(q, obs, myrobot, 4, eta);
        Frep_5 = get_F_i_rep(q, obs, myrobot, 5, eta);
        Frep_6 = get_F_i_rep(q, obs, myrobot, 6, eta);
        
        % J calculation
        % Pad Jo_i with 3x1 zeros vector to make each matrix 3x6
%         zeros3 = zeros(3, 1);
%         Jo_1 = [get_J_oi(q, myrobot, 1) zeros3 zeros3 zeros3 zeros3 zeros3];
%         Jo_2 = [get_J_oi(q, myrobot, 2) zeros3 zeros3 zeros3 zeros3];
%         Jo_3 = [get_J_oi(q, myrobot, 3) zeros3 zeros3 zeros3];
%         Jo_4 = [get_J_oi(q, myrobot, 4) zeros3 zeros3];
%         Jo_5 = [get_J_oi(q, myrobot, 5) zeros3];
%         Jo_6 = [get_J_oi(q, myrobot, 6)];
        
        Jo_1 = [get_J_oi(q, myrobot, 1)];
        Jo_2 = [get_J_oi(q, myrobot, 2)];
        Jo_3 = [get_J_oi(q, myrobot, 3)];
        Jo_4 = [get_J_oi(q, myrobot, 4)];
        Jo_5 = [get_J_oi(q, myrobot, 5)];
        Jo_6 = [get_J_oi(q, myrobot, 6)];
        
        tau = (Jo_1' * Frep_1) + (Jo_2' * Frep_2) + (Jo_3' * Frep_3) + (Jo_4' * Frep_4) + (Jo_5' * Frep_5) + (Jo_6' * Frep_6);
        
    end
    
    % So far all calculations with tau have been done as a COLUMN vector
    % Transpose tau before returning to return a ROW vector
    if norm(tau) == 0
        tau = zeros(1, n);
    else
        tau = tau / norm(tau);
        tau = tau';
    end
end