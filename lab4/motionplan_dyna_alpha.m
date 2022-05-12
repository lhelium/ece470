function qref = motionplan_dyna_alpha(q0,q2,t1,t2,myrobot,obs,tol) 
    q = q0;
    q_delta = norm(q(1:5) - q2(1:5));
    q_mat = q;
    
    % alpha scales the gradiant descent
    alpha_att = 0.013;
    alpha_rep = 0.01;
    % beta scales aphla according to change in q
    beta = 1;

    % iter counter
    iter = 0;
    while (q_delta>=tol)
        tau_att = att(q,q2,myrobot)';
        tau_rep = zeros(size(tau_att));
        
        for k = 1:length(obs)
            temp = rep(q,myrobot,obs{k})';
            tau_rep = tau_rep + temp;
        end
        
        q = q + beta * (alpha_att*tau_att + alpha_rep*tau_rep);
        q_delta = norm(q(1:5) - q2(1:5));
        q_mat = [q_mat q];
        iter = iter + 1;
        
        beta = 0.49 * tanh( 10 * ( q_delta - 0.25 ) ) + 0.51;
        
        disp(q_delta);

        if iter > 5000
            disp('stuck! break!');
            break;
        end
    end 

    % transition q6 by linspace
    q_mat(6,:) = linspace(q0(6),q2(6),(iter+1));

    % transpose
    q_mat = q_mat';
    
    %make a spline
    t = linspace(t1, t2, size(q_mat, 1));
    qref = spline(t, q_mat');
end

