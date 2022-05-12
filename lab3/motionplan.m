function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol) 
    q = q0;
    q_delta = norm(q(1:5) - q2(1:5));
    q_mat = q;
    
    % alpha scales the gradiant descent
    alpha = 0.02;
    
    % iter counter
    iter = 0;
    while (q_delta>=tol)
        tau_att = att(q,q2,myrobot)';
        tau_rep = zeros(size(tau_att));
        for k = 1:length(obs)
            temp = rep(q,myrobot,obs{k})';
            tau_rep = tau_rep + temp;
        end
        q = q + alpha*(tau_att+tau_rep);
        q_delta = norm(q(1:5) - q2(1:5));
        q_mat = [q_mat q];
        iter = iter +1;
        
        if iter > 5000
            disp('stuck! break!');
            break;
        end
    end
    
    q_mat = q_mat';
    
    %make a spline
    t = linspace(t1, t2, size(q_mat, 1));
    qref = spline(t, q_mat');
end

