%{
Parameters:
    q   := 6x1 initial joint var
    obs := 6 obstacle data objects
    myrobot := robot seriallink
    idx := which joint to compute
    eta := repulsive constant
Return:
    force := 3x1 repulsive force on joint idx
%}
function force = get_F_i_rep(q, obs, myrobot, idx, eta)
    % o_q
    h_idx  = forward_idx(q, myrobot, idx);
    o_idx  = h_idx(1:3, 4);
    
    % check obs
    rho0 = obs.rho0;
    type = obs.type;

    % check whether cylinder or plane
    % rho_vector := o - b
    if type == 'cyl'
        R  = obs.R;
        h  = obs.h;
        cx = obs.c(1);
        cy = obs.c(2);

        if o_idx(3) <= h
            cz = o_idx(3);
        else
            cz = h;
        end
        
        denom = sqrt((o_idx(1) - cx)^2 + (o_idx(2) - cy)^2);        
        b = [cx + R * (o_idx(1) - cx)/denom; 
             cy + R * (o_idx(2) - cy)/denom;
             cz];
        
        rho_vector = o_idx - b;

    else % type == 'pla'
        h = obs.h;
        b = [o_idx(1); o_idx(2); h];
        rho_vector = o_idx - b;
    end

    % force calculation
    rho = norm(rho_vector);
    if rho <= rho0
        del_rho = rho_vector / rho;
        force = eta * (1/rho - 1/rho0) / rho^2 * del_rho;
    else
        force = zeros(3,1);
    end
end