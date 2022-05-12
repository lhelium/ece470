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
function force = get_F_i_rep(q,obs,myrobot,idx,eta)
    % o_q
    h_idx  = forward_idx(q,myrobot,idx);
    o_idx  = h_idx(1:3,4);
    % check obs
    R =  obs.R;
    c =  obs.c;
    rho0 = obs.rho0;
    type = obs.type;

    % check wether sphere or cylinder
    % rho_vector := o - b
    if     type == 'sph'
        rho_vector = (o_idx - c) * ( 1 - R / norm(o_idx - c));

    elseif type == 'cyl'
        rho_vector = [(o_idx(1:2) - c); 0] * ( 1 - R / norm([(o_idx(1:2) - c); 0]) );
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