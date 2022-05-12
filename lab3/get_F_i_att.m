%{
Parameters:
    q   := 6x1 initial joint var
    q2  := 6x1 final joint var
    myrobot := robot seriallink
    idx := which joint to compute
    d   := saturation distance
    zet := spring constant
Return:
    force := 3x1 attractive force on joint idx
%}
function force = get_F_i_att(q,q2,myrobot,idx,d,zet)
    % o_delta := o_q - o_q2
    h_q  = forward_idx(q,myrobot,idx);
    o_q  = h_q(1:3,4);
    h_q2 = forward_idx(q2,myrobot,idx);
    o_q2 = h_q2(1:3,4);
    o_delta = o_q - o_q2;
    % check whether greater than d
    o_delta_norm = norm(o_delta);
    if o_delta_norm <= d
        force = - zet * o_delta;
    else
        force = - d * zet * o_delta / o_delta_norm;
    end
end