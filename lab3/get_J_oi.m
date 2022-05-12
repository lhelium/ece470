%{
Parameters:
    q   := 6x1 initial joint var
    myrobot := robot seriallink
    idx := which joint to compute
Return:
    J := 3xn Jacobian up to joint idx
%}
function J = get_J_oi(q,myrobot,idx)
    % n := number of joints
    q_size = size(q);
    n = q_size(1);
    % J column by column
    J = zeros(3,n);
    for col = 1:idx
        % o_delta := o_idx - o_(col-1)
        h_idx  = forward_idx(q,myrobot,idx);
        o_idx  = h_idx(1:3,4);
        h_col_1 = forward_idx(q,myrobot,col-1);
        o_col_1 = h_col_1(1:3,4);
        o_delta = o_idx - o_col_1;
        % z_col_1
        z_col_1 = h_col_1(1:3,3);
        % J_col
        J(1:3,col) = cross(z_col_1,o_delta);
    end
end

