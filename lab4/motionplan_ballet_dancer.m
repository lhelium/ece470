function qref = motionplan_ballet_dancer(q0,q2,t1,t2,myrobot,obs,tol) 
    % home
    q_m1 = [q0(1) pi/2 0 0 pi/2 0]';
    q_m2 = [q2(1) pi/2 0 0 pi/2 0]';
    
    q_mat = [q0 q_m1 q_m2 q2];


    % transpose
    q_mat = q_mat';
    
    %make a spline
    t = linspace(t1, t2, size(q_mat, 1));
    qref = spline(t, q_mat');
end

