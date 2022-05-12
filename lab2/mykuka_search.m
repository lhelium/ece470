function myrobot = mykuka_search(delta)
    % robot object with kinematics defined by the matrix dh which has one row per joint and each row is [theta d a alpha]
    DH = [0     400     25    pi/2;
          pi/2    0    315    0;
          0       0     35    pi/2;
          0     365      0   -pi/2;
          pi/2    0      0    pi/2;
          0     161.44+delta(2) -(296.23+delta(1)) 0];
    myrobot = SerialLink(DH);
    
end
