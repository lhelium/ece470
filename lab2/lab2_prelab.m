% Lab 2 Prelab: Q3 and Q4
% Define base DH matrix of robot representing the 6 frames of the robot
% (angles are in radians and lengths are mm)
% Each row is [theta d a alpha]
DH = [0  400    25   -pi/2;
      0   0    315     0;
      0   0     35    pi/2;
      0  365    0     pi/2;
      0   0     0     pi/2;
      0  161.44 296.23 0];

% Definition of Robot Structure
kuka = mykuka(DH);

% Forward kinematics
forward = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka);
disp(forward)

% Inverse kinematics
inverse = inverse_kuka(forward, kuka);
disp(inverse)