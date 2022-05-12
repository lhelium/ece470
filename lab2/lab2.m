%Parameters
vel = 0.04; % Set velocity 

% Configuration Testing
% End Effector Postion
X1=[529.17 -104.12 82.73]'; 
X2=[705.19 -200.26 82.73]';
X3=[616.33 184.42 83.62]';

% Joint angles corresponding to the above end effector positions
Q1=[-0.2466    0.9236   -0.5339   -0.2651    1.2017    0.0983]';
Q2=[-0.3330    0.5142    0.2721   -0.4524    0.8461    0.3129]';
Q3=[0.3985    0.7035   -0.1018    0.4695    1.0297   -0.2606]';

% Calibrate DH Table for robot
delta = fminunc(@deltajoint, [0 0]);
my_robot = mykuka_search(delta);

% Calculate average offset in y between the calculated end effector
% position and the actual end effector position
offset1 = -104.12 - (-112.84);
offset2 = -200.26 - (-208.81) ;
offset3 = 184.42 - 173.92;
average_offset = (offset1+offset2+offset3)/3 ;

% Define end effector rotation
R0_6 = [0  0  1;
        0 -1  0;
        1  0  0];
    
% Define homogenous transformation for end effector
H = [R0_6 X1;0 0 0 1];

% Calculate the joint angles using inverse kinematics
q = inverse_kuka(H, my_robot);

% Part 4.5: Command the robot to draw a line segment and a circle
mysegment(my_robot)
mycircle(my_robot)
% Note: We didn't have enough time in the lab to draw the circle and the
% creative pattern
