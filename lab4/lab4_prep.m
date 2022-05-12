%% Lab 4
% Define DH Table for Kuka robot: (angles are in radians and lengths are mm)
DH = [0     400     25    pi/2;
      pi/2    0    315    0;
      0       0     35    pi/2;
      0     365      0   -pi/2;
      pi/2    0      0    pi/2;
      0     161.44  -156  0];
  
DH_forces = [0     400     25    pi/2;
             pi/2    0    315    0;
             0       0     35    pi/2;
             0     365      0   -pi/2;
             pi/2    0      0    pi/2;
             0     161.44   0    0]; % set a6 = 0 for better attractive and repulsive force results
% define myrobot
myrobot = mykuka(DH);

% define myforces
myforces = mykuka(DH_forces);



%% Prelab
% Testing repulsive forces
disp("PRELAB")
disp("Test repulsive forces")
setupobstacle_lab4prep;
expected_tau = [0.1795 0.9540 0.2353 -0.0344 -0.0344 0.0000];
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], myforces, prepobs{1});
disp("Tau expected value: ");
disp(expected_tau);
disp("Tau actual value: ");
disp(tau);
diff = abs(expected_tau - tau);
if diff < 1e-4
    diff = zeros(1, 6);
end
disp("Difference: ");
disp(diff);

% Test motion planning
p1 = [620 375 50];
p2 = [620 -375 50];
R = [0 0 1;0 -1 0;1 0 0];
H1 = [R p1';zeros(1,3) 1];
H2 = [R p2';zeros(1,3) 1];
q1 = inverse_kuka(H1, myrobot);
q2 = inverse_kuka(H2, myrobot);
qref = motionplan(q1, q2, 0, 10, myforces, prepobs, 0.01); % using t1, t2, tol values from lab 3
t=linspace(0,10,300);
q = ppval(qref,t)';

hold on;
disp("Test motion planning")
plotobstacle(prepobs);
plot(myrobot,q);