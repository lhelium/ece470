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

%% Lab 4.1
offset = 137;  % robot frame had a large offset
z_grid = 45 + offset;
p0 = [370 -440 150 + offset];
p1 = [370 -440 z_grid];
p2 = [750 -220 225 + offset];
p3 = [620 350 225 + offset];
R  = [0 0 1;0 -1 0;1 0 0];

setupobstacle;

% home config
qh = [0 pi/2 0 0 pi/2 0]';
% q0 config
H0 = [R p0';0 0 0 1];
q0 = inverse_kuka(H0, myrobot);
% q1 config
H1 = [R p1';0 0 0 1];
q1 = inverse_kuka(H1, myrobot);
% q2 config
H2 = [R p2';0 0 0 1];
q2 = inverse_kuka(H2, myrobot);
% q3 config
H3 = [R p3';0 0 0 1];
q3 = inverse_kuka(H3, myrobot);

% motionplan
q_h0 = motionplan(qh, q0, 0, 10, myforces, obs, 0.15);
q_01 = motionplan(q0, q1, 0, 10, myforces, obs, 0.15);
q_12 = motionplan(q1, q2, 0, 10, myforces, obs, 0.15);
q_23 = motionplan(q2, q3, 0, 10, myforces, obs, 0.15);

% t linspace
t=linspace(0,10,300);

% ppval
plot_h0 = ppval(q_h0,t)';
plot_01 = ppval(q_01,t)';
plot_12 = ppval(q_12,t)';
plot_23 = ppval(q_23,t)';

% plot
disp("Test motion planning")
figure(1)
hold on;
plotobstacle(obs);
plot(myrobot,plot_h0);
 
figure(2)
hold on
plotobstacle(obs);
plot(myrobot,plot_01);

figure(3)
hold on
plotobstacle(obs);
plot(myrobot,plot_12);

figure(4)
% Note: in lab this function resulted in the robot travelling through one
% of the obstacles. This was found to be the result of the a_att being set
% too high. Setting a_att=0.013 and a_rep=0.013 results in a valid motion
% plan
hold on
plotobstacle(obs);
plot(myrobot,plot_23);

%% Lab 4.2
% Execute motion plan on robot

% vel = 0.04;
% setHome(vel);
% setGripper(0)
% from qh to q0
% for i = 1:300
%     setAngles(plot_h0(i,:),vel)
% end
% from q0 to q1
% setAngles(q1,vel)
% setGripper(1)
% from q1 to q2
% for i = 1:300
%     setAngles(plot_12(i,:),vel)
% end
% from q2 to q3
% for i = 1:300
%     setAngles(plot_23(i,:),vel)
% end
% setGripper(0)

%% Lab 4.3

% For the creative component of the lab, we chose to make a new path for 
% our robot to follow. We planned a path so that the robot straightens 
% out in between motions making it look like a ballet dancer.

offset = 137;
z_grid = 45 + offset;
p0 = [370 -440 150 + offset];
p1 = [370 -440 z_grid];
p2 = [750 -220 225 + offset];
p3 = [620 350 225 + offset];
R  = [0 0 1;0 -1 0;1 0 0];

setupobstacle;

% home config
qh = [0 pi/2 0 0 pi/2 0]';
% q0 config
H0 = [R p0';0 0 0 1];
q0 = inverse_kuka(H0, myrobot);
% q1 config
H1 = [R p1';0 0 0 1];
q1 = inverse_kuka(H1, myrobot);
% q2 config
H2 = [R p2';0 0 0 1];
q2 = inverse_kuka(H2, myrobot);
% q3 config
H3 = [R p3';0 0 0 1];
q3 = inverse_kuka(H3, myrobot);

% motionplan
q_h0 = motionplan_ballet_dancer(qh, q0, 0, 10, myforces, obs, 0.15);
q_01 = motionplan_ballet_dancer(q0, q1, 0, 10, myforces, obs, 0.15);
q_12 = motionplan_ballet_dancer(q1, q2, 0, 10, myforces, obs, 0.15);
q_23 = motionplan_ballet_dancer(q2, q3, 0, 10, myforces, obs, 0.15);

% t linspace
t=linspace(0,10,300);

% ppval
plot_h0 = ppval(q_h0,t)';
plot_01 = ppval(q_01,t)';
plot_12 = ppval(q_12,t)';
plot_23 = ppval(q_23,t)';

% plot (?)
disp("Test motion planning")
figure(1)
hold on;
plotobstacle(obs);
plot(myrobot,plot_h0);

figure(2)
hold on
plotobstacle(obs);
plot(myrobot,plot_01);

figure(3)
hold on
plotobstacle(obs);
plot(myrobot,plot_12);

figure(4)
hold on
plotobstacle(obs);
plot(myrobot,plot_23);