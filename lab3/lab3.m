%% Lab 3
% Define DH: (angles are in radians and lengths are cm)
DH = [0  76     0    pi/2;
      0 -23.65 43.23 0;
      0   0     0    pi/2;
      0  43.18  0   -pi/2;
      0   0     0    pi/2;
      0  20     0    0];

% define myrobot
myrobot = mypuma560(DH);

H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
q1 = inverse(H1,myrobot);

% This is the starting joint variable vector.
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);

%% 3.1 The Attractive Field

% This is the final joint variable vector
tau = att(q1,q2,myrobot);

disp("Tau expected to be -0.9050 -0.0229 -0.4003 -0.0977 0.1034 0.0000");
disp(tau)

%% 3.2 Motion Planning without Obstacles

qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';

figure(1)
plot(myrobot,q)
disp("Please do not close figure 1 window. It will interfere with the running of figure 2")

%% 3.3 Motion Planning with Obstacles

setupobstacle

% test cyl
q3 = 0.9*q1+0.1*q2;
tau = rep(q3,myrobot,obs{1});
disp("Tau expected to be: 0.9950 0.0291 -0.0504 0.0790 0.0197 0.0000");
disp(tau)

% test sph
q = [pi/2 pi 1.2*pi 0 0 0]';
disp("Tau expected to be: -0.1138 -0.2140 -0.9702 0 -0.0037 0");
tau = rep(q, myrobot, obs{6});
disp(tau)


% plot
figure(2)
hold on;
axis([-100 100 -100 100 0 200]);
view(-32, 50);
plotobstacle(obs);
disp("Motion planning expected time: ~1:30 min");
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);
hold off;