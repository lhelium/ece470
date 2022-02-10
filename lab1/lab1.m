% Lab 1
DH = [0 76 0 pi/2;
       0 -23.65 43.23 0;
       0 0 0 pi/2;
       0 43.18 0 -pi/2;
       0 0 0 pi/2;
       0 20 0 0];

%4.1 Definition of Robot Structure
myrobot = mypuma560(DH);

%4.2 Plotting the Trajectory
theta_1_range = linspace(0, pi, 200);
theta_2_range = linspace(0, pi/2, 200);
theta_3_range = linspace(0, pi, 200);
theta_4_range = linspace(pi/4, 3/4*pi, 200);
theta_5_range = linspace(-pi/3, pi/3, 200);
theta_6_range = linspace(0, 2*pi, 200);

q = [theta_1_range' theta_2_range' theta_3_range' theta_4_range' theta_5_range' theta_6_range'];

figure(1)
plot(myrobot, q);

%4.3 Forward Kinematics
o = zeros(200, 3);
for i = 1:200
    H = forward(q(i, :), myrobot);
    o(i, :) = H(1:3, 4);
end
  
figure(2)
plot3(o(:, 1), o(:, 2), o(:, 3), "r");
hold on 
plot(myrobot, q)

%4.4 Inverse Kinematics
H_test = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
q_test = inverse(H_test, myrobot);
q_exp =[ -0.0331 -1.0667 1.0283 3.1416 3.1032 0.8185];

disp(q_test);

assert(norm(q_test - q_exp) < 1e-3, "inverse function incorrect!");

x_lin = linspace(10, 30, 100);
y_lin = linspace(23, 30, 100);
z_lin = linspace(15, 100, 100);
d = [x_lin' y_lin' z_lin'];

rot_z_pi_4 = [cos(pi/4) -sin(pi/4) 0;
              sin(pi/4)  cos(pi/4) 0;
                 0          0      1];
q_4 = zeros(100, 6);

for i = 1:100
    H_fun = [[rot_z_pi_4 d(i,:)']; 0 0 0 1];
    q_4(i, :) = inverse(H_fun, myrobot);
end

figure(3)
plot3(d(:, 1), d(:, 2), d(:, 3), "r");
hold on 
plot(myrobot, q_4)