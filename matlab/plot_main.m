%% Part A : Plot accelerometers values 
clc; clear all; close all;
name = "localization.csv";
[N_SIM, T_SIM, T, data] = read_log(name);

f = figure('Name','accelerometer [m/s^2]');


% Plot x acceleration 
subplot(3,1,1);
plot(data.time , data.acc(:,1));
title('acc[0]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min( data.acc(:,1)),max(data.acc(:,1))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot y acceleration
subplot(3,1,2);
plot(data.time , data.acc(:,2));
title('acc[1]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,2)),max(data.acc(:,2))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

% Plot z acceleration
subplot(3,1,3);
plot(data.time , data.acc(:,3));
title('acc[2]');
xlabel('Time [s]'); ylabel('acc [m/s^2]');
y_lim = [min(data.acc(:,3)),max(data.acc(:,3))];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part B : Compute and plot the odometry computed using the accelerometer

[N_SIM, T_SIM, T, data] = read_log(name);

odo.x       = zeros(N_SIM + 1,1);
odo.vx      = zeros(N_SIM + 1,1);

% Compute the static biais (mean)
acc_mean = 0.01; %as the trajectories does not allow calibration, the mean acc is considered as 0
acc_mean = mean(data.acc(1:313,:));
% Compute the odometry
for t_ = T_SIM
    [odo.x(t_ + 1), odo.vx(t_ + 1)] = ...
    odo_acc(odo.x(t_), odo.vx(t_), T, data.acc(t_,:), acc_mean);
end

% Plot the odometry computed using the accelerometer
f = figure('Name','Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(data.time, data.pose_x); hold on;
plot(data.time, odo.x(2:end));
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([odo.x(2:end);  data.pose_x]),max([odo.x(2:end);  data.pose_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part C : Plot the odometry computed using the accelerometer on Webots

[N_SIM, T_SIM, T, data] = read_log(name);

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using accelerometer [m/s^2]');

% Plot x : odometry vs ground truth (gps)
plot(data.time, data.pose_x); hold on;
plot(data.time, data.odo_acc_x);
title('x trajectory : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Accelerometer');
xlabel('Time [s]'); ylabel('x [m]');
y_lim = [min([data.odo_acc_x;  data.pose_x]),max([data.odo_acc_x;  data.pose_x])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part D : Compute and plot the odometry computed using the wheel encoders

[N_SIM, T_SIM, T, data] = read_log(name);

odo.x       = zeros(N_SIM ,1);
odo.y       = zeros(N_SIM ,1);
odo.heading = zeros(N_SIM ,1);

% Compute Delta wheel encoders
A_enc_r = diff(data.right_enc);
A_enc_l = diff(data.left_enc); 

% Compute the odometry
for t_ = T_SIM(1:end-1)
    [odo.x(t_ + 1), odo.y(t_ + 1), odo.heading(t_ + 1)] = ...
     odo_enc(odo.x(t_), odo.y(t_), odo.heading(t_), T, A_enc_r(t_), A_enc_l(t_));
end

% Plot the odometry computed using the accelerometer
f = figure('Name','Odometry using wheel encoders [Rad]'); 
subplot(3,1,[1 2]);hold on;

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.pose_x(2:end) , data.pose_y(2:end)); hold on;
plot(odo.x(2:end) , odo.y(2:end));
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([odo.x(2:end);  data.pose_x]),max([odo.x(2:end);  data.pose_x])];
y_lim = [min([odo.y(2:end);  data.pose_y]),max([odo.y(2:end);  data.pose_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data.time ,  data.pose_heading); hold on;
plot(data.time , wrapToPi(odo.heading));
title('Heading : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(odo.heading(2:end));  data.pose_heading]),max([wrapToPi(odo.heading(2:end));  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Part E : Plot the odometry computed using the wheel encoders on Webots

[N_SIM, T_SIM, T, data] = read_log(name);

% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : Odometry using wheel encoders [Rad]'); 
subplot(3,1,[1 2]);hold on;

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.pose_x(2:end) , data.pose_y(2:end)); hold on;
plot(data.odo_enc_x , data.odo_enc_y);
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.odo_enc_x;  data.pose_x]),max([data.odo_enc_x;  data.pose_x])];
y_lim = [min([data.odo_enc_y;  data.pose_y]),max([data.odo_enc_y;  data.pose_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data.time ,  data.pose_heading); hold on;
plot(data.time , wrapToPi(data.odo_enc_heading));
title('Heading : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(data.odo_enc_heading);  data.pose_heading]),max([wrapToPi(data.odo_enc_heading);  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
%% Part F : Plot kalman computed on Webots
clc; clear all; close all;
name = "localization.csv";
name_true = "datatrue.csv";
[N_SIM, T_SIM, T, data] = read_log(name);
[N_SIMsup, T_SIMsup, T, datasup] = read_log(name_true);

if (N_SIM ~= N_SIMsup ||  T_SIMsup ~= T_SIMsup)
    disp("not from the same simulation")
end




% Plot the odometry computed using the accelerometer
f = figure('Name','Webots : kalman [Rad]'); 
subplot(3,1,[1 2]);hold on;

% Plot x -y plan : odometry vs ground truth (gps)
plot(data.pose_x(2:end) , data.pose_y(2:end)); hold on;
plot(data.kalman_x , data.kalman_y);
title('x -y plan : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data.odo_enc_x;  data.pose_x]),max([data.odo_enc_x;  data.pose_x])];
y_lim = [min([data.odo_enc_y;  data.pose_y]),max([data.odo_enc_y;  data.pose_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data.time ,  data.pose_heading); hold on;
plot(data.time , wrapToPi(data.odo_enc_heading));
title('Heading : odometry vs ground truth (gps)');
legend('Ground Thruth : GPS', 'Odometry : Wheel encoders');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(data.odo_enc_heading);  data.pose_heading]),max([wrapToPi(data.odo_enc_heading);  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
