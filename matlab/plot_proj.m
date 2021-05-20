clc; clear all; close all;
%name = "../Initial_Material/Initial_Material/controllers/localization_controller/localization.csv";
name = "localization.csv";
name_true = "datatrue.csv";
[N_SIM, T_SIM, T, data] = read_log(name);
data_true= readtable(name_true,"ReadVariableNames",0,"NumHeaderLines",0);



% Plot the odometry computed using the chosen pose
f = figure('Name','Webots : Estimation [Rad]'); 
subplot(3,1,[1 2]);hold on;


%chose the right data to plot
est_x = data.mode_x;
est_y = data.mode_y;
est_h = data.mode_heading;



% Plot x -y plan : odometry vs ground truth (supervisor)
plot(data_true.Var2 , -data_true.Var3); hold on;
plot(est_x , est_y,'--');
title('x -y plan : Estimation vs ground truth (supervisor)');
legend('Ground Thruth : supervisor', 'Estimated');
xlabel('x [m]'); ylabel('y [m]');
x_lim = [min([data_true.Var2;  est_x]),max([data_true.Var2;  est_x])];
y_lim = [min([data_true.Var3;  est_y]),max([data_true.Var3;  est_y])];
xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
axis equal;

% Plot heading : odometry vs ground truth (gps)
subplot(3,1,3);hold on;
plot(data_true.Var1 ,  data_true.Var4); hold on;
plot(data.time , wrapToPi(data.odo_enc_heading));
title('Heading : odometry vs ground truth (supervisor)');
legend('Ground Thruth', 'Estimation');
xlabel('Time [s]'); ylabel('heading [Rad]');
y_lim = [min([wrapToPi(data.odo_enc_heading);  data.pose_heading]),max([wrapToPi(data.odo_enc_heading);  data.pose_heading])];
xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% Speed measured vs ground truth
clc; close all;
name = "localization.csv";
name_true = "datatrue.csv";
[N_SIM, T_SIM, T, data] = read_log(name);
data_true= readtable(name_true,"ReadVariableNames",0,"NumHeaderLines",0);



% Plot the odometry computed using the chosen pose
f = figure('Name','Webots : Estimation [Rad]'); 
subplot(3,1,[1 2]);hold on;



vx_true = (data_true.Var2(2:end)-data_true.Var2(1:end-1))./(data_true.Var1(2:end)-data_true.Var1(1:end-1));
vy_true = -(data_true.Var3(2:end)-data_true.Var3(1:end-1))./(data_true.Var1(2:end)-data_true.Var1(1:end-1));
vx_est = (est_x(2:end)-est_x(1:end-1))./(data.time(2:end)-data.time(1:end-1));
vy_est = (est_y(2:end)-est_y(1:end-1))./(data.time(2:end)-data.time(1:end-1));
% Plot x -y plan : odometry vs ground truth (supervisor)
plot(data_true.Var1(1:end-1) , vx_true); hold on;
plot(data_true.Var1(1:end-1) , vy_true); hold on;
plot(data.time(1:end-1) , vx_est); hold on;
plot(data.time(1:end-1) , vy_est); hold on;
% plot(est_x , est_y,'--');
% title('x -y plan : Estimation vs ground truth (supervisor)');
legend('true vx', 'true vy','est vx','est vy');
% xlabel('x [m]'); ylabel('y [m]');
% x_lim = [min([data_true.Var2;  est_x]),max([data_true.Var2;  est_x])];
% y_lim = [min([data_true.Var3;  est_y]),max([data_true.Var3;  est_y])];
% xlim(x_lim + [-0.05,0.05]*(x_lim(2)-x_lim(1)));ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));
% axis equal;

% 
% % Plot heading : odometry vs ground truth (gps)
% subplot(3,1,3);hold on;
% plot(data_true.Var1 ,  data_true.Var4); hold on;
% plot(data.time , wrapToPi(data.odo_enc_heading));
% title('Heading : odometry vs ground truth (supervisor)');
% legend('Ground Thruth', 'Estimation');
% xlabel('Time [s]'); ylabel('heading [Rad]');
% y_lim = [min([wrapToPi(data.odo_enc_heading);  data.pose_heading]),max([wrapToPi(data.odo_enc_heading);  data.pose_heading])];
% xlim([data.time(1), data.time(end)]);ylim(y_lim + [-0.05,0.05]*(y_lim(2)-y_lim(1)));

%% compute the mean accuracy

%subsample to have the same number of points as the ground truth

x_est_sub = est_x(4:4:size(data_true,1)*4);
y_est_sub = est_y(4:4:size(data_true,1)*4);
h_est_sub = est_h(4:4:size(data_true,1)*4);

dif_x = x_est_sub-data_true.Var2;
dif_y = y_est_sub+data_true.Var3;
dif_h = h_est_sub-data_true.Var4;

avg_acc_x = mean(dif_x.^2,'omitnan');
avg_acc_y = mean(dif_y.^2,'omitnan');
avg_acc_h = mean(dif_h.^2,'omitnan');

min_acc_x = max(dif_x.^2);
min_acc_y = max(dif_y.^2);
min_acc_h = max(dif_h.^2);
disp('mean accuracy over the run:')
disp(mean([avg_acc_x,avg_acc_y]))
