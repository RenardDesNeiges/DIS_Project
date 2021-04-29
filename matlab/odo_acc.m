function [x, vx] = odo_acc(x, vx, T, acc, acc_mean)
% Express the acceleration in Webots world frame and remove the bias (Assume 1-D) 
% To Do : Complete the following equation. Make sure to use the correct index of the acc.
%acc_wx = ...;

% Odometry in X 
% To Do : Complete the following equations using the previous acc_wx
acc_cor = acc-acc_mean;

x = x+vx*T+0.5*acc_cor(2)*T.^2;
vx = vx+acc_cor(2)*T;
    
end