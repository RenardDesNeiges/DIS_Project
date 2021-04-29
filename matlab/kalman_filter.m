clear all, close all, clc

% Read data
D = csvread('data.csv');
t = D(:, 1);
ax = D(:, 2);
ay = D(:, 3);
gpsx = D(:, 4);
gpsy = D(:, 5);
true_x = D(:, 6);
true_y = D(:, 7);

% Initialize some variables to record results anbd plot later
x_est = [0];
y_est = [0];
vx_est = [0];
vy_est = [0];
diag_cov_1 = [0.001];
diag_cov_2 = [0.001];

% Kalman filter
% We have the following formulation of state variable X:
% X= [pos_x, pos_y, vel_x, vel_y]^T; 
% Note that this is a 4x1 column vector.
% Formulate state variables
X = zeros(4,1);

R = [ 0.05,   0,   0,   0;
        0, 0.05,   0,   0;
        0,   0, 0.01,   0
        0,   0,   0, 0.01];

Cov = [0.001,     0,     0,     0;
           0, 0.001,     0,     0;
           0,     0, 0.001,     0;
           0,     0,     0, 0.001];

       

for i = 2:length(t)   
    
    
  % ###################################################################
  % Section A - Prediction/Process step of Kalman filter
  % Formulate the matrices A and B. Calculate the new state and
  % covariance: X_new and Cov_new.
  % ###################################################################
    % Read accelerometer values
    acc = [ax(i), ay(i)]';
    dt = t(i) - t(i-1);
    
    A = [1 0 dt 0
         0 1 0 dt
         0 0 1 0
         0 0 0 1];
         
     B = [0 0 
          0 0
          dt 0 
          0 dt];
      
    X_new = A*X+B*acc;
    Cov_new =  A*Cov*A'+R*dt;
    
  % ###################################################################
  
  
  % ###################################################################
  % Section B - Measurement step of Kalman filter
  % Formulate the matrices C and Q. Calculate the new state and
  % covariance: X_new and Cov_new.
  % ###################################################################
  
  if (mod(i, 4) == 0) % restrict GPS updates to one per second.
      
      % Measurement
      z = [gpsx(i), gpsy(i)]';
      
      C =  [1 0 0 0
            0 1 0 0];
      Q = [1 0 
           0 1];
      
      K = Cov_new*C'/(C*Cov_new*C'+Q);
      X_new = X_new + K*(z-C*X_new); 
      Cov_new =  (eye(4) - K*C)* Cov_new;
      
  end
  
  % ###################################################################
    
  % Record data for potting
    x_est = [x_est X_new(1)];
    y_est = [y_est X_new(2)];
    vx_est = [vx_est X_new(3)];
    vy_est = [vy_est X_new(4)];
    diag_cov_1 = [diag_cov_1 Cov_new(1,1)];
    diag_cov_2 = [diag_cov_2 Cov_new(2,2)];
    
    X = X_new;
    Cov = Cov_new;
end

figure, 
hold on
grid on
plot(true_x, true_y);
plot(x_est,y_est, '-x');
title('KF Estimated trajectory')
legend('True', 'Est.', 'location', 'northwest')

figure, 
hold on
plot(t, diag_cov_1);
plot(t, diag_cov_2);
title('Evolution of diagonal elements of \Sigma')
legend('\Sigma_{11}', '\Sigma_{22}','location', 'northwest')