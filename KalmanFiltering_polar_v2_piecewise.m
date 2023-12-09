% Given data
clear all;
clc
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\RadarData40.mat');
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\TruePath.mat');
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\Speed.mat');
load('./../dataset/RadarData40.mat');
load('./../dataset/TruePath.mat');
load('./../dataset/Speed.mat');

radar1_data = reshape(RadarData40(1:2,:), 61, 2);
radar2_data = reshape(RadarData40(3:4,:), 61, 2);
true_path(:, 1) = reshape(TruePath(1,:), 61, 1);
true_path(:, 2) = reshape(TruePath(4,:), 61, 1);
speed = reshape(dzhX(1:2,:), 61, 2);

measured_data = 0.5*radar1_data + 0.5*radar2_data;
measured_data_polar(12:24, 1) = sqrt(measured_data(12:24 , 1).^2 + measured_data(12:24 , 2).^2);
measured_data_polar(12:24, 2) = atan2(measured_data(12:24 , 2), measured_data(12:24 , 1));

% Kalman filter initialization 12-24
dt = 5;

A = [1 dt 0  0; 
     0  1 0  0;
     0  0 1 dt; 
     0  0 0  1];

% x = [measured_data(12, 1); -10; measured_data(12, 2); 0];
x = [true_path(12, 1); -10; true_path(12, 2); 0];


P = eye(4) * 18; % Initial covariance % 35
Q = eye(4) * 100; % Process noise covariance % 1.11
R = [9025 0 0 0; 0 0 0 0; 0 0 0.001225 0; 0 0 0 0]; % Measurement noise covariance, based on radar specs

estimated_positions = zeros(61, 2);
estimated_positions(12,:) = [x(1), x(3)];

for k = 13:24
    % Prediction
    x(2) = -10;
    x(4) = 0;
    x_hat = A * x;
    P_hat = A * P * A' + Q;
    
    % Convert state to polar coordinates
    r_hat = sqrt(x_hat(1)^2 + x_hat(3)^2);
    vr_hat = (x_hat(1)*x_hat(2) + x_hat(3)*x_hat(4)) / r_hat;
    theta_hat = atan2(x_hat(3), x_hat(1));
    omega_hat = (x_hat(1)*x_hat(4) - x_hat(3)*x_hat(2)) / r_hat^2;
    
    % Jacobian of the Measurement Function H
    H_j = [x_hat(1)/r_hat                                   0                     x_hat(3)/r_hat                                       0;
          ((x_hat(2)/r_hat) - (x_hat(1)*vr_hat/r_hat))      (x_hat(1)/r_hat)     ((x_hat(4)/r_hat) - (x_hat(3)*vr_hat/r_hat))          (x_hat(3)/r_hat);
           -x_hat(3)/r_hat^2                                0                     x_hat(1)/r_hat^2                                     0;
           (x_hat(4)/r_hat - 2*x_hat(1)*omega_hat/r_hat)    (-x_hat(3)/r_hat^2)  (-x_hat(2)/r_hat - 2*x_hat(3)*omega_hat/r_hat)  (x_hat(1)/r_hat^2)];  

    measured_data_vx = (measured_data(k, 1) - measured_data(k-1, 1))/dt;
    measured_data_vy = (measured_data(k, 2) - measured_data(k-1, 2))/dt;
    measured_data_polar_vr = (measured_data(k, 1)*measured_data_vx + measured_data(k, 2)*measured_data_vy) / measured_data_polar(k, 1);
    measured_data_polar_omega = (measured_data(k, 1)*measured_data_vy - measured_data(k, 2)*measured_data_vx) / measured_data_polar(k, 1)^2;

    % Measurement Update
    z = [measured_data_polar(k, 1); measured_data_polar_vr; measured_data_polar(k, 2); measured_data_polar_omega];
    y = z - [r_hat; vr_hat; theta_hat; omega_hat]; 
%     z = [measured_data_polar(k, 1); 0; measured_data_polar(k, 2); 0];  
%     y = z - [r_hat; 0; theta_hat; 0];
    S = H_j * P_hat * H_j' + R;
    K = P_hat * H_j' / S; 
    x = x_hat + K * y;
    P = (eye(4) - K * H_j) * P_hat;

    estimated_positions(k, :) = [x(1), x(3)];
end


% Plotting
figure;

% Plot radar measurements
% plot(radar1_data(1:2, 1), radar1_data(1:2, 2), 'ro', 'DisplayName', 'Radar 1 Data'); 
hold on;
% plot(radar2_data(1:2, 1), radar2_data(1:2, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
plot(measured_data(12:24, 1), measured_data(12:24, 2), 'g.', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path(12:24, 1), true_path(12:24, 2), 'r.', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
plot(estimated_positions(12:24, 1), estimated_positions(12:24, 2), 'ko', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

%% x = 94.957, y = 2.849
