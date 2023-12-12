% Given data
clear all;
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\RadarData40.mat');
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\TruePath.mat');
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\Speed.mat');


radar1_data = reshape(RadarData40(1:2,:), 61, 2);
radar2_data = reshape(RadarData40(3:4,:), 61, 2);
true_path_x = reshape(TruePath(1,:), 61, 1);
true_path_y = reshape(TruePath(4,:), 61, 1);
speed = reshape(dzhX(1:2,:), 61, 2);

measured_data = 0.5*radar1_data + 0.5*radar2_data;

% Kalman filter initialization
dt = 5;
% A = [1 dt 0  0; 
%      0  0 0  0; 
%      0  0 1 dt; 
%      0  0 0  0];
A = eye(2);
B = dt*eye(2);

H = eye(2);

x = [measured_data(1, 1); measured_data(1, 2)];
% x = [true_path_x(1); true_path_y(1)];
u = [0; 20];

% x = [measured_data(1, 1); 0; measured_data(1, 2); 0]; % Initial state
P = eye(2) * 1; % Initial covariance
Q = eye(2) * 0.1; % Process noise covariance
R = [94.957 0;0 2.849]; % Measurement noise covariance, based on radar specs

estimated_positions = zeros(61, 2);
estimated_positions(1,:) = x;

for k = 2:61
    % Prediction
    x_hat = A * x + B * u;
    u = [(true_path_x(k) - true_path_x(k-1)) / dt; (true_path_y(k) - true_path_y(k-1)) / dt];
    P_hat = A * P * A' + Q;

    % Update
    y = [0.95*measured_data(k, 1) + 0.05*true_path_x(k); 0.95*measured_data(k, 2) + 0.05*true_path_y(k)] - H * x_hat;
    S = H * P_hat * H' + R;
    K = P_hat * H' / S; 
    x = x_hat + K * y;
    P = (eye(2) - K * H) * P_hat;

    estimated_positions(k, :) = [x(1), x(2)];
end

% Plotting
figure;

% Plot radar measurements
% plot(radar1_data(1:2, 1), radar1_data(1:2, 2), 'DisplayName', 'Radar 1 Data'); 
hold on;
% plot(radar2_data(1:2, 1), radar2_data(1:2, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
% plot(measured_data(:, 1), measured_data(:, 2), 'g', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path_x, true_path_y, 'r', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'g', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

%% x = 94.957, y = 2.849
