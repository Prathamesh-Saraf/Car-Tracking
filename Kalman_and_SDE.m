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

average = mean(speed);
std_deviation = std(speed);

measured_data = 1 * radar1_data + 0 * radar2_data;

% Kalman filter initialization
dt = 5;
% A = [1 dt 0  0; 
%      0  0 0  0; 
%      0  0 1 dt; 
%      0  0 0  0];
A = eye(2);
B = dt*eye(2);

x = [true_path_x(1); true_path_y(1)];

% x = [measured_data(1, 1); measured_data(1, 2)];


u = [speed(1, 1); speed(1, 2)];

% x = [measured_data(1, 1); 0; measured_data(1, 2); 0]; % Initial state
P = eye(2) * 100; % Initial covariance
Q = eye(2) * 100; % Process noise covariance
R = [94.942 0;0 3.324]; % Measurement noise covariance, based on radar specs

estimated_positions = zeros(61, 2);
estimated_positions(1,:) = x;
epsilon_t = randn(61, 1);


for k = 2:61
    % Prediction
    % del_x = speed(k,1) * dt + std_deviation * sqrt(dt) * epsilon_t(k);
    del_x = speed(k, 1)*epsilon_t(k);
    H = [x(1)/sqrt(x(1)^2 + x(2)^2) x(2)/sqrt(x(1)^2 + x(2)^2); -x(2)/(x(1)^2 + x(2)^2) x(1)/(x(1)^2 + x(2)^2)];
    %Q = [del_x(1) 0; 0 del_x(2)];
    x_hat = A*x + B*u;

    u = [speed(k, 1); speed(k, 2)];
    P_hat = A * P * A' + Q;

    % Update
    y = measured_data(k, :)' - H * x_hat;
    S = H * P_hat * H' + R;
    K = P_hat * H' / S; 
    x = x_hat + K * y;
    P = (eye(2) - K * H) * P_hat;

    estimated_positions(k, :) = [x(1), x(2)];
end

% Plotting
figure;

% Plot radar measurements
% plot(radar1_data(:, 1), radar1_data(:, 2), 'ro', 'DisplayName', 'Radar 1 Data'); 
hold on;
% plot(radar2_data(:, 1), radar2_data(:, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
% plot(measured_data(:, 1), measured_data(:, 2), 'g.', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path_x(1:2), true_path_y(1:2), 'r.', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
plot(true_path_x(1,1) + 10*speed(1,1)*dt, true_path_y(1,1) + 10*speed(1,2)*dt, 'ko', 'DisplayName', 'estimated path');

% plot(estimated_positions(1:2, 1), estimated_positions(1:2, 2), 'ko', 'DisplayName', 'Kalman Filter Estimate');



legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

%% x = 94.957, y = 2.849
