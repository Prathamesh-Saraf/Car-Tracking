% Given data
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\RadarData40.mat');


radar1_data = reshape(RadarData40(1:2,:), 61, 2); % [61 x 2] matrix
radar2_data = reshape(RadarData40(3:4,:), 61, 2); % [61 x 2] matrix

measured_data = 0.8*radar1_data + 0.2*radar2_data;

% Kalman filter initialization
dt = 5;
A = [1 dt 0  0; 
     0  1 0  0; 
     0  0 1 dt; 
     0  0 0  1];

H = [1 0 0 0; 
     0 0 1 0];

x = [measured_data(1, 1); 0; measured_data(1, 2); 0]; % Initial state
P = eye(4) * 1; % Initial covariance
Q = eye(4); % Process noise covariance
R = eye(2); % Measurement noise covariance, based on radar specs

estimated_positions = zeros(61, 2);

for k = 1:61
    % Prediction
    x_hat = A * x;
    P_hat = A * P * A' + Q;

    % Update
    y = measured_data(k, :)' - H * x_hat;
    S = H * P_hat * H' + R;
    K = P_hat * H' / S; 
    x = x_hat + K * y;
    P = (eye(4) - K * H) * P_hat;

    estimated_positions(k, :) = [x(1), x(3)];
end

% Plotting
figure;

% Plot radar measurements
plot(radar1_data(:, 1), radar1_data(:, 2), 'ro', 'DisplayName', 'Radar 1 Data'); hold on;
plot(radar2_data(:, 1), radar2_data(:, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
plot(measured_data(:, 1), measured_data(:, 2), 'g.', 'DisplayName', 'Averaged Radar Data');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'k-', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;
