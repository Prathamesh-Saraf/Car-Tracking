% Given data
clear all;
clc

load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\RadarData40.mat');
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\TruePath.mat');
load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\Speed.mat');

radar1_data = reshape(RadarData40(1:2,:), 61, 2);
radar2_data = reshape(RadarData40(3:4,:), 61, 2);
true_path(:, 1) = reshape(TruePath(1,:), 61, 1);
true_path(:, 2) = reshape(TruePath(4,:), 61, 1);
speed = reshape(dzhX(1:2,:), 61, 2);

measured_data = 0.5*radar1_data + 0.5*radar2_data;
dt = 5;          % Time interval (adjust according to your data)
alpha = 0.01;    % Example value for alpha
beta = 0.5;    % Example value for beta

% Apply the alpha-beta filter
estimated_positions = alphaBetaFilter(measured_data, dt, alpha, beta);

% Plotting
figure;

% Plot radar measurements
% plot(radar1_data(1:2, 1), radar1_data(1:2, 2), 'ro', 'DisplayName', 'Radar 1 Data'); 
hold on;
% plot(radar2_data(1:2, 1), radar2_data(1:2, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
% plot(measured_data(:, 1), measured_data(:, 2),'m', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path(:, 1), true_path(:, 2), 'r', 'DisplayName', 'True Path');

% Plot particles
% plot(particle(:, :, 1), particle(:, :, 3), 'b.');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'g', 'DisplayName', 'Alpha-Beta Filter Estimate');

legend('Location', 'best');
title('Vehicle Position Estimation using Alpha-Beta Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

function estimated_positions = alphaBetaFilter(data, dt, alpha, beta)
    % Alpha-Beta Filter implementation for 2D position data.
    %
    % :param data: Nx2 matrix of (x, y) measurements
    % :param dt: Time interval between measurements
    % :param alpha: Alpha parameter for the filter
    % :param beta: Beta parameter for the filter
    %
    % :return: Estimated positions after applying the filter

    % Initialize variables
    estimated_positions = zeros(size(data));
    estimated_velocity = [0, 0];  % Initial velocity is assumed to be zero

    % Set the first estimate equal to the first measurement
    estimated_positions(1, :) = data(1, :);

    % Apply the alpha-beta filter
    for i = 2:size(data, 1)
        % Predict the new position and velocity
        predicted_position = estimated_positions(i-1, :) + estimated_velocity * dt;
        predicted_velocity = estimated_velocity;

        % Measurement update
        residual = data(i, :) - predicted_position;
        estimated_positions(i, :) = predicted_position + alpha * residual;
        estimated_velocity = predicted_velocity + (beta / dt) * residual;
    end
end
