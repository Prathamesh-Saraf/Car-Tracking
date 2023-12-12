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

N = 20; % Number of particles
particles = zeros(N, 4); % Each particle has x, vx, y, vy
initial_noise_pos_x = 1; % Initial uncertainty in the car's position
initial_noise_pos_y = 1;
initial_noise_vel = 1; % Initial uncertainty in the car's velocity

estimated_positions = zeros(61, 2);
estimated_positions(1,:) = [true_path(1, 1), true_path(1, 2)];

% Initialize the particles randomly around the first measurement with initial velocity guess
for i = 1:N
    particles(i, :) = [true_path(1, 1), 0, true_path(1, 2), 0] + randn(1, 4); % .* [initial_noise_pos_x, initial_noise_vel, initial_noise_pos_y, initial_noise_vel];
end
particle(1, :, :) = particles;
dt = 5;
A = [1 dt 0  0; 
     0  1 0  0; 
     0  0 1 dt; 
     0  0 0  1];

for k = 2:61 % Assuming we have 61 data points
%     motion_noise = [measured_data(k, 1), (true_path(k, 1) - true_path(k - 1, 1)) / dt, measured_data(k, 2), (true_path(k, 2) - true_path(k - 1, 2)) / dt]; % Adjust this based on the expected system noise
    motion_noise = [true_path(k, 1), (true_path(k, 1) - true_path(k - 1, 1)) / dt, true_path(k, 2), (true_path(k, 2) - true_path(k - 1, 2)) / dt]; % Adjust this based on the expected system noise

    for i = 1:N
        % Apply the motion model
        particles(i, :) = 0.9 * A * particles(i, :)' + 0.1 * motion_noise' + dt * randn(4, 1);%  + randn(4, 1) .* motion_noise';
    end
    particle(k, :, :) = particles;
    measurement_noise = 100; % Measurement noise

    weights = zeros(N, 1);

    for i = 1:N
        % Consider only position components for measurement likelihood
        distance = norm([measured_data(k, 1), measured_data(k, 2)] - particles(i, [1, 3]));
%         distance = abs(sqrt((true_path(k, 1) - particles(i, 1))^2 + (true_path(k, 2) - particles(i, 3))^2));
        weights(i) = exp(-0.5 * (distance^2) / (measurement_noise^2));
    end
    weights = weights / sum(weights); % Normalize the weights

    % Resampling
    indices = resample_indices(weights);
    particles = particles(indices, :);

    % Estimate the position
    estimated_position = mean(particles(:, [1, 3]), 1);
    estimated_positions(k, :) = estimated_position;
end


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
plot(particle(:, :, 1), particle(:, :, 3), 'b.');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'g', 'LineWidth',2, 'DisplayName', 'Particle Filter Estimate');
% 
% for i = 1:61
%     plotEllipse(estimated_positions(i, 1), estimated_positions(i, 2), randn(1, 1)*20, randn(1, 1)*20);
% end
% legend('Location', 'best');
title('Vehicle Position Estimation using Particle Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

% Here, additional motion model logic can be applied if needed
% For example, if there are non-linear dynamics, you might want to add them here

% function indices = resample_indices(weights)
%     % Systematic resampling
%     N = length(weights);
%     positions = (1:N) + rand(1, N);
%     indices = zeros(1, N);
%     cumsum_weights = cumsum(weights);
%     i = 1;
%     for pos_idx = 1:N
%         pos = positions(pos_idx);
%         while cumsum_weights(i) < pos
%             i = min(i + 1, N);
%             if i == N || pos_idx == N
%                 break
%             end
%         end
%         indices(pos_idx) = i;
%     end
%     return
% end

function indices = resample_indices(weights)
    N = length(weights);
    positions = (rand(1, N) + (0:N-1)) / N;
    indices = zeros(1, N);
    cumsum_weights = cumsum(weights);
    i = 1;
    for pos_idx = 1:N
        while cumsum_weights(i) < positions(pos_idx)
            i = i + 1;
        end
        indices(pos_idx) = i;
    end
end

function plotEllipse(x0, y0, a, b)
    % Plot an ellipse centered at (x0, y0) with semi-major axis 'a' and
    % semi-minor axis 'b'.
    %
    % :param x0, y0: Coordinates of the center of the ellipse
    % :param a: Semi-major axis of the ellipse
    % :param b: Semi-minor axis of the ellipse

    theta = linspace(0, 2*pi, 100);  % Parameter for the ellipse
    x = x0 + a * cos(theta);         % X coordinates
    y = y0 + b * sin(theta);         % Y coordinates

    plot(x, y, 'm');
    axis equal;  % Ensures the ellipse is not skewed
    grid on;
    
end
