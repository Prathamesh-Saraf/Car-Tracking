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
estimated_positions = zeros(61, 2);
average = mean(speed);
std_deviation = std(speed);
dt = 5;
epsilon_t = randn(61, 1);
estimated_positions(1,:) = 0.5*radar1_data(1,:) + 0.5*radar2_data(1,:);

for k = 2:61
    del_x = average*dt + std_deviation*sqrt(dt)*epsilon_t(k);
    estimated_positions(k, :) = estimated_positions(k-1, :) + [del_x(1), del_x(2)];

end
% Plot true path
plot(true_path_x, true_path_y, 'r.', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'ko', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;