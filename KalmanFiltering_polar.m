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

radar1_data_polar(: , 1) = sqrt(radar1_data(: , 1).^2 + radar1_data(: , 2).^2);
radar1_data_polar(: , 2) = atan2(radar1_data(: , 2), radar1_data(: , 1));

radar2_data_polar(: , 1) = sqrt(radar2_data(: , 1).^2 + radar2_data(: , 2).^2);
radar2_data_polar(: , 2) = atan2(radar2_data(: , 2), radar2_data(: , 1));

true_path_polar(: , 1) = sqrt(true_path_x(: , 1).^2 + true_path_y(: , 1).^2);
true_path_polar(: , 2) = atan2(true_path_y(: , 1), true_path_x(: , 1));

for i = 1:60
    radar2_data_polar_movav(i, 1) = (radar2_data_polar(i, 1) + radar2_data_polar(i+1, 1))/2; 
    radar2_data_polar_movav(i, 2) = (radar2_data_polar(i, 2) + radar2_data_polar(i+1, 2))/2; 
end

radar2_data_polar_movav(61, 1) = radar2_data_polar(61, 1);
radar2_data_polar_movav(61, 2) = radar2_data_polar(61, 2);
radar2_data_car_movav(:, 1) = radar2_data_polar_movav(:, 1).*cos(radar2_data_polar_movav(:, 2));
radar2_data_car_movav(:, 2) = radar2_data_polar_movav(:, 1).*sin(radar2_data_polar_movav(:, 2));
true_path_car(:, 1) = true_path_polar(:, 1).*cos(true_path_polar(:, 2));
true_path_car(:, 2) = true_path_polar(:, 1).*sin(true_path_polar(:, 2));

% Plotting
figure;

% Plot radar measurements
plot(radar2_data(:, 1), radar2_data(:, 2), 'ro', 'DisplayName', 'Radar 2 Data'); 
hold on;
plot(radar2_data_car_movav(:, 1), radar2_data_car_movav(:, 2), 'bo', 'DisplayName', 'Radar 2 Data filtered');

% Plot averaged measurements
% plot(measured_data(1:2, 1), measured_data(1:2, 2), 'g.', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path_car(:, 1), true_path_car(:, 2), 'r.', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
% plot(estimated_positions(1:2, 1), estimated_positions(1:2, 2), 'ko', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;