% Piecewise EKF from CV motion model using nominal velocity - single path
% Given data
clear all;
clc
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\RadarData40.mat');
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\TruePath.mat');
% load('C:\Users\prath\Desktop\UCSD Courses\FA23\ECE272B\ClassData copy\Speed.mat');
load('./../dataset/RadarData40.mat');
load('./../dataset/TruePath.mat');
load('./../dataset/Speed.mat');

dt = 1;

radar1_data = reshape(RadarData40(1:2, :), 61, 2);

true_path(:, 1) = reshape(TruePath(1, :), 61, 1);
true_path(:, 2) = reshape(TruePath(4, :), 61, 1);

% x = true_path(12,:)'; 
x = radar1_data(12, :)';

% P = zeros(2, 2); % Change this

vel_increment_std = [1.4, 0; 0, 0.7];

A = [1, 0; 0, 1];

B = [dt, 0; 0, 0];

P = [95.957, 0; 0, 2.149];

% Q = [1, 0; 0, 0];
Q = (vel_increment_std * vel_increment_std) / 4;

R = [95*95, 0; 0, 1.4] * 1;

estimated_positions = zeros(61, 2);
estimated_positions_temp = zeros(61, 2);

estimated_positions(12, :) = x';
estimated_positions_temp(12, :) = x';

temp_x = x;

for j = 13:24
    temp_x = A * temp_x + 5 * B * [-10;0];
    estimated_positions_temp(j, :) = temp_x';
end

for k = 13:24
    
    % speed = (radar1_data(k, :) - radar1_data(k-1, :))/dt;
    % speed = speed';
    speed = [-10;0];
    % speed = speed + mvnrnd([0, 0], vel_increment_std)';
    for i = 1:5
        x = A * x + B * speed + mvnrnd([0, 0], 2 * Q)';
        P = A * P * A' + Q;
    end

    % Q = ([-10;0] - speed)/dt;
    % Q(2) = 0.02;

    % Q = vel_increment_std * 1;

    x_hat = A * x + B * speed;
    P_hat = A * P * A' + Q;

    r = sqrt(x_hat(1)^2 + x_hat(2)^2);
    theta = atan(x_hat(2)/x_hat(1));

    H = [x_hat(1)/r, x_hat(2)/r; -x_hat(2)/r^2, x_hat(1)/r^2];
    z = radar1_data(k, :)';
    y = z - H * x_hat;

    S = H * P_hat * H' + R;
    K = P_hat * H' * inv(S);

    x = x_hat + K * y;
    P = (eye(2) - K * H) * P_hat;

    estimated_positions(k, :) = x';

end

figure;


velocity = radar1_data(12:24, :) - radar1_data(11:23, :);
velocity = velocity / 5;
mean_vel_x = mean(velocity(:, 1));
var_vel_x = std(velocity(:, 1));
plot(true_path(12:24, 1), true_path(12:24, 2), 'b');%, 'MarkerSize', 10);

hold on;

plot(estimated_positions(12:24, 1), estimated_positions(12:24, 2), 'r');%, 'MarkerSize', 10);

%plot(estimated_positions_temp(12:24, 1), estimated_positions_temp(12:24, 2), 'g');%, 'MarkerSize', 10);

legend('True Path', 'Estimated Path');

title('True Path vs Estimated Path');
grid on;

hold off;