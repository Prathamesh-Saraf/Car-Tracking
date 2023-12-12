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

x = true_path(1,:)';
%x = radar1_data(1, :)';

vel_increment_std_EW = [1.4, 0; 0, 1.7];
vel_increment_std_NS = [1.4, 0; 0, 2.8];

A = [1, 0; 0, 1];

B_EW = [dt, 0; 0, dt];
B_NS = [dt, 0; 0, dt];

P = [10.149, 0; 0, 36.957];

Q_EW = (vel_increment_std_EW * vel_increment_std_EW) / 4;
Q_NS = (vel_increment_std_NS * vel_increment_std_NS) / 4;

R = [95*95, 0; 0, 10] * 1;

estimated_positions = zeros(61, 2);

estimated_positions(1, :) = x';

intersectionMap = get_map();

speed_x = reshape(TruePath(2, :), 61, 1);
speed_y = reshape(TruePath(5, :), 61, 1);

speed = [speed_x(1); speed_y(1)];
for k = 2:61 

    for i = 1:5
        if speed(1) == 0
            x = A * x + B_NS * speed + mvnrnd([0, 0], 2 * Q_NS)';
        else
            x = A * x + B_EW * speed + mvnrnd([0, 0], 2 * Q_EW)';
        end
    end

    if speed(1) == 0
        x_hat = A * x + B_NS * speed + mvnrnd([0, 0], 2 * Q_NS)';
        P_hat = A * P * A' + Q_NS;
    else
        x_hat = A * x + B_EW * speed + mvnrnd([0, 0], 2 * Q_EW)';
        P_hat = A * P * A' + Q_EW;
    end

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
    speed = [speed_x(k-1); speed_y(k-1)];
    % speed = [dzhX(1,k); dzhX(2,k)];

end

figure;

plot(true_path(1:61, 1), true_path(1:61, 2), 'b');%, 'MarkerSize', 10);

hold on;

plot(estimated_positions(1:61, 1), estimated_positions(1:61, 2), 'r');%, 'MarkerSize', 10);

%plot(estimated_positions_temp(12:24, 1), estimated_positions_temp(12:24, 2), 'g');%, 'MarkerSize', 10);

legend('True Path', 'Estimated Path');

title('True Path vs Estimated Path');
grid on;

hold off;