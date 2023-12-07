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
measured_data_polar(:, 1) = sqrt(measured_data(: , 1).^2 + measured_data(: , 2).^2);
measured_data_polar(:, 2) = atan2(measured_data(: , 2), measured_data(: , 1));

% Kalman filter initialization 12-24
dt = 5;

A = [1 dt 0  0; 
     0  1 0  0; 
     0  0 1 dt; 
     0  0 0  1];

x = [measured_data(1, 1); 0; measured_data(1, 2); 20];

P = eye(4) * 0.1; % Initial covariance
Q = eye(4) * 100; % Process noise covariance
R = [9025 0 0 0; 0 0 0 0; 0 0 0.001225 0; 0 0 0 0]; % Measurement noise covariance, based on radar specs

estimated_positions = zeros(61, 2);
estimated_positions(1,:) = [x(1), x(3)];

for k = 2:61
    % Prediction
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

% for k = 2:61
%     % Prediction
%     r = sqrt(x(1)^2 + x(3)^2);
%     vr = (x(1)*x(2) + x(3)*x(4)) / r;
%     theta = atan2(x(3), x(1));
%     omega = (x(1)*x(4) - x(3)*x(2)) / r^2;
%     
%     J = [x(1)/r                          0            x(3)/r                      0;
%          ((x(2)/r) - (x(1)*vr/r))     (x(1)/r)     ((x(4)/r) - (x(3)*vr/r))    (x(3)/r);
%          -x(3)/r^2                       0            x(1)/r^2                    0;
%          (x(4)/r - 2*x(1)*omega/r)    (-x(3)/r^2)  (-x(2)/r - 2*x(3)*omega/r)  (x(1)/r^2)]
%     inv_J = inv(J)
%     x = [r; vr; theta; omega]
%     A_polar = J * A * inv(J)
%     x_hat =  A_polar * x;
% 
%     P_hat = A_polar * P * A_polar' + Q;
%     
%     measured_data_vx = (measured_data(k, 1) - measured_data(k-1, 1))/dt;
%     measured_data_vy = (measured_data(k, 2) - measured_data(k-1, 2))/dt;
%     measured_data_polar_vr = (measured_data(k, 1)*measured_data_vx + measured_data(k, 2)*measured_data_vy) / measured_data_polar(k, 1);
%     measured_data_polar_omega = (measured_data(k, 1)*measured_data_vy - measured_data(k, 2)*measured_data_vx) / measured_data_polar(k, 1)^2;
% 
%     % Update
%     y = [measured_data_polar(k, 1); measured_data_polar_vr; measured_data_polar(k, 2); measured_data_polar_omega] - H * x_hat;
%     S = H * P_hat * H' + R;
%     K = P_hat * H' / S; 
%     x = x_hat + K * y;
%     P = (eye(4) - K * H) * P_hat;
% 
%     estimated_positions(k, :) = [x(1), x(3)];
% end

% Plotting
figure;

% Plot radar measurements
% plot(radar1_data(1:2, 1), radar1_data(1:2, 2), 'ro', 'DisplayName', 'Radar 1 Data'); 
hold on;
% plot(radar2_data(1:2, 1), radar2_data(1:2, 2), 'bo', 'DisplayName', 'Radar 2 Data');

% Plot averaged measurements
plot(measured_data(:, 1), measured_data(:, 2), 'g.', 'DisplayName', 'Averaged Radar Data');

% Plot true path
plot(true_path(:, 1), true_path(:, 2), 'r.', 'DisplayName', 'True Path');

% Plot estimated positions from Kalman filter
plot(estimated_positions(:, 1), estimated_positions(:, 2), 'ko', 'DisplayName', 'Kalman Filter Estimate');


legend('Location', 'best');
title('Vehicle Position Estimation using Kalman Filter');
xlabel('X Coordinate');
ylabel('Y Coordinate');
grid on;
hold off;

%% x = 94.957, y = 2.849
