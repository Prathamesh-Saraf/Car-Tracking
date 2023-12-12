function [path, P_temp] = get_path(dt, x, P, intersectionMap, speeds, Q_EW, Q_NS, data, path)

    x_init = x;
    A = [1, 0;
         0, 1];
    
    B = [dt, 0; 0, dt];

    z = data;

    R = [9500, 0; 0, 1.4] * 1;

    intersections = [-500, -200;
                     -350, -200;
                     -350, -800;
                     -750, -800;
                     -950, -800;
                     -950, -400;
                     -950, 200;
                     -750, 200;
                     -550, 200;
                     -550, -400;
                     -750, -400];
    
    nearest_intersection = [];
    dist = 100;

    for i = 1:size(intersections, 1)
        [result, dist_temp] = withinOneStd(intersections(i,:), x', P);
        if result
            if dist > dist_temp
                dist = dist_temp;
                nearest_intersection = intersections(i,:);
            end
        end
    end

    if size(nearest_intersection, 1) == 0
        path = path;
        P_temp = P;
        return
    end


    paths = intersectionMap(mat2str(nearest_intersection));
    valid_paths = [];

    n = size(paths);
    
    for i = 1:n(2)
        if paths(i) ~= path
            valid_paths = [valid_paths; paths(i)];
        end
    end

    if size(valid_paths, 1) == 0
        path = path;
        P_temp = P;
        return
    end

    straight = 0;
    for i = 1:size(valid_paths, 1)
        if abs(valid_paths(i) - path) == 2
            straight = 1;
            break
        end
    end

    if straight == 0
        if P(1,1) > P(2,2)
            P(1,1) = 0.3 * P(1,1);
        else
            P(2,2) = 0.3 * P(2,2);
        end
    end

    % Caluclate mean along the rows
    speed = mean(speeds(:, valid_paths),2);
    

    n = size(valid_paths);
    x_temp = A * x + B * speed;
    P_temp = A * P * A';
    for i = 1:n(1)
        if valid_paths(i) == 1 || valid_paths(i) == 3
            P_temp = P_temp + (Q_NS) / (n(1) * n(1));
        else
            P_temp = P_temp + (Q_EW) / (n(1) * n(1));
        end
    end

    r = sqrt(x_temp(1)^2 + x_temp(2)^2);
    theta = atan2(x_temp(2), x_temp(1));

    H = [x_temp(1) / r, x_temp(2) / r;
         -x_temp(2) / r^2, x_temp(1) / r^2];

    y = z' - H * x_temp;

    S = H * P_temp * H' + R;
    K = P_temp * H' * inv(S);
    x = x_temp + K * y;
    P = (eye(2) - K * H) * P_temp;

    diff = x - x_init;
    diff = diff';
    disp(diff)

    if abs(diff(1)) > abs(diff(2))
        if diff(1) > 0
            path = 2;
        else
            path = 4;
        end
    else
        if diff(2) > 0
            path = 1;
        else
            path = 3;
        end
    end
    disp(path)

end

function [isWithinOneStd, mahalanobisDistance] = withinOneStd(vector, meanVector, covarianceMatrix)
    % Calculate Mahalanobis distance
    delta = vector - meanVector;
    mahalanobisDistance = sqrt(delta * inv(covarianceMatrix) * delta');
    % disp(mahalanobisDistance);

    % Check if within 1 standard deviation
    isWithinOneStd = mahalanobisDistance <= 2.5 * sqrt(trace(covarianceMatrix));
    % disp(sqrt(trace(covarianceMatrix)));
end