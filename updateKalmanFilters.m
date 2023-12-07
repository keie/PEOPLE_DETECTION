function [kalmanFilters, colors] = updateKalmanFilters(kalmanFilters, bboxes, colors)
    % Function to update or initialize Kalman filters and assign colors
    for i = 1:size(bboxes, 1)
        bbox = bboxes(i, :);
        if i > numel(kalmanFilters)
            % Initialize a new Kalman filter for this detection
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                                                 bbox(1:2), [1 1]*1e5, ...
                                                 [25, 10], 25);
            kalmanFilters{i} = kalmanFilter;
            
            % Assign a random color for this filter
            colors{i} = rand(1, 3);
        else
            % Update existing Kalman filter with new detection
            kalmanFilter = kalmanFilters{i};
            bbox = correct(kalmanFilter, bbox(1:2));
        end
    end
end