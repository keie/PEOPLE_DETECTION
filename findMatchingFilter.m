function filterIndex = findMatchingFilter(kalmanFilters, bbox, maxDistance)
    % This function finds the best matching Kalman filter for the given bounding box.
    % It returns the index of the Kalman filter with the closest prediction to the bbox center.
    % If no filter is close enough (within maxDistance), it returns empty.

    filterIndex = [];  % Default to no match found
    minDistance = inf;  % Start with infinity as the minimum distance
    
    % Calculate the center of the input bounding box
    bboxCenter = [bbox(1) + bbox(3)/2, bbox(2) + bbox(4)/2];
    
    % Loop through all existing Kalman filters to find the best match
    for i = 1:length(kalmanFilters)
        kalmanFilter = kalmanFilters{i};
        predictedPosition = predict(kalmanFilter);
        predictedCenter = [predictedPosition(1) + predictedPosition(3)/2, ...
                           predictedPosition(2) + predictedPosition(4)/2];
        distance = norm(bboxCenter - predictedCenter);  % Euclidean distance
        
        % Check if this filter is a better match than what we've found before
        if distance < minDistance && distance < maxDistance
            minDistance = distance;
            filterIndex = i;
        end
    end
end
