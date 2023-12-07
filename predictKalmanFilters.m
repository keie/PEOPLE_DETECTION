
function bboxes = predictKalmanFilters(kalmanFilters)
    % Function to predict positions using Kalman filters
    bboxes = [];
    for i = 1:numel(kalmanFilters)
        kalmanFilter = kalmanFilters{i};
        predictedPosition = predict(kalmanFilter);
        bboxes = [bboxes; predictedPosition];
    end
end