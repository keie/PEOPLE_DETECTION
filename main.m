


% Create a VideoReader object to read the MP4 file
videoObj = VideoReader('/Users/appleuser/Desktop/pazmany/algorithms_image_processing/assignment_2/video/input_video.mp4');

% Create a People Detector object
peopleDetector = vision.PeopleDetector;

colors = {'red','yellow','green','blue','orange','purple'}

% Initialize array to store Kalman filters for each detected person
kalmanFilters = [];

% Read and process each frame of the video
while hasFrame(videoObj)
    frame = readFrame(videoObj);  % Read a single frame

    % Pre-processing to improve image quality
    frame = histeq(frame);  % Improve contrast
    frame = imgaussfilt(frame, 2);  % Apply Gaussian blur for noise reduction

    [bboxes, scores] = peopleDetector.step(frame); % Detect people

    % Update or initialize Kalman filters for detected people
    kalmanFilters = updateKalmanFilters(kalmanFilters, bboxes);
    
    % Draw bounding boxes around detected or predicted people
    % frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores,'Color','red');
    % Annotate frame with colored bounding boxes
    for i = 1:size(bboxes, 1)
        disp(colors{i})
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes(i,:), scores(i), 'Color', colors{i});
    end

    imshow(frame); % Display the annotated frame
    pause(1/videoObj.FrameRate); % Pause for the duration of one frame
end



