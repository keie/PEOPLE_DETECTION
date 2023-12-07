% % Create a VideoReader object to read the MP4 file
% videoObj = VideoReader('/Users/appleuser/Desktop/pazmany/algorithms_image_processing/assignment_2/video/input_video.mp4');
% 
% % Create a People Detector object
% peopleDetector = vision.PeopleDetector;
% 
% % Read and process each frame of the video
% while hasFrame(videoObj)
%     frame = readFrame(videoObj);  % Read a single frame
% 
%     % Pre-processing to improve image quality
% 
%     %--enhancemene improvement
%     frame = histeq(frame);  % Improve contrast
%     frame = imgaussfilt(frame, 2);  % Apply Gaussian blur for noise reduction
%     %--enhancemene improvement
% 
%     [bboxes, scores] = peopleDetector.step(frame); % Detect people
% 
%     % Draw bounding boxes around detected people
%     if ~isempty(bboxes)
%         frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores);
%     end
% 
%     imshow(frame); % Display the annotated frame
%     pause(1/videoObj.FrameRate); % Pause for the duration of one frame
% end




% Create a VideoReader object to read the MP4 file
videoObj = VideoReader('/Users/appleuser/Desktop/pazmany/algorithms_image_processing/assignment_2/video/input_video.mp4');

% Create a People Detector object
peopleDetector = vision.PeopleDetector;

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
    frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores);

    imshow(frame); % Display the annotated frame
    pause(1/videoObj.FrameRate); % Pause for the duration of one frame
end



