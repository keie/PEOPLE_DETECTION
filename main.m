% % Create a VideoReader object to read the MP4 file
% videoObj = VideoReader('/Users/appleuser/Desktop/pazmany/algorithms_image_processing/assignment_2/video/input_video.mp4');
% 
% % Create a People Detector object
% peopleDetector = vision.PeopleDetector;
% 
% colors = {'red','yellow','green','blue','orange','purple'}
% 
% % Initialize array to store Kalman filters for each detected person
% kalmanFilters = [];
% 
% % Read and process each frame of the video
% while hasFrame(videoObj)
%     frame = readFrame(videoObj);  % Read a single frame
% 
%     % Pre-processing to improve image quality
%     frame = histeq(frame);  % Improve contrast
%     frame = imgaussfilt(frame, 2);  % Apply Gaussian blur for noise reduction
% 
%     [bboxes, scores] = peopleDetector.step(frame); % Detect people
% 
%     % Update or initialize Kalman filters for detected people
%     kalmanFilters = updateKalmanFilters(kalmanFilters, bboxes);
% 
%     % Draw bounding boxes around detected or predicted people
%     % frame = insertObjectAnnotation(frame, 'rectangle', bboxes, scores,'Color','red');
%     % Annotate frame with colored bounding boxes
%     for i = 1:size(bboxes, 1)
%         disp(colors{i})
%         frame = insertObjectAnnotation(frame, 'rectangle', bboxes(i,:), scores(i), 'Color', colors{i});
%     end
% 
%     imshow(frame); % Display the annotated frame
%     pause(1/videoObj.FrameRate); % Pause for the duration of one frame
% end


% Crear un VideoReader para leer el archivo MP4
videoObj = VideoReader('/Users/appleuser/Desktop/pazmany/algorithms_image_processing/assignment_2/video/input_video.mp4');

% Crear un detector de personas
peopleDetector = vision.PeopleDetector;

% Inicializar colores
colors = {'red', 'yellow', 'green', 'blue', 'orange', 'purple'};

% Inicializar contenedores para Kalman Filters y IDs
kalmanFilters = [];
personIDs = [];
nextID = 1;

% Leer y procesar cada fotograma del video
while hasFrame(videoObj)
    frame = readFrame(videoObj); % Leer un solo fotograma

    % Pre-procesamiento para mejorar la calidad de la imagen
    frame = histeq(frame); % Mejorar contraste
    frame = imgaussfilt(frame, 2); % Aplicar desenfoque gaussiano

    [bboxes, scores] = peopleDetector.step(frame); % Detectar personas

    % Actualizar o inicializar Kalman filters y IDs para personas detectadas
    [kalmanFilters, personIDs, nextID] = updateKalmanFilters(kalmanFilters, personIDs, bboxes, nextID);

    % Dibujar bounding boxes con ID y color para cada persona
    for i = 1:min(length(personIDs), size(bboxes, 1))
        colorIndex = mod(personIDs(i) - 1, length(colors)) + 1;
        label = sprintf('ID: %d', personIDs(i));
        frame = insertObjectAnnotation(frame, 'rectangle', bboxes(i,:), label, 'Color', colors{colorIndex});
    end


    imshow(frame); % Mostrar el fotograma anotado
    pause(1/videoObj.FrameRate); % Pausar por la duración de un fotograma
end



