% function [kalmanFilters, colors] = updateKalmanFilters(kalmanFilters, bboxes, colors)
%     % Function to update or initialize Kalman filters and assign colors
%     for i = 1:size(bboxes, 1)
%         bbox = bboxes(i, :);
%         if i > numel(kalmanFilters)
%             % Initialize a new Kalman filter for this detection
%             kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
%                                                  bbox(1:2), [1 1]*1e5, ...
%                                                  [25, 10], 25);
%             kalmanFilters{i} = kalmanFilter;
% 
%             % Assign a random color for this filter
%             colors{i} = rand(1, 3);
%         else
%             % Update existing Kalman filter with new detection
%             kalmanFilter = kalmanFilters{i};
%             bbox = correct(kalmanFilter, bbox(1:2));
%         end
%     end
% end

function [kalmanFilters, personIDs, nextID] = updateKalmanFilters(kalmanFilters, personIDs, bboxes, nextID)
    % Función para actualizar o inicializar los filtros de Kalman y asignar IDs
    numAssigned = 0; % Contador para las detecciones ya asignadas

    for i = 1:size(bboxes, 1)
        bbox = bboxes(i, :);

        if numAssigned < numel(kalmanFilters)
            % Actualizar filtro de Kalman existente con nueva detección
            kalmanFilter = kalmanFilters{numAssigned + 1};
            bbox = correct(kalmanFilter, bbox(1:2));
            numAssigned = numAssigned + 1;
        else
            % Inicializar un nuevo filtro de Kalman para esta detección
            kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
                                                 bbox(1:2), [1 1]*1e5, ...
                                                 [25, 10], 25);
            kalmanFilters{end + 1} = kalmanFilter;

            % Asignar un nuevo ID para esta detección
            personIDs(end + 1) = nextID;
            nextID = nextID + 1;
        end
    end
end
