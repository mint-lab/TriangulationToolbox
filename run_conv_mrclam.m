close all;
clear all;

disp('== MRCLAM Dataset Conversion for Triangulation Toolbox ==');

% Configure conversion %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dataset.robot_n = 5;
dataset.dir = 'dataset_mrclam/MRCLAM_Dataset1';
dataset.output = 'dataset_mrclam/mrclam1.mat';
outlier.distance = 0.5;     % A threshold for outlier distance [m]
outlier.rotation = 0.5;     % A threshold for outlier bearing [rad]
threshold.elapse = 1;       % The maximum duration [sec] for being stationary state
threshold.distance = 0.01;  % The minimum translational change [m] for being stationary state
threshold.rotation = 0.01;  % The minimum rotational change [rad] for being stationary state
threshold.landmark = 3;     % The minimum number of landmarks

% Convert MRCLAM dataset to landmark-based localization %%%%%%%%%%%%%%%%%%%%%%%
% Read landmarks and their barcode ID
landmark = textread([dataset.dir, '/Landmark_Groundtruth.dat'], '', 'delimiter', '\t', 'commentstyle', 'shell');
landmark = landmark(:,1:3);
barcode = textread([dataset.dir, '/Barcodes.dat'], '', 'delimiter', '\t', 'commentstyle', 'shell');
threshold.minId = min(landmark(:,1));
threshold.maxId = max(landmark(:,1));

% Extract useful measurements for the toolbox
expNum = 0;
groundtruth = [];
measurement = [];
elapsedTime = [];
for r = 1:dataset.robot_n
    % 1. Read measurements and their true trajectory
    prefix = [dataset.dir, '/Robot', num2str(r), '_'];
    truth = textread([prefix, 'Groundtruth.dat'], '', 'delimiter', '\t', 'commentstyle', 'shell');
    measure = textread([prefix, 'Measurement.dat'], '', 'delimiter', '\t', 'commentstyle', 'shell');
    for j = 1:size(barcode,1)
        measure(measure(:,2) == barcode(j,2),2) = barcode(j,1); % Substitute 'barcode ID' to 'subject ID'
    end
    disp([' * Robot #', num2str(r), ': ', num2str(size(measure,1)), ' frames in the measurements.']);
    disp([' * Robot #', num2str(r), ': ', num2str(size(truth,1)), ' frames in the ground truth.']);

    % 2. Calculate piecewise translation and rotation
    delta.time = [0; truth(2:end,1) - truth(1:end-1,1)];
    delta.dist = truth(2:end,2:3) - truth(1:end-1,2:3);
    delta.dist = [0; sqrt(delta.dist(:,1).^2 + delta.dist(:,2).^2)];
    delta.rota = [0; truth(2:end,4) - truth(1:end-1,4)];
    delta.rota(delta.rota >= pi) = delta.rota(delta.rota >= pi) - 2 * pi;
    delta.rota(delta.rota < -pi) = delta.rota(delta.rota < -pi) + 2 * pi;
    delta.rota = abs(delta.rota);

    % 3. Find instantaneously stationary sections in the ground truth
    section.t_index = [];
    section.t_time = [];
    start = 1;
    while start < length(delta.dist)
        acc.time = 0;
        acc.dist = 0;
        acc.rota = 0;
        for j = (start+1):length(delta.dist)
            acc.time = acc.time + delta.time(j);
            acc.dist = acc.dist + delta.dist(j);
            acc.rota = acc.rota + delta.rota(j);
            if (acc.time > threshold.elapse) || (acc.dist > threshold.distance) || (acc.rota > threshold.rotation)
                break;
            end
        end
        if j > (start+1)
            section.t_index = [section.t_index; start, j-1];
            section.t_time = [section.t_time; truth(start,1), truth(j-1,1)];
        end
        start = start + 1;
    end
    disp([' * Robot #', num2str(r), ': ', num2str(size(section.t_index,1)), ' subsequences satisfy time, translation, and orientation conditions.']);

    % 4. Match the stationary sections to the measurements
    section.m_index = zeros(size(section.t_index));
    start = 1;
    for j = 1:size(section.m_index,1)
        minIndex = inf;
        maxIndex = 0;
        for k = start:size(measure,1)
            if measure(k,1) > section.t_time(j,2)
                break;
            elseif measure(k,1) >= section.t_time(j,1)
                minIndex = min(minIndex, k);
                maxIndex = max(maxIndex, k);
            end
        end
        section.m_index(j,:) = [minIndex, maxIndex];
        if ~isinf(minIndex)
            start = minIndex;
        end
    end

    % 5. Select the stationary sections which statisfy the minimum landmark condition
    selected.truth = [];
    selected.measure = [];
    selected.elapse = [];
    for j = 1:size(section.m_index,1)
        if ~isinf(section.m_index(j,1)) && (section.m_index(j,2) ~= 0) % If there are landmarks
            % 5.1. Reject landmarks from other robots
            sample = measure(section.m_index(j,1):section.m_index(j,2),2:end);
            sample = sample(sample(:,1) >= threshold.minId,:);
            sample = sample(sample(:,1) <= threshold.maxId,:);

            % 5.2. Reject outlier data
            if section.t_index(j,1) == section.t_index(j,2)
                pose = truth(section.t_index(j,1),2:4);
            else
                pose = mean(truth(section.t_index(j,1):section.t_index(j,2),2:4));
            end
            map = [];
            for k = 1:size(sample,1)
                map = [map; landmark(landmark(:,1) == sample(k,1), 2:end)];
            end
            distance = observe_distance([map, zeros(size(map,1),4)], [pose(1:2), 0, 0, 0, pose(3)]);
            bearing = observe_bearing([map, zeros(size(map,1),4)], [pose(1:2), 0, 0, 0, pose(3)]);
            diff = abs(sample(:,2:end) - [distance, bearing(:,1)]);
            sample = sample((diff(:,1) < outlier.distance) & (diff(:,2) < outlier.rotation),:);

            % 5.3. Remove duplicated measurements (select the most recent one)
            table = zeros(threshold.maxId,size(sample,2));
            table(sample(:,1),:) = sample;
            mask = zeros(threshold.maxId,1);
            mask(sample(:,1)) = 1;
            sample = table(mask == 1,:);

            % 5.4. Accept a section which statisfies the minimum landmark condition
            if sum(mask) >= threshold.landmark
                expNum = expNum + 1;
                selected.truth = [selected.truth; expNum, pose];
                selected.measure = [selected.measure; expNum * ones(size(sample,1),1), sample];
                selected.elapse = [selected.elapse; expNum, section.t_time(j,2) - section.t_time(j,1)];
            end
        end
    end
    disp([' * Robot #', num2str(r), ': ', num2str(size(selected.truth,1)), ' subsequences satisfy the minimal landmark condition.']);

    groundtruth = [groundtruth; selected.truth];
    measurement = [measurement; selected.measure];
    elapsedTime = [elapsedTime; selected.elapse];
end
disp([' * ', num2str(size(groundtruth,1)), ' subsequences are extracted.']);

% Save landmarks, ground truths, measurements
landmark(:,1) = landmark(:,1) - threshold.minId + 1;        % Start index from 1
measurement(:,2) = measurement(:,2) - threshold.minId + 1;  % Start index from 1
save(dataset.output, 'landmark', 'groundtruth', 'measurement');
