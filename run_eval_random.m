close all;
clear all;

disp('== Localization Evaluatioin for Triangulation Toolbox ==');

% Configure experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.dim       = 2;                           % Dimension of localization (2 or 3)
config.space     = [100, 100, 50];              % Size of the operating space
config.pool      = 1000;                        % The number of pre-generated landmarks (> 5)
config.trial     = 2000;                        % The number of trials (> 1)
config.fixPose   = [50, 50, 0, 0, 0, pi / 4];   % Pose of the target object
config.fixNoise  = 0.1;                         % Standard deviation of noise (default)
config.fixN      = 4;                           % The number of landmarks for localization (default)
config.varNoise  = 0:0.1:1.0;                   % Range of std. of noise
config.varN      = [2, 3, 4, 6, 8, 16, 32, 64]; % Range of the number of landmarks for localization
config.algorithm = ...                          % Description of localization algorithms
{                                                                                                       ...
  % #, Dim, Name,         Local. Function,      Observation Function,     Min. N, Valid,    Line Sytle; ...
    1,  2,  'Sayed05-2D', @localize2d_sayed05,  @observe_distance,             3, [1 1 0 0 0 0], 'kx-'; ...
    2,  2,  'Betke97',    @localize2d_betke97,  @observe_bearing,              3, [1 1 0 0 0 1], 'gd-'; ...
    3,  2,  'Shim02-Alg', @localize2d_shimshoni02_algebraic, @observe_bearing, 3, [1 1 0 0 0 1], 'b--'; ...
    4,  2,  'Shim02-Imp', @localize2d_shimshoni02_improved,  @observe_bearing, 3, [1 1 0 0 0 1], 'b+-'; ...
    5,  2,  'Se05',       @localize2d_se05,     @observe_displacement,         2, [1 1 0 0 0 1], 'rs-'; ...
    6,  3,  'Sayed05-3D', @localize3d_sayed05,  @observe_distance,             4, [1 1 1 0 0 0], 'ko-'; ...
    7,  3,  'Thomas05',   @localize3d_thomas05, @observe_distance,             3, [1 1 1 0 0 0], 'm+-'; ...
};
config.verbose   = true;  % Show progress of experiments (true or false)
config.warning   = 'off'; % Show warning during experiments ('on' or 'off')
config.matLoad   = false; % Use saved MAT-file without experiments (true or false)
config.matFile   = 'run_eval_random.mat'; % Filename for loading and saving MAT-file
config.csvFile   = 'run_eval_random.csv'; % Filename for writing CSV-file
config.showGraph = [1, 2, 3, 4, 5, 6]; % Index of algorithms to show their graphs
config.showHist  = [1, 2, 1];          % Index of experiment, variable, and criteria to show histograms

variable.name   = {'Magnitude of Noise', 'Number of Landmarks'};   % Name of independent variables
variable.value  = {config.varNoise, config.varN};                  % Range of independent variables
variable.format = {'%.1f', '%d'};                                  % Format for printing text

criteria.name   = {'Position Error', 'Orientation Error [deg]', ...
                   'Computing Time [msec]', 'Number of Failures'}; % Name of evaluation criteria
criteria.repr   = {@median, @median, @median, @sum};               % Functions for calculating representive value
                                                                   %  (e.g. mean, median, std, and sum)
criteria.format = {'%.6f', '%.3f', '%.6f', '%d'};                  % Format for printing text

% Perform experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if ~config.matLoad
    % 1. Generate landmarks randomly
    if config.pool <= 5
        error('TT: The number of landmarks, config.pool, should be more than 5!');
    end
    pool = zeros(config.pool,6);
    pool(:,1:config.dim) = repmat(config.space(1:config.dim), config.pool, 1) .* rand(config.pool,config.dim);
    pool(:,4:6)          = 2 * pi * rand(config.pool,3) - pi;

    % 2. Execute each localization 'config.trial' times
    if config.trial <= 1
        error('TT: The number of trials, config.trial, should be more than 1!');
    end
    for ex = 1:length(variable.value)
        record{ex,1} = inf * ones(config.trial, length(variable.value{ex}), size(config.algorithm,1));
        record{ex,2} = inf * ones(config.trial, length(variable.value{ex}), size(config.algorithm,1));
        record{ex,3} = inf * ones(config.trial, length(variable.value{ex}), size(config.algorithm,1));
        record{ex,4} = zeros(config.trial, length(variable.value{ex}), size(config.algorithm,1));
    end
    if isequal(config.warning, 'off');
        warning off;
    end
    for ex = 1:length(variable.value)                           % Loop for 'ex'periments
        if config.verbose
            fprintf('\n==== Progress on Experiment #%d: %s ====\n', ex, variable.name{ex});
        end
        param = [config.fixNoise, config.fixN];
        for v = 1:length(variable.value{ex})                    % Loop for 'v'ariables
            param(ex) = variable.value{ex}(v);
            for t = 1:config.trial                              % Loop for 't'rials
                % Select landmarks randomly
                sample = zeros(1,config.pool,'uint8');
                while sum(sample) < param(2)
                    index = floor(config.pool * rand()) + 1;
                    sample(index) = 1;
                end
                cleanMap = pool(sample == 1,:);
                noisyMap = cleanMap;
                noisyMap(:,1:config.dim) = apply_noise_gauss(cleanMap(:,1:config.dim), param(1));

                for m = 1:size(config.algorithm,1)              % Loop for 'm'ethods
                    % Check the operating condition
                    if (config.dim > config.algorithm{m,2}) || (param(2) < config.algorithm{m,6})
                        continue;
                    end

                    % Estimate pose
                    obsData = feval(config.algorithm{m,5}, noisyMap, config.fixPose);
                    tic;
                    [pose, valid] = feval(config.algorithm{m,4}, obsData, cleanMap);
                    record{ex,3}(t,v,m) = toc * 1000; % [sec] to [msec]
                    if isequal(valid, config.algorithm{m,7})
                        record{ex,1}(t,v,m) = error_position(config.fixPose(1:3), pose(1:3));
                        record{ex,2}(t,v,m) = tran_rad2deg(error_orientation(config.fixPose(4:6), pose(4:6))); % [rad] to [deg]
                    else
                        record{ex,4}(t,v,m) = 1;
                    end
                end
            end % End of 'for t'

            % Print progress
            if config.verbose
                fprintf([' [', variable.name{1}(1), variable.format{1}, ', ', ...
                               variable.name{2}(1), variable.format{2}, '] ', ...
                         'Each method performed %d times.\n'], param, config.trial);
            end
        end % End of 'for v'
    end % End of 'for ex'
    warning on;
    if ~isempty(config.matFile)
        backup.matFile   = config.matFile;
        backup.csvFile   = config.csvFile;
        backup.showGraph = config.showGraph;
        backup.showHist  = config.showHist;
        save(config.matFile, 'config', 'variable', 'criteria', 'record');
        config.matFile   = backup.matFile;
        config.csvFile   = backup.csvFile;
        config.showGraph = backup.showGraph;
        config.showHist  = backup.showHist;
    end
else
    load(config.matFile);
end % End of 'if config.matLoad'

% Show experimental results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Retrieve 'result' from 'record'
for ex = 1:length(variable.value)
    result{ex} = zeros(size(config.algorithm,1), length(variable.value{ex}), length(criteria.name));
end
for ex = 1:length(variable.value)
    for cr = 1:length(criteria.name)
        for m = 1:size(config.algorithm,1)
            result{ex}(m,:,cr) = feval(criteria.repr{cr}, record{ex,cr}(:,:,m));
        end
    end
end

% 4. Write 'result' to a text file
if ~isempty(config.csvFile)
    fid = fopen(config.csvFile, 'wt');
    if fid < 0
        error('TT: Cannot open a file, %s!', config.csvFile);
    end
    for ex = 1:length(variable.value)
        fprintf(fid, '==== Results on Experiment #%d: %s ====\n', ex, variable.name{ex});
        for cr = 1:length(criteria.name)
            % Print header
            fprintf(fid, '\n%d) %s\n', cr, criteria.name{cr});
            fprintf(fid, '%s', variable.name{ex});
            for v = 1:length(variable.value{ex})
                fprintf(fid, ', %.1f', variable.value{ex}(v));
            end
            fprintf(fid, '\n');

            % Print results of each method
            for m = 1:size(config.algorithm,1)
                fprintf(fid, '%s', config.algorithm{m,3});
                for i = 1:length(variable.value{ex})
                    fprintf(fid, [', ', criteria.format{cr}], result{ex}(m,i,cr));
                end
                fprintf(fid, '\n');
            end
        end
        fprintf(fid, '\n');
    end
    fclose(fid);
end

% 5. Show 'result' as graphs
for ex = 1:length(variable.value)
    for cr = 1:length(criteria.name)
        % Draw results of each method
        isDrawn = [];
        figure('Color', [1, 1, 1]);
        hold on;
            set(gca, 'FontSize', 12);
            box on;
            grid on;
            for m = config.showGraph
                if (cr == 2) && (config.algorithm{m,7}(end) == 0), continue; end
                plot(variable.value{ex}, result{ex}(m,:,cr), ...
                    config.algorithm{m,8}, 'LineWidth', 2, 'MarkerSize', 10);
                isDrawn = [isDrawn, m];
            end
            title(sprintf('Experiment #%d: %s - %s', ex, variable.name{ex}, criteria.name{cr}), 'FontSize', 12);
            xlabel(variable.name{ex}, 'FontSize', 12);
            ylabel(criteria.name{cr}, 'FontSize', 12);
            legend(config.algorithm(isDrawn,3), 'FontSize', 10);
        hold off;
    end
end

% 6. Show 'record' as histograms
ex = config.showHist(1);
v  = config.showHist(2);
cr = config.showHist(3);
param = [config.fixNoise, config.fixN];
param(ex) = variable.value{ex}(v);
histX = 0:0.02:1;
figure('Color', [1, 1, 1]);
hold on;
    set(gca, 'FontSize', 12);
    box on;
    grid on;
    for m = config.showGraph
        histN = histc(record{ex,cr}(:,v,m), histX);
        histN = histN / config.trial;
        plot(histX, histN, config.algorithm{m,8}, 'LineWidth', 2, 'MarkerSize', 2);
    end
    title(sprintf(['Histogram: %s at (', variable.format{1}, ', ', variable.format{2}, ')'], ...
        criteria.name{cr}, param), 'FontSize', 12);
    xlabel(variable.name{ex}, 'FontSize', 12);
    ylabel('Frequency Ratio', 'FontSize', 12);
    legend(config.algorithm(config.showGraph,3), 'FontSize', 10);
hold off;
