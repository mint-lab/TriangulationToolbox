close all;
clear all;

disp('== Localization Evaluatioin (with Random Landmarks) for Triangulation Toolbox ==');

% Configure experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.dim       = 2;                       % Dimension of localization (2 or 3)
config.observe   = [];                      % An observation function to apply noise
                                            %  (e.g. @observe_bearing / c.f. empty []: noise is applied to the map)
config.space     = [100, 100, 50];          % Size of the operating space
config.pool      = 10000;                   % The number of pre-generated landmarks (> 5)
config.trial     = 2000;                    % The number of trials (> 1)
config.pose      = [50, 50, 0, 0, 0, pi/4]; % Pose of the target object
config.fixNoise  = 0.1;                     % Standard deviation of noise (default)
config.fixN      = 4;                       % The number of landmarks for localization (default)
config.varNoise  = 0:0.1:1;                 % Range of std. of noise
config.varN      = [2, 3, 4, 5, 8, 16, 32, 64, 128]; % Range of the number of landmarks for localization
config.algorithm = ...                      % Description of localization algorithms
{                                                                                                           ...
  % #, Dim, Name,           Local. Function,            Observation Function, Min. N, Valid,    Line Sytle; ...
    1,  2,  'Sayed05-TOA2D',@localize2d_sayed05_toa,    @observe_distance,         3, [1 1 0 0 0 0], 'kx:'; ...
    2,  2,  'Sayed05-TDOA', @localize2d_sayed05_tdoa,   @observe_distance_relative,3, [1 1 0 0 0 0], 'k--'; ...
    3,  2,  'Betke97',      @localize2d_betke97,        @observe_bearing,          3, [1 1 0 0 0 1], 'gd-'; ...
    4,  2,  'Shim02-Alg',   @localize2d_shimshoni02_algebraic, @observe_bearing,   3, [1 1 0 0 0 1], 'b--'; ...
    5,  2,  'Shim02-Imp',   @localize2d_shimshoni02_improved,  @observe_bearing,   3, [1 1 0 0 0 1], 'b+-'; ...
    6,  2,  'Se05',         @localize2d_se05,           @observe_displacement,     2, [1 1 0 0 0 1], 'rs-'; ...
    7,  2,  'Sayed05-AOA',  @localize2d_sayed05_aoa,    @observe_displacement,     2, [1 1 0 0 0 1], 'ko-'; ...
    8,  2,  'Pose',         @localize2d_pose,           @observe_pose,             1, [1 1 0 0 0 1], 'r--'; ...
    9,  3,  'Sayed05-TOA3D',@localize3d_sayed05_toa,    @observe_distance,         4, [1 1 1 0 0 0], 'kx:'; ...
   10,  3,  'Thomas05',     @localize3d_thomas05,       @observe_distance,         3, [1 1 1 0 0 0], 'm+-'; ...
};
config.verbose   = true;                    % Show progress of experiments (true or false)
config.warning   = 'off';                   % Show warning during experiments ('on' or 'off')
config.matLoad   = false;                   % Use saved MAT-file without experiments (true or false)
config.matFile   = 'run_eval_random.mat';   % Filename for loading and saving MAT-file
config.csvFile   = 'run_eval_random.csv';   % Filename for writing CSV-file

variable.name    = {'Magnitude of Noise [m]', 'Number of Landmarks'}; % Name of independent variables
variable.value   = {config.varNoise, config.varN};                  % Range of independent variables
variable.repr    = {config.varNoise, config.varN};                  % Range of independent variables (in graphs and table)
variable.format  = {'%.1f', '%d'};                                  % Format for printing text

criteria.name    = {'Position Error [m]', 'Orientation Error [deg]', ...
                    'Computing Time [msec]', 'Number of Failures'}; % Name of evaluation criteria
criteria.repr    = {@median, @median, @median, @sum};               % Functions for calculating representive values
                                                                    %  (e.g. mean, median, std, and sum)
criteria.format  = {'%.6f', '%.3f', '%.6f', '%d'};                  % Format for printing text

% Perform experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.algoDims  = 2;
config.algoName  = 3;
config.algoEstm  = 4;
config.algoObsv  = 5;
config.algoMinN  = 6;
config.algoVald  = 7;
config.algoLine  = 8;
config.algoSelM  = 1:size(config.algorithm,1);
choose = [config.algorithm{:,config.algoDims}] == config.dim;
if ~isempty(config.observe)
    for m = 1:length(choose)
        if choose(m) == 1 && ~isequal(config.algorithm{m,config.algoObsv}, config.observe)
            choose(m) = 0;
        end
    end
end
config.algoSelM = config.algoSelM(choose);
if ~config.matLoad
    % 1. Generate features randomly
    if config.pool <= 5
        error('TT: The number of landmarks ''config.pool'' should be more than 5!');
    end
    pool = zeros(config.pool,6);
    pool(:,1:config.dim) = repmat(config.space(1:config.dim), config.pool, 1) .* rand(config.pool,config.dim);
    pool(:,4:6)          = 2 * pi * rand(config.pool,3) - pi;

    % 2. Execute each algorithm 'config.trial' times
    if config.trial <= 1
        error('TT: The number of trials ''config.trial'' should be more than 1!');
    end
    for ex = 1:length(variable.value)
        for v = 1:length(variable.value{ex})
            record.perf{ex,v} = inf * ones(config.trial, length(criteria.name), size(config.algorithm,1));
            record.pose{ex,v} = zeros(config.trial, 6, size(config.algorithm,1));
        end
    end
    if isequal(config.warning, 'off');
        warning off;
    end
    for ex = 1:length(variable.value)                           % Loop for 'ex'periments
        if config.verbose
            fprintf('\n==== Progress on Experiment #%d: %s ====\n', ex, variable.name{ex});
        end
        for v = 1:length(variable.value{ex})                    % Loop for 'v'ariables
            param = [config.fixNoise, config.fixN];
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
                if isempty(config.observe)
                    noisyMap(:,1:config.dim) = apply_noise_gauss(cleanMap(:,1:config.dim), param(1));
                end

                for m = config.algoSelM                              % Loop for 'm'ethods
                    % Check the operating condition
                    if (config.dim > config.algorithm{m,config.algoDims}) || ...
                       (param(2) < config.algorithm{m,config.algoMinN})
                        continue;
                    end

                    % Estimate pose
                    obsData = feval(config.algorithm{m,config.algoObsv}, cleanMap, config.pose);
                    if isequal(config.algorithm{m,config.algoObsv}, config.observe)
                        obsData = apply_noise_gauss(obsData, param(1));
                    end
                    tic;
                    [pose, valid] = feval(config.algorithm{m,config.algoEstm}, obsData, noisyMap);
                    elapse = toc * 1000; % [sec] to [msec]
                    if size(pose,1) > 1  % When there are multiple solutions
                        bestIndex = 1;
                        bestError = inf;
                        for i = 1:size(pose,1)
                            err = norm(config.pose - pose(i,:));
                            if bestError > err
                                bestIndex = i;
                                bestError = err;
                            end
                        end
                        pose = pose(bestIndex,:);
                        valid = valid(bestIndex,:);
                    end
                    record.perf{ex,v}(t,1,m) = error_position(config.pose(1:3), pose(1:3));
                    if valid(end)
                        record.perf{ex,v}(t,2,m) = tran_rad2deg(error_orientation(config.pose(4:6), pose(4:6))); % [rad] to [deg]
                    end
                    record.perf{ex,v}(t,3,m) = elapse;
                    record.perf{ex,v}(t,4,m) = ~isequal(valid, config.algorithm{m,config.algoVald});
                    record.pose{ex,v}(t,:,m) = pose;
                end
            end % End of 'for t'

            % Print progress
            if config.verbose
                fprintf('Each method performed %d times. [Param] %s\n', config.trial, num2str(param));
            end
        end % End of 'for v'
    end % End of 'for ex'
    warning on;
    if ~isempty(config.matFile)
        save(config.matFile, 'config', 'variable', 'criteria', 'record');
    end
else
    backup.matFile   = config.matFile;
    backup.csvFile   = config.csvFile;
    load(config.matFile);
    config.matFile   = backup.matFile;
    config.csvFile   = backup.csvFile;
end % End of 'if config.matLoad'

% Show experimental results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3. Retrieve 'result' from 'record'
for ex = 1:length(variable.value)
    for cr = 1:length(criteria.name)
        result{ex,cr} = zeros(size(config.algorithm,1), length(variable.value{ex}));
    end
end
for ex = 1:length(variable.value)
    for cr = 1:length(criteria.name)
        for v = 1:length(variable.value{ex})
            for m = config.algoSelM
                result{ex,cr}(m,v) = feval(criteria.repr{cr}, record.perf{ex,v}(:,cr,m));
            end
        end
    end
end

% 4. Write 'result' to a text file
if ~isempty(config.csvFile)
    fid = fopen(config.csvFile, 'wt');
    if fid < 0
        error('TT: Cannot open a file, ''%s''!', config.csvFile);
    end
    for ex = 1:length(variable.value)
        fprintf(fid, '==== Results on Experiment #%d: %s ====\n', ex, variable.name{ex});
        for cr = 1:length(criteria.name)
            % Print header
            fprintf(fid, '\n%d) %s\n', cr, criteria.name{cr});
            fprintf(fid, '%s', variable.name{ex});
            for v = 1:length(variable.value{ex})
                fprintf(fid, ', %.1f', variable.repr{ex}(v));
            end
            fprintf(fid, '\n');

            % Print results of each method
            for m = config.algoSelM
                fprintf(fid, '%s', config.algorithm{m,config.algoName});
                for v = 1:length(variable.value{ex})
                    fprintf(fid, [', ', criteria.format{cr}], result{ex,cr}(m,v));
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
            for m = config.algoSelM
                if (cr == 2) && (config.algorithm{m,config.algoVald}(end) == 0), continue; end
                plot(variable.repr{ex}, result{ex,cr}(m,:), ...
                    config.algorithm{m,config.algoLine}, 'LineWidth', 2, 'MarkerSize', 10);
                isDrawn = [isDrawn, m];
            end
            if ex == 2
                set(gca, 'XScale', 'log');
                set(gca, 'XTick', [2, 4, 8, 16, 32, 64, 128]);
                set(gca, 'XLim', [2, 128]);
            end
            %title(sprintf('Experiment #%d: %s - %s', ex, variable.name{ex}, criteria.name{cr}), 'FontSize', 12);
            xlabel(variable.name{ex}, 'FontSize', 12);
            ylabel(criteria.name{cr}, 'FontSize', 12);
            legend(config.algorithm(isDrawn,config.algoName), 'FontSize', 12);
        hold off;
    end
end
