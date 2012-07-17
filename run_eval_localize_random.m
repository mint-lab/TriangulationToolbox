close all;
clear all;

disp('== Localization Evaluatioin for Triangulation Toolbox ==');

% The given configuration
config.space     = [100, 100, 10];
config.dim       = 3;
config.pool      = 1000;
config.trial     = 100;
config.verbose   = true;
config.filename  = 'run_eval_localize_random.csv';
config.fixPose   = [50, 50, 0, 0, 0, pi / 4];
config.fixNoise  = 0.1;
config.fixN      = 4;
config.varNoise  = 0.1:0.1:1.0;
config.varN      = [2, 3, 4, 8, 16, 32, 64];
config.varMethod =                                                                            ...
{                                                                                             ...
  % #, Dim, Name,         Local. Function,      Obs. Function,         Min. N, Valid;         ...
    1,  2,  'Sayed05-2D', @localize2d_sayed05,  @observe_distance,     3,      [1 1 0 0 0 0]; ...
    2,  2,  'Betke97',    @localize2d_betke97,  @observe_bearing,      3,      [1 1 0 0 0 1]; ...
    3,  2,  'Shimshoni02-Alg', @localize2d_shimshoni02_algebraic, @observe_bearing, 3, [1 1 0 0 0 1]; ...
    4,  2,  'Shimshoni02-Imp', @localize2d_shimshoni02_improved,  @observe_bearing, 3, [1 1 0 0 0 1]; ...
    5,  2,  'Se05',       @localize2d_se05,     @observe_displacement, 2,      [1 1 0 0 0 1]; ...
    6,  3,  'Sayed05-3D', @localize3d_sayed05,  @observe_distance,     4,      [1 1 1 0 0 0]; ...
    7,  3,  'Thomas05',   @localize3d_thomas05, @observe_distance,     3,      [1 1 1 0 0 0]; ...
};

if config.trial <= 1
    error('The number of trials, config.trial, should be more than 1!');
end
if config.pool  <= 5
    error('The number of landmarks in the pool, config.pool, should be more than 5!');
end

% Generate a pool of landmarks
pool = zeros(config.pool,6);
pool(:,1:config.dim) = repmat(config.space(1:config.dim), config.pool, 1) .* rand(config.pool,config.dim);
pool(:,4:6)          = 2 * pi * rand(config.pool,3) - pi;

% Perform experiment #1 and #2
record.position =                                                                ...
{                                                                                ...
    inf * ones(config.trial, length(config.varNoise), size(config.varMethod,1)), ...
    inf * ones(config.trial, length(config.varN),     size(config.varMethod,1)), ...
};
record.orientation = record.position;
record.runtime     = record.position;
variable.name  = {'Noise', 'N'};
variable.value = {config.varNoise, config.varN};
warning off;
for ex = 1:length(variable.value)                           % Loop for experiments
    if config.verbose
        fprintf('\n==== Progress on Experiment #%d: %s ====\n', ex, variable.name{ex});
    end
    param = [config.fixNoise, config.fixN];
    for v = 1:length(variable.value{ex})                    % Loop for values
        param(ex) = variable.value{ex}(v);
        for t = 1:config.trial                              % Loop for trials
            % 1. Select landmarks randomly
            sample = zeros(1,config.pool,'uint8');
            while sum(sample) < param(2)
                index = floor(config.pool * rand()) + 1;
                sample(index) = 1;
            end
            cleanMap = pool(sample == 1,:);
            noisyMap = cleanMap;
            noisyMap(:,1:config.dim) = apply_noise_gauss(cleanMap(:,1:config.dim), param(1));

            for m = 1:size(config.varMethod,1)              % Loop for methods
                % 2. Check the operating condition
                if (config.dim ~= config.varMethod{m,2}) || (param(2) < config.varMethod{m,6})
                    continue;
                end

                % 3. Estimate pose
                obsData = feval(config.varMethod{m,5}, noisyMap, config.fixPose);
                tic;
                [pose, valid] = feval(config.varMethod{m,4}, obsData, cleanMap);
                record.runtime{ex}(t,v,m) = toc * 1000; % [sec] to [msec]
                if isequal(valid, config.varMethod{m,7})
                    record.position{ex}(t,v,m) = error_position(config.fixPose(1:3), pose(1:3));
                    record.orientation{ex}(t,v,m) = tran_rad2deg(error_orientation(config.fixPose(4:6), pose(4:6))); % [rad] to [deg]
                end
            end
        end

        % 4. Print progress
        if config.verbose
            fprintf('[Noise %.1f, N: %d] Each method performed %d times.\n', ...
                param(1), param(2), config.trial);
        end
    end
end
warning on;

% Write results to a text file
criteria.name = {'1) Position Error', '2) Orientation Error', ...
                 '3) Computing Time [msec]', '4) Number of Failure'};
criteria.format = {'%.6f', '%.3f', '%.6f', '%d'};
if isempty(config.filename)
    fid = 1;
else
    fid = fopen(config.filename, 'wt');
end
for ex = 1:length(variable.value)
    fprintf(fid, '\n==== Results on Experiment #%d: %s ====\n', ex, variable.name{ex});
    for cr = 1:length(criteria.name)
        % 1. Print header
        fprintf(fid, '\n%s\n', criteria.name{cr});
        fprintf(fid, '%s', variable.name{ex});
        for v = 1:length(variable.value{ex})
            fprintf(fid, ', %.1f', variable.value{ex}(v));
        end
        fprintf(fid, '\n');

        % 2. Print results by each method
        for m = 1:size(config.varMethod,1)
            switch cr
                case 1
                    result = median(record.position{ex}(:,:,m));
                case 2
                    result = median(record.orientation{ex}(:,:,m));
                case 3
                    result = median(record.runtime{ex}(:,:,m));
                case 4
                    result = sum(record.position{ex}(:,:,m) == inf);
            end
            fprintf(fid, '%s', config.varMethod{m,3});
            for i = 1:size(result,2)
                fprintf(fid, [', ', criteria.format{cr}], result(i));
            end
            fprintf(fid, '\n');
        end
    end
end
if ~isempty(config.filename)
    fclose(fid);
end

% Visualize results
