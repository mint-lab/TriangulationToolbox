close all;
clear all;

disp('== Localization Evaluatioin for Triangulation Toolbox ==');

% The given configuration
config.space     = [100, 100, 4];
config.dim       = 2;
config.pool      = 100;
config.trial     = 2;
config.verbose   = true;
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

% Generate a pool of landmarks
pool = zeros(config.pool,6);
pool(:,1:config.dim) = repmat(config.space(1:config.dim), config.pool, 1) .* rand(config.pool,config.dim);
pool(:,4:6) = 2 * pi * rand(config.pool,3) - pi;

% Perform experiment #1 and #2
record.position =                                                           ...
{                                                                           ...
    -ones(config.trial, size(config.varMethod,1), length(config.varNoise)), ...
    -ones(config.trial, size(config.varMethod,1), length(config.varN))      ...
};
record.orientation = record.position;
record.runtime = record.position;
variable = {config.varNoise, config.varN};
for ex = 1:length(variable)                                 % Experiments
    if config.verbose
        fprintf('==== Experiment #%d ====\n', ex);
    end
    param = [config.fixNoise, config.fixN];
    for v = 1:length(variable{ex})                          % Values
        param(ex) = variable{ex}(v);
        for t = 1:config.trial                              % Trials
            % 1. Select landmarks randomly
            sample = zeros(1,config.pool,'uint8');
            while sum(sample) < param(2)
                index = floor(config.pool * rand()) + 1;
                sample(index) = 1;
            end
            cleanMap = pool(sample == 1,:);
            noisyMap = cleanMap;
            noisyMap(:,1:config.dim) = apply_noise_gauss(cleanMap(:,1:config.dim), param(1));

            for m = 1:size(config.varMethod,1)              % Methods
                % 2. Check the operating condition
                if (config.dim ~= config.varMethod{m,2}) || (param(2) < config.varMethod{m,6})
                    continue;
                end

                % 3. Estimate pose
                obsData = feval(config.varMethod{m,5}, noisyMap, config.fixPose);
                tic;
                [pose, valid] = feval(config.varMethod{m,4}, obsData, cleanMap);
                record.runtime{ex}(t,m,v) = toc;
                if isequal(valid, config.varMethod{m,7})
                    record.position{ex}(t,m,v) = error_position(config.fixPose(1:3), pose(1:3));
                    record.orientation{ex}(t,m,v) = error_orientation(config.fixPose(4:6), pose(4:6));
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

% Visualize results

