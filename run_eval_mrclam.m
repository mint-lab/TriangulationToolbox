close all;
clear all;

disp('== Localization Evaluatioin (with MRCLAM Dataset) for Triangulation Toolbox ==');

% Configure experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.dataFile = ... % A list of data files
{                                 ...
    'dataset_mrclam/mrclam1.mat'; ...
    'dataset_mrclam/mrclam2.mat'; ...
    'dataset_mrclam/mrclam3.mat'; ...
    'dataset_mrclam/mrclam4.mat'; ...
    'dataset_mrclam/mrclam5.mat'; ...
    'dataset_mrclam/mrclam6.mat'; ...
    'dataset_mrclam/mrclam7.mat'; ...
    'dataset_mrclam/mrclam8.mat'; ...
    'dataset_mrclam/mrclam9.mat'; ...
};
config.pose = [];
config.algorithm = ... % Description of localization algorithms
{                                                                                                       ...
  % #, Dim, Name,         Local. Function,      Observation Function,     Min. N, Valid,    Line Sytle; ...
    1,  2,  'Sayed05-2D', @localize2d_sayed05,  @observe_distance,             3, [1 1 0 0 0 0], 'kx-'; ...
    2,  2,  'Betke97',    @localize2d_betke97,  @observe_bearing,              3, [1 1 0 0 0 1], 'gd-'; ...
    3,  2,  'Shim02-Alg', @localize2d_shimshoni02_algebraic, @observe_bearing, 3, [1 1 0 0 0 1], 'b--'; ...
    4,  2,  'Shim02-Imp', @localize2d_shimshoni02_improved,  @observe_bearing, 3, [1 1 0 0 0 1], 'b+-'; ...
    5,  2,  'Se05',       @localize2d_se05,     @observe_displacement,         2, [1 1 0 0 0 1], 'rs-'; ...
};
config.matFile = 'run_eval_mrclam.mat';

criteria.name = {'Position Error [m]', 'Orientation Error [deg]', ...
                 'Computing Time [msec]', 'Number of Failures'}; % Name of evaluation criteria
criteria.repr = {@median, @median, @median, @sum}; % Functions for calculating representive values
                                                   %  (e.g. mean, median, std, and sum)
criteria.format = {'%.6f', '%.3f', '%.6f', '%d'};  % Format for printing text

% Perform experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.algoDims = 2;
config.algoName = 3;
config.algoEstm = 4;
config.algoObsv = 5;
config.algoMinN = 6;
config.algoVald = 7;
config.algoLine = 8;
config.algoSelM = 1:size(config.algorithm,1);

for m = 1:size(config.algorithm,1)
    obsFuncName{m} = func2str(config.algorithm{m,config.algoObsv});
end
record.perf{1,1} = [];
record.pose{1,1} = [];
for d = 1:size(config.dataFile,1)
    % 1. Read each data file which contains landmarks, ground truths, and measurements
    data = load(config.dataFile{d});
    for t = 1:size(data.groundtruth,1)
        truth = [data.groundtruth(t,2:3), zeros(1,3), data.groundtruth(t,4)];
        measure = [data.measurement(data.measurement(:,1) == t,2:end)];
        map = [data.landmark(measure(:,1),2:end), zeros(size(measure,1),4)];
        set.perf = zeros(1,4,size(config.algorithm,1));
        set.pose = zeros(1,6,size(config.algorithm,1));
        for m = config.algoSelM
            % 2. Rearrange measurements for each algorithm
            if isequal(obsFuncName{m}, 'observe_distance')
                obsData = measure(:,2);
            elseif isequal(obsFuncName{m}, 'observe_bearing')
                obsData = [measure(:,3), zeros(size(measure,1),1)];
            elseif isequal(obsFuncName{m}, 'observe_displacement')
                obsData = repmat(measure(:,2),1,3) .* ...
                    [cos(measure(:,3)), sin(measure(:,3)), zeros(size(measure,1),1)];
            else
                error(['This dataset cannot be applied to the algorithm #', num2str(m), '!']);
            end

            % 3. Perform each algorithm
            tic;
            [pose, valid] = feval(config.algorithm{m,config.algoEstm}, obsData, map);
            elapse = toc * 1000; % [sec] to [msec]
            set.perf(1,1,m) = error_position(truth(1:3), pose(1:3));
            set.perf(1,2,m) = tran_rad2deg(error_orientation(truth(4:6), pose(4:6))); % [rad] to [deg]
            set.perf(1,3,m) = elapse;
            set.perf(1,4,m) = ~isequal(valid, config.algorithm{m,config.algoVald});
            set.pose(1,:,m) = pose;
        end
        record.perf{1,1} = [record.perf{1,1}; set.perf];
        record.pose{1,1} = [record.pose{1,1}; set.pose];
    end
    disp([' * Experiment on dataset #', num2str(d), ' is complete.']);
end

% 4. Save experimental results
save(config.matFile, 'config', 'criteria', 'record');

% To visualize the result, please use the script, 'run_draw_distribution', with 'target.ex = 1' and 'target.v = 1'.
