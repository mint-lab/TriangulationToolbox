close all;
clear all;

disp('== Localization Evaluatioin (with Roh'' Angulation Dataset) for Triangulation Toolbox ==');

% Configure experiments %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
config.mapFile = 'dataset_roh/landmark.csv'; % A map file
config.dataFile = ... % A list of data files
{                                  ...
    'dataset_roh/(X1.5,Y1.5).csv'; ...
    'dataset_roh/(X1.5,Y3.0).csv'; ...
    'dataset_roh/(X1.5,Y4.5).csv'; ...
    'dataset_roh/(X3.0,Y1.5).csv'; ...
    'dataset_roh/(X3.0,Y3.0).csv'; ...
    'dataset_roh/(X3.0,Y4.5).csv'; ...
    'dataset_roh/(X4.5,Y1.5).csv'; ...
    'dataset_roh/(X4.5,Y3.0).csv'; ...
    'dataset_roh/(X4.5,Y4.5).csv'; ...
};
config.pose = ... % A list of ground truth
[                            ...
    1.5, 1.5, 0, 0, 0, pi/2; ...
    1.5, 3.0, 0, 0, 0, pi/2; ...
    1.5, 4.5, 0, 0, 0, pi/2; ...
    3.0, 1.5, 0, 0, 0, pi/2; ...
    3.0, 3.0, 0, 0, 0, pi/2; ...
    3.0, 4.5, 0, 0, 0, pi/2; ...
    4.5, 1.5, 0, 0, 0, pi/2; ...
    4.5, 3.0, 0, 0, 0, pi/2; ...
    4.5, 4.5, 0, 0, 0, pi/2; ...
];
config.algorithm = ... % Description of localization algorithms
{                                                                                                       ...
  % #, Dim, Name,         Local. Function,      Observation Function,     Min. N, Valid,    Line Sytle; ...
    1,  2,  'Betke97',    @localize2d_betke97,  @observe_bearing,              3, [1 1 0 0 0 1], 'gd-'; ...
    2,  2,  'Shim02-Alg', @localize2d_shimshoni02_algebraic, @observe_bearing, 3, [1 1 0 0 0 1], 'b--'; ...
    3,  2,  'Shim02-Imp', @localize2d_shimshoni02_improved,  @observe_bearing, 3, [1 1 0 0 0 1], 'b+-'; ...
};
config.matFile = 'run_eval_roh.mat';

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

% 1. Read the map file
map = load(config.mapFile);
map = [map, zeros(size(map,1),4)];

record.perf{1,1} = [];
record.pose{1,1} = [];
for d = 1:size(config.dataFile,1)
    % 2. Read each data file
    data = tran_deg2rad(load(config.dataFile{d}))';
    truth = config.pose(d,:);
    for t = 1:size(data,2)
        obsData = [data(:,t), zeros(size(data,1),1)];
        set.perf = zeros(1,4,size(config.algorithm,1));
        set.pose = zeros(1,6,size(config.algorithm,1));
        for m = config.algoSelM
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
