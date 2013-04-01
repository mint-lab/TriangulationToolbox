close all;
clear all;

disp('== Detailed Analysis of Localization Evaluatioin ==');

target.file = 'result_eval_random(map,2d)/run_eval_random.mat';
target.ex = 1;      % Index of experiment to analyze in detail
target.v = 6;       % Index of variable to analyze in detail
target.binN = 50;   % The number of bins for histogram

% Load the target evaluation file
load(target.file);
if target.ex < 1 || target.ex > length(variable.value)
    error(sprintf('target.ex is out of range, [1, %d]!', length(variable.value)));
end
if target.v < 1 || target.v > length(variable.value{target.ex})
    error(sprintf('target.v is out of range, [1, %d]!', length(variable.value{target.ex})));
end

% Draw distribution of estimated position
figure('Color', [1, 1, 1]);
hold on;
    set(gca, 'FontSize', 12);
    box on;
    grid on;
    for m = config.algoSelM
        plot3(record.pose{target.ex,target.v}(:,1,m), record.pose{target.ex,target.v}(:,2,m), ...
            record.pose{target.ex,target.v}(:,3,m), [config.algorithm{m,config.algoLine}(1), '.'])
    end
    truth = config.pose(1:3);
    line(get(gca, 'XLim'), [truth(2), truth(2)], [truth(3), truth(3)], 'Color', 'y', 'LineWidth', 1);
    line([truth(1), truth(1)], get(gca, 'YLim'), [truth(3), truth(3)], 'Color', 'y', 'LineWidth', 1);
    line([truth(1), truth(1)], [truth(2), truth(2)], get(gca, 'ZLim'), 'Color', 'y', 'LineWidth', 1);
    title(sprintf(['Distribution of Estimated Position: %s at ', variable.format{target.ex}], ...
        variable.name{target.ex}, variable.value{target.ex}(target.v)), 'FontSize', 12);
    xlabel('X [m]', 'FontSize', 12);
    ylabel('Y [m]', 'FontSize', 12);
    zlabel('Z [m]', 'FontSize', 12);
    legend(config.algorithm(config.algoSelM,config.algoName), 'FontSize', 12);
hold off;

% Draw error cumulative histogram (similar to cdf)
for cr = 1:2
    med = median(median(record.perf{target.ex,target.v}(:,cr,config.algoSelM)));
    bins = 0:(3*med/target.binN):3*med;
    isDrawn = [];
    figure('Color', [1, 1, 1]);
    hold on;
        set(gca, 'FontSize', 12);
        box on;
        grid on;
        for m = config.algoSelM
            if (cr == 2) && (config.algorithm{m,config.algoVald}(end) == 0), continue; end
            cdfN = histc(record.perf{target.ex,target.v}(:,cr,m), bins) / config.trial;
            for i = 2:size(cdfN)
                cdfN(i) = cdfN(i-1) + cdfN(i);
            end
            plot(bins, cdfN, config.algorithm{m,config.algoLine}, 'LineWidth', 2, 'MarkerSize', 2);
            isDrawn = [isDrawn, m];
        end
        axis([0, 3*med, 0, 1]);
        title(sprintf(['Cumulative Histogram: %s at ', variable.format{target.ex}], ...
            variable.name{target.ex}, variable.value{target.ex}(target.v)), 'FontSize', 12);
        xlabel(criteria.name{cr}, 'FontSize', 12);
        ylabel('Cumulative Occurence Rate', 'FontSize', 12);
        legend(config.algorithm(isDrawn,config.algoName), 'FontSize', 12);
    hold off;
end
