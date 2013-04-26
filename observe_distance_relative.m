function [obsData, obsMap] = observe_distance_relative(map, pose, visibleRate)

if nargin < 3
    visibleRate = 1;
end

[obsData, obsMap] = observe_distance(map, pose, visibleRate);
if ~isempty(obsData)
    obsData(2:end) = obsData(2:end) - obsData(1);
end
