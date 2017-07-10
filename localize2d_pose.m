function [pose, valid] = localize2d_pose(data, map)
%LOCALIZE2D_POSE  Estimate 2D position and orientation using poses measured from landmarks (N >= 1)
%
%   [POSE, VALID] = LOCALIZE2D_POSE(DATA, MAP)
%       (matrix) DATA : The measured pose from landmarks (Nx6 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_POSE, for the convention of DATA, MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 2D position and orientation,
%       the expected VALID is [true, true, false, false, false, true].
%
%   Example:
%       N = 1;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = [10 * rand(N,2), zeros(N,3), 2 * pi (rand(N,1) - 0.5)]; % Random measurement
%       [pose, valid] = localize2d_pose(data, map)

if size(data,1) < 1
    error('DATA has less number of observations!');
end
if size(data,2) ~= 6
    error('DATA has wrong size!');
end

theta = map(:,6) - data(:,6);
c = cos(theta);
s = sin(theta);
x = map(:,1) - c .* data(:,1) + s .* data(:,2);
y = map(:,2) - s .* data(:,1) - c .* data(:,2);

pose = [mean(x), mean(y), 0, 0, 0, atan2(mean(s), mean(c))];
valid = [true, true, false, false, false, true];
