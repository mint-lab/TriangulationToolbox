function [pose, valid] = localize2d_sayed05_tdoa(data, map)
%LOCALIZE2D_SAYED05_TDOA  Estimate 2D position using relative distances measured from landmarks (N >= 3)
%
%   [POSE, VALID] = LOCALIZE2D_SAYED05_TDOA(DATA, MAP)
%       (matrix) DATA : The relative distances w.r.t. the first observation (Nx1 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_DISTANCE, for the convention of DATA,
%       MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 2D position, the expected
%       VALID is [true, true, false, false, false, false].
%
%   Reference:
%       [1] A. H. Sayed et al., Network-based Wireless Location,
%           IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275
%
%   Example:
%       N = 3;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = 10 * rand(N,1); % Random measurement
%       [pose, valid] = localize2d_sayed05_tdoa(data, map)
%
%   See also localize_sayed05_toa.

if size(data,1) < 3
    error('DATA has less number of observations!');
end
if size(data,2) ~= 1
    error('DATA has wrong size!');
end

origin = map(1,1:2);
H = map(2:end,1:2) - repmat(origin, size(map,1) - 1, 1);
if rank(H) < 2
    warning('All landmarks on MAP lie on a line!');
end
c = -data(2:end);
d = 0.5 * (H(:,1).^2 + H(:,2).^2 - data(2:end).^2);
x = pinv(H) * (data(1) * c + d); % Solve H * x = data(1) * c + d

pose = [x' + origin, 0, 0, 0, 0];
valid = [true, true, false, false, false, false];
