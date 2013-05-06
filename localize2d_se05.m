function [pose, valid] = localize2d_se05(data, map)
%LOCALIZE2D_SE05  Estimate 2D position and orientation using displacements
%   measured from landmarks (N >= 2)
%
%   [POSE, VALID] = LOCALIZE2D_SE05(DATA, MAP)
%       (matrix) DATA : The measured displacement from landmarks (Nx3 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_DISPLACEMENT, for the convention of DATA,
%       MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 2D position and orientation,
%       the expected VALID is [true, true, false, false, false, true].
%
%   Note: This implementation is extended from Se et al. [1], so it can take into account
%       more than two measurements in lease-squares sense.
%
%   Reference:
%       [1] S. Se et al., Vision-Based Global Localization and Mapping for Mobile Robots,
%           IEEE Transactions on Robotics, Vol. 21, No. 3, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1435480
%
%   Example:
%       N = 2;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = [10 * rand(N,2), zeros(N,1)]; % Random measurement
%       [pose, valid] = localize2d_se05(data, map)
%
%   See also localize2d_sayed05_toa.

if size(data,1) < 2
    error('DATA has less number of observations!');
end
if size(data,2) ~= 3
    error('DATA has wrong size!');
end

A = data(2:end,1) - data(1,1);
B = data(2:end,2) - data(1,2);
C = map(2:end,1) - map(1,1);
D = map(2:end,2) - map(1,2);
theta = pinv([A, B; B, -A]) * [C; D]; % Solve 'theta' in least squares
align = map(1,1:2)' - [data(1,1), data(1,2); data(1,2), -data(1,1)] * theta;

pose = [align(1), align(2), 0, 0, 0, -atan2(theta(2), theta(1))];
    % Q) Why '-atan2(...)' for orientation?
    % A) In this toolbox, CCW is positive rotation, but the paper [1] uses it as negative.
valid = [true, true, false, false, false, true];
