function [pose, valid] = localize3d_sayed05(data, map)
%LOCALIZE3D_SAYED05  Estimate 3D position using distances measured from landmarks (N >= 4)
%
%   [POSE, VALID] = LOCALIZE3D_SAYED05(DATA, MAP)
%       (matrix) DATA : The measured distances from landmarks (Nx1 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_DISTANCE, for the convention of DATA,
%       MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 3D position, the expected
%       VALID is [true, true, true, false, false, false].
%
%   Reference:
%       [1] A. H. Sayed et al., Network-based Wireless Location,
%           IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275
%
%   Example:
%       N = 4;
%       map = [10 * rand(N,3), zeros(N,3)]; % Random 2D landmark map
%       data = 10 * rand(N,1); % Random measurement
%       [pose, valid] = localize3d_sayed05(data, map)
%
%   See also localize2d_sayed05, localize3d_thomas05.

if size(data,1) < 4
    error('DATA has less number of observations!');
end
if size(data,2) ~= 1
    error('DATA has wrong size!');
end

origin = map(1,1:3);
H = map(2:end,1:3) - repmat(origin, size(map,1) - 1, 1);
dof = rank(H);
if dof < 2
    warning('All landmarks on MAP lie on a same line!');
elseif dof < 3
    warning('All landmarks in MAP lie on a same plane!');
end
b = (H(:,1).^2 + H(:,2).^2 + H(:,3).^2 - data(2:end).^2 + data(1)^2) * 0.5;
x = pinv(H) * b; % Solve H * x = b

pose = [x' + origin, 0, 0, 0];
valid = [true, true, true, false, false, false];
