function [pose, valid] = localize2d_sayed05_aoa(data, map)
%LOCALIZE2D_SAYED05_AOA  Estimate 2D position and orientation using displacements
%   measured from landmarks (N >= 2)
%
%   [POSE, VALID] = LOCALIZE2D_SAYED05_AOA(DATA, MAP)
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
%   Note: This implementation is extended from Sayed et al. [1], so it can utilize
%       angle-of-arrival (AOA) obtained at a mobile station (MS), not base station (BS).
%
%   Reference:
%       [1] A. H. Sayed et al., Network-based Wireless Location,
%           IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275
%
%   Example:
%       N = 2;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = [10 * rand(N,2), zeros(N,1)]; % Random measurement
%       [pose, valid] = localize2d_sayed05_aoa(data, map)
%
%   See also localize2d_se05

if size(data,1) < 2
    error('DATA has less number of observations!');
end
if size(data,2) ~= 3
    error('DATA has wrong size!');
end

n = size(data,1);
A4 = zeros(2 * n,1);
A4(1:2:end) = -data(:,2);
A4(2:2:end) = +data(:,1);
A = [repmat(eye(2,2), n, 1), reshape(data(:,1:2)', 2 * n, []), A4];
b = reshape(map(:,1:2)', 2 * n, []);
x = pinv(A) * b;
x = x / norm(x(3:4));

pose = [x(1), x(2), 0, 0, 0, atan2(x(4), x(3))];
valid = [true, true, false, false, false, true];
