function [pose, valid] = localize2d_shimshoni02_algebraic(data, map)
%LOCALIZE2D_SHIMSHONI02_ALGEBRAIC  Estimate 2D position and orientation using bearing
%   angles measured from landmarks (N >= 3)
%
%   [POSE, VALID] = LOCALIZE2D_SHIMSHONI02_ALGEBRAIC(DATA, MAP)
%       (matrix) DATA : The measured bearing angles from landmarks (Nx2 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_BEARING, for the convention of DATA,
%       MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 2D position and orientation,
%       the expected VALID is [true, true, false, false, false, true].
%
%   Note: This implementation is based on Shimshoni's basic algebraic approach [1]
%       without the proposed improvement.
%
%   Reference:
%       [1] I. Shimshoni, On Mobile Robot Localization from Landmark Bearings,
%           IEEE Transactions on Robotics and Automation, Vol. 18, No. 6, 2002
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1159015
%
%   Example:
%       N = 3;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = [2 * pi * rand(N,1) - pi, zeros(N,1)]; % Random measurement
%       [pose, valid] = localize2d_shimshoni02_algebraic(data, map)
%
%   See also localize2d_shimshoni02_improved, localize2d_betke97.

if size(data,1) < 3
    error('DATA has less number of observations!');
end
if size(data,2) ~= 2
    error('DATA has wrong size!');
end

% Calculate pose using algebraic method (Section II.C)
if sum(data(:,1) == 0) > 0 % A degenerate case of cotangent
    pose = zeros(1,6);
    valid = false * ones(1,6);
    return;
end
A =                                        ...
[                                          ...
    map(:,1) - map(:,2) .* cot(data(:,1)), ...
    map(:,2) + map(:,1) .* cot(data(:,1)), ...
    ones(size(data,1),1),                  ...
    -cot(data(:,1))                        ...
];
[U,S,V] = svd(A);
W = V(:,end);
W = W ./ norm(W(1:2));  % Scaling
R = [W(1), W(2); -W(2), W(1)];
T = W(3:4);
P = -R' * T;            % T = -R * P

% Select one betweetn two solutions using the first landmark (i = 1)
P_i = map(1,1:2)';
l = norm(P - P_i);
signL = sign(l * [cos(data(1,1)); sin(data(1,1))]);
signR = sign(R * P_i + T);
if ~isequal(signL, signR)
    W = -W;
    R = [W(1), W(2); -W(2), W(1)];
    T = W(3:4);
    P = -R' * T;
end

pose = [P(1), P(2), 0, 0, 0, atan2(W(2), W(1))];
valid = [true, true, false, false, false, true];
