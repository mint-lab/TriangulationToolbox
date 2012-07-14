function [pose, valid] = localize2d_se05(data, map)
%   References:
%       [1] S. Se et al., Vision-Based Global Localization and Mapping for Mobile Robots, IEEE Trans. on Robotics, Vol. 21, No. 3, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1435480

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
    % Why '-atan2(...)' for orientation?
    % In this toolbox, CCW is positive rotation, but the paper[1] uses it as negative.
valid = [true, true, false, false, false, true];
