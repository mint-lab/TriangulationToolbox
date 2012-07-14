function [pose, valid] = localize2d_shimshoni02_algebraic(data, map)
%   References:
%       [1] I. Shimshoni, On Mobile Robot Localization from Landmark Bearings, IEEE Trans. on Robotics and Automation, Vol. 18, No. 6, 2002
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1159015

if size(data,1) < 3
    error('DATA has less number of observations!');
end
if size(data,2) ~= 2
    error('DATA has wrong size!');
end

% Calculate pose using algebraic method (Section II.C)
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
