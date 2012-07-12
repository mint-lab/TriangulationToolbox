function [pose, valid] = localize2d_shimshoni02_improved(data, map)
%   References:
%       [1] I. Shimshoni, On Mobile Robot Localization from Landmark Bearings, IEEE Trans. on Robotics and Automation, Vol. 18, No. 6, 2002
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1159015

if size(data,1) < 3
    error('TT: localize2d_sayed05: DATA has less number of observation!');
end
if size(data,2) ~= 2
    error('TT: localize2d_sayed05: DATA has wrong size!');
end

% Calculate pose using algebraic method (Section III)
C = mean(map(:,1:2));
delta = map(:,1:2) - repmat(C, size(map,1), 1);
SZ = max(sqrt(delta(:,1).^2 + delta(:,2).^2));
map = (map(:,1:2) - repmat(C, size(map,1), 1)) / SZ; % 2nd improvement, scaled data
A =                                                          ...
[                                                            ...
    map(:,1) .* sin(data(:,1)) - map(:,2) .* cos(data(:,1)), ...
    map(:,2) .* sin(data(:,1)) + map(:,1) .* cos(data(:,1)), ...
    sin(data(:,1)),                                          ...
    -cos(data(:,1))                                          ...
];                                                   % 1st improvement, bounded variance
sigma = std(data(:,1));
d = ones(size(map,1),1);
for i = 1:10                                         % 3rd improvement, iterative
    A = A ./ repmat(d, 1, 4);
    [U,S,V] = svd(A);
    W = V(:,end);
    A_p =                                                        ...
    [                                                            ...
        map(:,2) .* sin(data(:,1)) + map(:,1) .* cos(data(:,1)), ...
        map(:,2) .* cos(data(:,1)) - map(:,1) .* sin(data(:,2)), ...
        cos(data(:,1)),                                          ...
        sin(data(:,1))                                           ...
    ];
    d = A_p * W * sigma;
end
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

P = P * SZ + C';
pose = [P(1), P(2), 0, 0, 0, atan2(W(2), W(1))];
valid = [true, true, false, false, false, true];
