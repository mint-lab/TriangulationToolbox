function [pose, valid] = localize2d_sayed05(data, map)
%   References:
%       [1] A. H. Sayed et al., Network-based Wireless Location, IEEE Signal Processing Magazine, Vol. 24, No. 4, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1458275

if size(data,1) < 3
    error('TT: localize2d_sayed05: DATA has less number of observation!');
end
if size(data,2) ~= 1
    error('TT: localize2d_sayed05: DATA has wrong size!');
end

origin = map(1,1:2);
H = map(2:end,1:2) - repmat(origin, size(map,1) - 1, 1);
b = (H(:,1).^2 + H(:,2).^2 - data(2:end).^2 + data(1)^2) * 0.5;
x = pinv(H) * b; % Solve H * x = b

pose = [x' + origin, 0, 0, 0, 0];
valid = [true, true, false, false, false, false];
