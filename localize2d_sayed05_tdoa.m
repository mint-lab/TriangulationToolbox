function [pose, valid] = localize2d_sayed05_tdoa(data, map)

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
