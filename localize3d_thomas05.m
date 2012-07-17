function [pose, valid] = localize3d_thomas05(data, map)
%   References:
%       [1] F. Thomas and L. Ros, Revisiting Trilateration for Robot Localization, IEEE Trans. on Robotics, Vol. 21, No. 1, 2005
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1391018

if size(data,1) < 3
    error('DATA has less number of observations!');
elseif size(data,1) > 3
    warning('The algorithm uses the first three observations!');
end
if size(data,2) ~= 1
    error('DATA has wrong size!');
end

p1 = map(1,1:3)';
p2 = map(2,1:3)';
p3 = map(3,1:3)';
l1 = data(1);
l2 = data(2);
l3 = data(3);

a = norm(p2 - p1);
b = norm(p3 - p1);
c = norm(p3 - p2);

area = -0.25 * det([[0,   1,   1,   1];
                    [1,   0, a*a, b*b];
                    [1, a*a,   0, c*c];
                    [1, b*b, c*c,   0]]);

volumen = 0.125 * det([[0,     1,     1,     1,     1];
                       [1,     0,   a*a,   b*b, l1*l1];
                       [1,   a*a,     0,   c*c, l2*l2];
                       [1,   b*b,   c*c,     0, l3*l3];
                       [1, l1*l1, l2*l2, l3*l3,     0]]);

if (area == 0) || (volumen < 0)
    warning('The algorithm detects a degernerate case!');

    pose = zeros(1,6);
    valid = [false, false, false, false, false, false];
else
    k1 = (0.25 / area) * det([[0,   1,   1,     1];
                              [1,   0, b*b, l1*l1];
                              [1, a*a, c*c, l2*l2];
                              [1, b*b,   0, l3*l3]]);

    k2 = (-0.25 / area) * det([[0,   1,   1,     1];
                               [1,   0, a*a, l1*l1];
                               [1, a*a,   0, l2*l2];
                               [1, b*b, c*c, l3*l3]]);

    k3 = sqrt(volumen) / area;

    p4a = p1 + k1 * (p2 - p1) + k2 * (p3 - p1) + k3 * cross(p2 - p1, p3 - p1);
    p4b = p1 + k1 * (p2 - p1) + k2 * (p3 - p1) - k3 * cross(p2 - p1, p3 - p1);

    pose = [p4a', 0, 0, 0];
    valid = [true, true, true, false, false, false];
    if ~isequal(p4a, p4b)
        pose = [pose; p4b', 0, 0, 0];
        valid = [valid; valid];
    end

    % Select one among two solutions using the 4th landmark if possible
    if size(data,1) > 3
        [~, select] = min([norm(map(4,1:3) - p4a'), norm(map(4,1:3) - p4b')]);
        pose = pose(select, :);
        valid = valid(select, :);
    end
end;
