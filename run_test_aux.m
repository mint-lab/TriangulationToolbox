close all;
clear all;

disp('== Unit Test for Triangulation Toolbox ==');

% landmark maps
map = [ ...
    % x,  y,  z, r_x, r_y, r_z
      0,  0,  0,   0,  0,  0;                  ...
      5,  0,  5,   0,  0,  tran_deg2rad( +90); ...
      5,  5,  0,   0,  0,  tran_deg2rad(-180); ...
      0,  5,  5,   0,  0,  tran_deg2rad( -90)  ...
];

if 0
% test_is_equal
disp('==== test_is_true ====');
test_is_true(82 < 84);
test_is_true([10, 18] == [10, 18]);
test_is_true(~isequal([3, 2, 9], [1, 0, 1, 8]));

% test_is_near
disp('==== test_is_near ====');
test_is_near(82, 84, 5);
test_is_near(4.17, 4.17 + eps);

% calculate_distance
disp('==== calculate_distance ====');
test_is_true(calculate_distance([1, 2], [3, 4]) == 2 * sqrt(2));

% trim_rad
disp('==== trim_rad ====');
test_is_true(trim_rad(pi) == -pi);
test_is_true(trim_rad(-pi) == -pi);
test_is_true(trim_rad(2 * pi) == 0);
test_is_true(trim_rad(-2 * pi) == 0);

in = (-4 * pi):(pi/100):(+4 * pi);
out = trim_rad(in);
figure();
hold on;
    plot(in, out);
    title('trim\_rad: Correctness on [-4 * pi, +4 * pi]');
    xlabel('in [rad]');
    ylabel('out [rad]');
    axis equal;
    box on;
    grid on;
hold off;

% tran_deg2rad
disp('==== tran_deg2rad ====');
test_is_near(tran_deg2rad(30), pi / 6);
test_is_near(tran_deg2rad(60), pi / 3);

% tran_rad2deg
disp('==== tran_rad2deg ====');
test_is_near(tran_rad2deg(pi / 6), 30);
test_is_near(tran_rad2deg(pi / 3), 60);

% tran_rad2rot
disp('==== tran_rad2rot ====');
test_is_near(tran_rad2rot([0, 0, pi / 2]), [0, -1, 0; 1, 0, 0; 0, 0, 1]);
r = tran_rad2rot([pi / 6, pi / 6, pi / 6]);
test_is_near(r * r', eye(3,3));

% observe_distance
disp('==== observe_distance ====');
[obsData, obsMap] = observe_distance(map, [0, 0, 0, 0, 0, 0]);
test_is_true(obsMap == map);
test_is_near(obsData(1), norm([0, 0, 0]));
test_is_near(obsData(2), norm([5, 0, 5]));
test_is_near(obsData(3), norm([5, 5, 0]));
test_is_near(obsData(4), norm([0, 5, 5]));

test_is_true(isempty(observe_distance([], [0, 0, 0, 0, 0, 0])));                    % In case of no landmark
test_is_true(isempty(observe_distance([1, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], 0))); % In case of zero visibility

data = observe_distance(repmat([10, 0, 0, 0, 0, 0], 10000, 1), [0, 0, 0, 0, 0, 0], 0.5, 1);
test_is_near(size(data,1), 10000 * 0.5, 100);
figure();
hold on;
    hist(data, 50);
    title('observe\_distance: Error Distribution');
    xlabel('Distance Observation');
    ylabel('Frequency');
    box on;
hold off;
end

% observe_displacement
disp('==== observe_displacement ====');
[obsData, obsMap] = observe_displacement(map, [5, 5, 5, 0, 0, pi / 2]);
test_is_near(obsData(1,:), [-5, +5, -5]);
test_is_near(obsData(2,:), [-5,  0,  0]);
test_is_near(obsData(3,:), [ 0,  0, -5]);
test_is_near(obsData(4,:), [ 0,  5,  0]);

% observe_bearing

% observe_pose

