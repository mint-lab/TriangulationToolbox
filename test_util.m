close all;
clear all;

disp('Unit Test for Triangulation Toolbox');
% IS_EQUAL: disp(A == B)
% IS_NEAR : disp(abs(A - B) < 1e-8)

if 0
% calculate_distance
disp(' * calculate_distance');
disp(calculate_distance([1, 2], [3, 4]) == 2 * sqrt(2));

% trim_rad
disp(' * trim_rad');
disp(trim_rad(pi) == -pi);
disp(trim_rad(-pi) == -pi);
disp(trim_rad(2 * pi) == 0);
disp(trim_rad(-2 * pi) == 0);

in = (-4 * pi):(pi/100):(+4 * pi);
out = trim_rad(in);
figure();
hold on;
    plot(in, out);
    title('trim\_anlge');
    xlabel('in [rad]');
    ylabel('out [rad]');
    axis equal;
    box on;
    grid on;
hold off;

% tran_deg2rad
disp(' * tran_deg2rad');
disp(abs(tran_deg2rad(30) - pi / 6) < 1e-8);
disp(abs(tran_deg2rad(60) - pi / 3) < 1e-8);

% tran_rad2deg
disp(' * tran_rad2deg');
disp(abs(tran_rad2deg(pi / 6) - 30) < 1e-8);
disp(abs(tran_rad2deg(pi / 3) - 60) < 1e-8);
end

% observe_distance
disp(' * observe_distance');
map = [
    % x,  y,  z,  r_x, r_y, r_z
      0,  0,  0,    0,  0,  0;
     10,  0,  0,    0,  0,  tran_deg2rad(90);
     10, 10,  0,    0,  0,  tran_deg2rad(-180);
      0, 10,  0,    0,  0,  tran_deg2rad(-90) ];
%obs = observe_distance();

% observe_bearing

% observe_displacement

% observe_pose

