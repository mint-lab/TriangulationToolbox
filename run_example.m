close all;
clear all;

disp('== A Simple Example for Triangulation Toolbox ==');

% The given configuration: a landmark map and true pose
trueMap =                                      ...
[                                              ...
    % x,  y,  z, r_x, r_y, r_z                 ...
      0,  0,  0,   0,  0,  0;                  ...
      4,  0,  0,   0,  0,  0;                  ...
      8,  0,  0,   0,  0,  tran_deg2rad( +90); ...
      8,  4,  0,   0,  0,  tran_deg2rad( +90); ...
      8,  8,  0,   0,  0,  tran_deg2rad(-180); ...
      4,  8,  0,   0,  0,  tran_deg2rad(-180); ...
      0,  8,  0,   0,  0,  tran_deg2rad( -90); ...
      0,  4,  0,   0,  0,  tran_deg2rad( -90); ...
];
truePose = [3, 2, 0, 0, 0, pi / 9];
paramVisibility = 0.8;
paramNoise = 0.05;

% Generate observation with Gaussian noise
[obsData, obsMap] = observe_bearing(trueMap, truePose, paramVisibility);
obsData = apply_noise_gauss(obsData, paramNoise);

% Estimate pose from observation
[pose, valid] = localize2d_shimshoni02_algebraic(obsData, obsMap);
errPos = error_position(truePose(1:3), pose(1:3));
errOri = error_orientation(truePose(4:6), pose(4:6));

% Print results
disp('==== The Given Configuration ====');
fprintf('- The true position   : (%.3f, %.3f, %.3f)\n', truePose(1:3));
fprintf('- The true orientation: (%.3f, %.3f, %.3f) [rad] /', truePose(4:6));
fprintf(' (%.1f, %.1f, %.1f) [deg]\n\n', tran_rad2deg(truePose(4:6)));
disp('==== The Estimated Pose ====');
fprintf('- The estimated position   : (%.3f, %.3f, %.3f)\n', pose(1:3));
fprintf('- The estimated orientation: (%.3f, %.3f, %.3f) [rad] /', pose(4:6));
fprintf(' (%.1f, %.1f, %.1f) [deg]\n', tran_rad2deg(pose(4:6)));
fprintf('- The position error   : %.3f\n', errPos);
fprintf('- The orientation error: %.3f [rad] /', errOri);
fprintf(' %.1f [deg]\n', tran_rad2deg(errOri));

% Visualize results
figure();
hold on;
    plot3(trueMap(:,1), trueMap(:,2), trueMap(:,3), 'b.', 'MarkerSize', 20);  % All landmarks
    plot3(obsMap(:,1), obsMap(:,2), obsMap(:,3), 'ro', 'LineWidth', 2);       % The observed landmarks
    plot3(truePose(1), truePose(2), truePose(3), 'b.', 'MarkerSize', 30);     % The true postion
    plot3(pose(1), pose(2), pose(3), 'go', 'LineWidth', 3, 'MarkerSize', 10); % The estimated position
    R = tran_rad2rot(truePose(4:6));
    arrow = [truePose(1:3); truePose(1:3) + R(:,1)'];
    plot3(arrow(:,1), arrow(:,2), arrow(:,3), 'b-', 'LineWidth', 2);          % The true orientation
    R = tran_rad2rot(pose(4:6));
    arrow = [pose(1:3); pose(1:3) + R(:,1)'];
    plot3(arrow(:,1), arrow(:,2), arrow(:,3), 'g-', 'LineWidth', 2);          % The estimated orientation
    title('Visualization for Triangulation Toolbox');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    legend({'All Landmarks', 'Obs. Landmarks', 'True Position', 'Est. Position'});
    grid on;
    axis equal;
    box on;
hold off;
