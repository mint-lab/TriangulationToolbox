function [obsData, obsMap] = observe_distance(map, pose, visibleRate, noiseStd)
%OBSERVE_DISTANCE  Measure distance from the given pose to landmarks.
%
%   [OBS_DATA, OBS_MAP] = CALCULATE_DISTANCE(MAP, POSE, VISIBLE_RATE, NOISE_STD)
%       (matrix) MAP         : A landmark map (Nx6 matrix)
%       (matrix) POSE        : Pose of the target object (1x6 matrix)
%       (scalar) VISIBLE_RATE: Visible probability of landmarks (default: 1)
%       (scalar) NOISE_STD   : Standard deviation of measurement (default: 0)
%       (matrix) OBS_DATA    : The measured distance from POSE to landmarks (Mx1 matrix)
%       (matrix) OBS_MAP     : The landmark map of measured landmarks (Mx6 matrix)
%
%   Note: Pose of an object, POSE, is represented by 1x6 vector whose first three
%       columns represent position of the object, (x, y, z), and last three
%       columns represent orientation of the object, (r_x, r_y, r_z) [rad].
%
%   Note: A landmark map, MAP, is Nx6 matrix which contains position and
%       orientation of landmarks in the world coordinate. Its first three columns
%       represents position of landmarks, (x, y, z). Its last three columns represent
%       orientation of landmarks, (r_x, r_y, r_z) [rad].
%
%   Note: The number of output data, M, will be approximately VISIBLE_RATE * N.
%       If there is no visible landmark, OBS_DATA and OBS_MAP will be empty matrice.
%       Please use the command, ISEMPTY, to identify an empty matrix.
%
%   Examples:
%       map  = [ 0, 0, 0, 0, 0, 0; ...
%                5, 5, 5, 0, 0, 0 ];
%       pose = [ 3, 2, 9, 0, 0, 0 ];
%       [obsData, obsMap] = observe_distance(map, pose)
%
%   See also observe_displacement.

if nargin < 3
    visibleRate = 1;
end
if nargin < 4
    noiseStd = 0;
end

isVisible = rand(size(map,1), 1) < visibleRate; % Select visible landmarks
obsMap = map(isVisible,:);
obsNum = size(obsMap,1);

obsData = zeros(obsNum,1);
if obsNum > 0
    delta = obsMap(:,1:3) - repmat(pose(1:3), obsNum, 1);
    obsData = sqrt(delta(:,1).^2 + delta(:,2).^2 + delta(:,3).^2); % Calculate distance
    obsData = obsData + noiseStd * randn(obsNum,1); % Add Gaussian noise
    obsData(obsData < 0) = 0;
end
