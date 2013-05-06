function [obsData, obsMap] = observe_displacement(map, pose, visibleRate)
%OBSERVE_DISPLACEMENT  Measure displacement from the given pose to landmarks.
%
%   [OBS_DATA, OBS_MAP] = OBSERVE_DISPLACEMENT(MAP, POSE, VISIBLE_RATE)
%       (matrix) MAP         : A landmark map (Nx6 matrix)
%       (matrix) POSE        : Pose of the target object (1x6 matrix)
%       (scalar) VISIBLE_RATE: Visible probability of landmarks (default: 1)
%       (matrix) OBS_DATA    : The measured displacement from POSE to landmarks (Mx3 matrix)
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
%       If there is no visible landmark, OBS_DATA and OBS_MAP will be an empty matrix.
%       Please use the command, ISEMPTY, to identify an empty matrix.
%
%   Note: The measured displacement, OBS_DATA, is represented by Mx3 matrix whose i-th row
%       is relative position of i-th landmark in OBS_MAP with respect to the given pose, POSE.
%
%   Example:
%       map  = [ 0, 0, 5, 0, 0, 0; ...
%                5, 0, 5, 0, 0, 0; ...
%                5, 5, 5, 0, 0, 0 ];
%                0, 5, 5, 0, 0, 0 ];
%       pose = [ 3, 2, 9, 0, 0, pi / 2 ];
%       [obsData, obsMap] = observe_displacement(map, pose)
%
%   See also observe_distance, observe_distance_relative, observe_bearing, observe_pose.

if nargin < 3
    visibleRate = 1;
end

isVisible = rand(size(map,1), 1) < visibleRate; % Select visible landmarks
obsMap = map(isVisible,:);
obsNum = size(obsMap,1);
obsDim = 3;

obsData = zeros(obsNum,obsDim);
if obsNum > 0
    delta = obsMap(:,1:3) - repmat(pose(1:3), obsNum, 1);
    obsData = delta * tran_rad2rot(pose(4:6)); % Calculate displacement
                                               % a = R' * b --> a' = b' * R
end
