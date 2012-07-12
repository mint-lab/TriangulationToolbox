function [R] = tran_rad2rot(radian)
%TRAN_RAD2ROT  Compose a 3D rotation matrix from the given orientation.
%
%   [R] = TRAN_RAD2ROT(RADIAN)
%       (matrix) RADIAN: The given orientation [rad] (1x3 matrix)
%       (matrix) R     : 3D rotation matrix (3x3 matrix)
%
%   Note: Orientation, RADIAN, is represented by 1x3 matrix whose elements are
%       rotation angle with respect to x, y, and z axes, respectively.
%
%   References:
%       [1] S.M. LaValle, Planning Algorithm, Cambridge, 2006,
%           URL: http://planning.cs.uiuc.edu/node102.html
%           URL: http://planning.cs.uiuc.edu/node103.html
%
%   Examples:
%       R = tran_rad2rot([pi/3, pi/4, pi/6])
%
%   See also tran_rot2rad.

Rz = @(x)([cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1]);
Ry = @(x)([cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x)]);
Rx = @(x)([1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x)]);

R = Rz(radian(3)) * Ry(radian(2)) * Rx(radian(1));
