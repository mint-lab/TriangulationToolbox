function [radian] = tran_rot2rad(R)
%TRAN_ROT2RAD  Decompose the given 3D rotation matrix in angular form.
%
%   [RADIAN] = TRAN_ROT2RAD(R)
%       (matrix) R     : The given 3D rotation matrix (3x3 matrix)
%       (matrix) RADIAN: Orientation [rad] (1x3 matrix)
%
%   Note: Orientation, RADIAN, is represented by 1x3 matrix whose elements are
%       rotation angle with respect to x, y, and z axes, respectively.
%
%   Note: R(1,1) and R(3,3) should not be zero due to singularity.
%
%   References:
%       [1] S.M. LaValle, Planning Algorithm, Cambridge, 2006,
%           URL: http://planning.cs.uiuc.edu/node102.html
%           URL: http://planning.cs.uiuc.edu/node103.html
%
%   Examples:
%       R = tran_rad2rot([pi/6, pi/6, pi/6]);
%       angle = tran_rot2rad(R)
%
%   See also tran_rad2rot.

if det(R) < 0
    R = -R;
end
if R(1,1) == 0 || R(3,3) == 0
    error('TT:tran_rot2rad: An element at (1,1) or (3,3) is zero!');
end

radian = [ atan2( R(3,2), R(3,3)), ...
           atan2(-R(3,1), norm([R(3,2), R(3,3)])), ...
           atan2( R(2,1), R(1,1)) ];
