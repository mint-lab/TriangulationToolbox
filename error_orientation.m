function [rad] = error_orientation(p, q)
%ERROR_ORIENTATION  Calculate angular difference between two sets of Euler angles.
%
%   D = ERROR_ORIENTATION(P, Q)
%       (matrix) P: An Euler angle (1x3 matrix)
%       (matrix) Q: An Euler angle (1x3 matrix)
%       (scalar) D: Angular difference between two sets of Euler angles [rad]
%
%   Note: Two vectors, P and Q, represent orientation in ZYX Euler angle. Each
%       element means rotation angle with repect to x, y, and z axes squentially.
%
%   Note: Angular difference, D, is represented by a scalar value. Since heading
%       of an object is described by x-axis in its local coordinate, error between
%       two orientation comes from angular difference of two x axes.
%
%   Example:
%       rad = error_orientation([0, 0, 0], [0, 0, pi])

v_p = tran_rad2rot(p) * [1; 0; 0];
v_q = tran_rad2rot(q) * [1; 0; 0];
rad = acos(v_p' * v_q);
