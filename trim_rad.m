function [out] = trim_rad(in)
%TRIM_RAD  Trim the given radian angle into [-pi, +pi).
%
%   OUT = TRIM_RAD(IN)
%       (matrix) IN : The given angle [rad]
%       (matrix) OUT: The trimmed angle [rad]
%
%   Example:
%       t = trim_rad(2 * pi)
%       t = trim_rad(0 : pi/6 : 4*pi)

out = mod(in, 2 * pi);
out(out >= pi) = out(out >= pi) - 2 * pi;
