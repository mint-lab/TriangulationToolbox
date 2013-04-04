function [radian] = tran_deg2rad(degree)
%TRAN_DEG2RAD  Transform the given degree angle into radian angle.
%
%   RADIAN = TRAN_DEG2RAD(DEGREE)
%       (matrix) DEGREE: The given angle [deg]
%       (matrix) RADIAN: The transformed angle [rad]
%
%   Example:
%       t = tran_deg2rad(30)
%       t = tran_deg2rad(60)
%
%   See also tran_rad2deg.

radian = degree * pi / 180;
