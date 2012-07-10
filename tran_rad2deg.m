function [degree] = tran_rad2deg(radian)
%TRAN_RAD2DEG  Transform the given radian angle into degree angle.
%
%   DEGREE = TRAN_RAD2DEG(RADIAN)
%       (matrix) RADIAN: The given angle [rad]
%       (matrix) DEGREE: The transformed angle [deg]
%
%   Examples:
%       t = tran_rad2deg(pi / 6)
%       t = tran_rad2deg(pi / 3)
%
%   See also tran_deg2rad.

degree = radian * 180 / pi;
