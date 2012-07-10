function [out] = tran_rad2deg(in)
%TRAN_RAD2DEG  Transform the given radian angle into degree angle
%
%   OUT = TRAN_RAD2DEG(IN)
%      (matrix) IN  : The given angle [rad]
%      (matrix) OUT : The transformed angle [deg]
%
%   Examples:
%       t = tran_rad2deg(pi / 6)
%       t = tran_rad2deg(pi / 3)

out = in * 180 / pi;