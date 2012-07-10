function [out] = tran_deg2rad(in)
%TRAN_DEG2RAD  Transform the given degree angle into radian angle
%
%   OUT = TRAN_DEG2RAD(IN)
%      (matrix) IN  : The given angle [deg]
%      (matrix) OUT : The transformed angle [rad]
%
%   Examples:
%       t = tran_deg2rad(30)
%       t = tran_deg2rad(60)

out = in * pi / 180;
