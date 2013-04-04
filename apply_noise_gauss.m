function [out] = apply_noise_gauss(in, stdev)
%APPLY_NOISE_GAUSS  Add unbiased Gaussian noise to the given data
%
%   OUT = APPLY_NOISE_GAUSS(IN, STDEV)
%       (matrix) IN   : The given data (NxM matrix)
%       (matrix) STDEV: Standard deviation of Gaussian noise (1x1 matrix,
%                       1xM matrix, MxM matrix)
%       (matrix) OUT  : The deteriorated data (NxM matrix)
%
%   Note: The given data, IN, are composed of N vectors whose dimension is M.
%
%   Note: 1x1 standard deviation, STDEV, for M-dimensional data is interpreted
%       as isotropic, STDEV * eye(M,M). Similarly, 1xM standard deviation, STDEV,
%       is interpreted as diagonal, diag(STDEV).
%
%   Example:
%       in = zeros(100, 2);
%       out = apply_noise_gauss(in, 3)
%       out = apply_noise_gauss(in, [1, 2])
%       out = apply_noise_gauss(in, [2, 1; 1, 2])

if size(stdev,1) == 1
    stdev = diag(stdev);
end
out = in + randn(size(in)) * stdev';
