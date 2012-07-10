function [result] = test_is_near(a, b, tol, verbose)
%TEST_IS_TRUE  Check whether the given two values are near or not
%
%   RESULT = TEST_IS_NEAR(A, B, TOL, VERBOSE)
%       (matrix) A      : The given value
%       (matrix) B      : The given value
%       (scalar) TOL    : Tolerance (default: 1.0e-8)
%       (scalar) VERBOSE: A flag to print its result (default: true)
%       (scalar) RESULT : The test result
%
%   Note: Two values, A and B, should be same size.
%
%   Examples:
%       t = test_is_near(4.17, 4.19, 0.1)
%       t = test_is_near(pi, pi + eps)
%       t = test_is_near([3, 29], [3 + eps, 29 - eps])

if nargin < 3
    tol = 1e-8;
end
if nargin < 4
    verbose = true;
end

if isequal(abs(a - b) < tol, ones(size(a)))
    result = true;
else
    result = false;
end

% Print 'result'
TEXT = {'true', 'false'};
if verbose
    disp(TEXT(2 - result));
end
