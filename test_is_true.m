function [result] = test_is_true(value, verbose)
%TEST_IS_TRUE  Check whether the given value is true or not
%
%   RESULT = TEST_IS_TRUE(VALUE, VERBOSE)
%       (matrix) VALUE  : The given value
%       (scalar) VERBOSE: A flag to print its result (default: true)
%       (scalar) RESULT : The test result
%
%   Examples:
%       t = test_is_true(82 < 84, false)
%       t = test_is_true([10, 18] == [10, 18])
%       t = test_is_true(isequal([1, 2; 3, 4], [5, 6]))
%
%   See alo test_is_near.

if nargin < 2
    verbose = true;
end

if isequal(value, ones(size(value)))
    result = true;
else
    result = false;
end

% Print 'result'
TEXT = {'true', 'false'};
if verbose
    disp(TEXT(2 - result));
end
