function [d] = calculate_distance(p, q)
%CALCULATE_DISTANCE  Calculate Euclidean distance between two vectors.
%
%   D = CALCULATE_DISTANCE(P, Q)
%       (matrix) P: A vector
%       (matrix) Q: A vector
%       (scalar) D: Euclidean distance between P and Q
%
%   Note: Two vectors, P and Q, should be same size.
%
%   Examples:
%       p = [82; 3; 29];
%       q = [84; 10; 18];
%       d = calculate_distance(p, q)

d = norm(p - q);
