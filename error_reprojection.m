function [d] = error_reprojection(p, q)
%ERROR_POSITION  Calculate Euclidean distance between two vectors.
%
%   D = ERROR_POSITION(P, Q)
%       (matrix) P: A vector (1xN matrix, Nx1 matrix)
%       (matrix) Q: A vector (1xN matrix, Nx1 matrix)
%       (scalar) D: Euclidean distance between P and Q
%
%   Note: Two vectors, P and Q, should be same size.
%
%   Example:
%       p = [82; 3; 29];
%       q = [84; 10; 18];
%       d = error_position(p, q)

d = norm(p - q);
