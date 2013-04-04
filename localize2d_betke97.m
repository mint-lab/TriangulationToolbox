function [pose, valid] = localize2d_betke97(data, map)
%LOCALIZE2D_BETKE97  Estimate 2D position and orientation using bearing angles
%   measured from landmarks (N >= 3)
%
%   [POSE, VALID] = LOCALIZE2D_BETKE97(DATA, MAP)
%       (matrix) DATA : The measured bearing angles from landmarks (Nx2 matrix)
%       (matrix) MAP  : The corresponding landmark map (Nx6 matrix)
%       (matrix) POSE : The estimated pose (1x6 matrix)
%       (matrix) VALID: A flag to represent validity of the estimated pose (1x6 matrix)
%
%   Note: Please refer to the command, OBSERVE_BEARING, for the convention of DATA,
%       MAP, and POSE.
%
%   Note: A flag for validity, VALID, is 1x6 matrix whose elements correspond to each
%       element of POSE. Since this algorithm estimates 2D position and orientation,
%       the expected VALID is [true, true, false, false, false, true].
%
%   Reference:
%       [1] M. Betke and L. Gurvits, Mobile Robot Localization using Landmarks,
%           IEEE Transactions on Robotics and Automation, Vol. 13, No. 2, 1997
%           URL: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=563647
%
%   Example:
%       N = 3;
%       map = [10 * rand(N,2), zeros(N,4)]; % Random 2D landmark map
%       data = [2 * pi * rand(N,1) - pi, zeros(N,1)]; % Random measurement
%       [pose, valid] = localize2d_betke97(data, map)
%
%   See also localize2d_shimshoni02_algebraic, localize2d_shimshoni02_improved.

if size(data,1) < 3
    error('DATA has less number of observations!');
end
if size(data,2) ~= 2
    error('DATA has wrong size!');
end

n = size(data,1);
z0 = map(n,1:2);
z = map(1:n-1,1:2);
tau0 = data(n,1);
psi = data(1:n-1,1) - tau0;
n = n - 1;

% 1. Initialize
v = z - repmat(z0, n, 1);                   % v_i = z_0 - z_i
c = [+v(:,1) ./ (v(:,1).^2 + v(:,2).^2), -v(:,2) ./ (v(:,1).^2 + v(:,2).^2)];
                                            % c_i = 1 / v_i
sum_of_c = sum(c);
b = [c(:,1) .* cos(psi) - c(:,2) .* sin(psi), c(:,1) .* sin(psi) + c(:,2) .* cos(psi)];
                                            % b_i = c_i * exp(j * psi_i)

% 2. Calculate s = 1 / 2 * A' * c
s = n * sum(b .* c, 2) - sum(b .* repmat(sum_of_c, n, 1), 2);

% 3. Calculate r = (A'*A)^-1 * A'
r = RatiosCalculator(b, s);

% 4. Calculate robot position, p
d = repmat(r, 1, 2) .* b - c;
zr0 = [+d(:,1) ./ (d(:,1).^2 + d(:,2).^2), -d(:,2) ./ (d(:,1).^2 + d(:,2).^2)];
                                            % zr0_i = 1 / (r_i * b_i - c_i)
p = repmat(z0, n, 1) - zr0;                 % p_i = z_0 - zr0_i
p = sum(p) / n;

% 5. Calculate robot orientation, theta
zr0 = z0 - p;
theta = trim_rad(atan2(zr0(2), zr0(1)) - tau0);

pose = [p, 0, 0, 0, theta];
valid = [true, true, false, false, false, true];



function [r] = RatiosCalculator(b, s)
n = size(s, 1);

% 1. Compute diagonal matrix (n * D)^-1
D = diag(b(:,1).^2 + b(:,2).^2);
Dinv = (n * D)^-1;
Dinv_b = Dinv * b;

% 2. Compute K^-1 * s
Kinv_s = Dinv*s      + Dinv_b(:,1)*Dinv_b(:,1)'*s      / (1 - Dinv_b(:,1)'*b(:,1));

% 3. Compute K^-1 * b_y
Kinv_b = Dinv_b(:,2) + Dinv_b(:,1)*Dinv_b(:,1)'*b(:,2) / (1 - Dinv_b(:,1)'*b(:,1));

% 4. Compute ratio vector r
r = Kinv_s + (Kinv_b * (Kinv_b'*s)) / (1 - Kinv_b'*b(:,2));
r(r < 0) = 0;
