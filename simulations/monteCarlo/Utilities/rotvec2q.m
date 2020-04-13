function [ qba ] = rotvec2q( phi )
%rotvec2q computes the quaternion from the A frame to the B frame in terms
%of a rotation vector specified by an angle and an axis
%
% Inputs:
%   phi = rotation vector (radians)
%
% Outputs
%   qba = attitude quaternion from the B frame to the A frame
%
% Example Usage
% [ qba ] = rotvec2q( u_phi_A, phi )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 11:10:54
% Reference: Strapdown Navigation Second Edition, Paul Savage, section
% 3.2.4.4
% Copyright 2018 Utah State University
phi_2 = norm(phi)/2;
sin_phi_2_phi = 0.5 * (1 - phi_2^2/factorial(3) + phi_2^4/factorial(5) ...
    - phi_2^6/factorial(7) + phi_2^8/factorial(9));
a = cos(phi_2);
b = phi(1)*sin_phi_2_phi;
c = phi(2)*sin_phi_2_phi;
d = phi(3)*sin_phi_2_phi;
qba = [a, b, c, d]';
qba = qba/norm(qba);
end
