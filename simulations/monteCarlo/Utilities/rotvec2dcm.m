function [ cba ] = rotvec2dcm( phi )
%rotvec2dcm generates a direction cosine matrix from a rotation vector that
%transforms a vector from the B frame to the A frame
%
% Inputs:
%   phi = rotation vector (radians)
%
% Outputs
%   cba = DCM from the B frame to the A frame
%
% Example Usage
% [ cba ] = rotvec2dcm( phi )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 10:51:11
% Reference: Strapdown Navigation Second Edition, Paul Savage, section
% 3.2.2.1
% Copyright 2018 Utah State University
phi_mag = norm(phi);
sin_phi_phi = 1 - phi_mag^2/factorial(3) + phi_mag^4/factorial(5)...
    - phi_mag^6/factorial(7);
one_cos_phi_phi_2 = 1/factorial(2) - phi_mag^2/factorial(4) ...
    + phi_mag^4/factorial(6) - phi_mag^6/factorial(8);
cba = eye(3) + sin_phi_phi*vx(phi) + one_cos_phi_phi_2*vx(phi)*vx(phi);
end
