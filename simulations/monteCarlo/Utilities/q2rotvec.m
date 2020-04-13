function [ phi ] = q2rotvec( qba )
%q2rotvec Summary of the function goes here
%   Detailed explanation goes here
%
% Inputs:
%   qba = attitude quaternion from the B frame to the A frame
%
% Outputs
%   u_phi_A = axis of rotation (unitless)
%   phi = angle (rad)
%
% Example Usage
% [ u_phi_A, phi ] = q2rotvec( qba )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 11:33:51
% Reference: Strapdown Navigation Second Edition, Paul Savage, section
% 3.2.4.5
% Copyright 2018 Utah State University

%Unpackaged the quaternion
a = qba(1);
b = qba(2);
c = qba(3);
d = qba(4);

if a<=0
    a = -a;
    b = -b;
    c = -c;
    d = -d;
end

phi_2 = atan2(sqrt(b^2 + c^2 + d^2),a);
f = 0.5 * (1 - phi_2^2/factorial(3) + phi_2^4/factorial(5) ...
    - phi_2^6/factorial(7));
phi_x = b/f;
phi_y = c/f;
phi_z = d/f;
phi = [phi_x, phi_y, phi_z]';
end