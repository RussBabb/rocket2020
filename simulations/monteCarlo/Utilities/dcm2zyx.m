function [ ypr ] = dcm2zyx( cba )
%dcm2ypr computes a ZYX euler angle sequence beginning at the A frame and
%undergoing a yaw rotation about the Z axis, followed by a pitch rotation
%about the once-rotated Y axis, followed by a roll rotation about the
%twice-rotated X axis.
%
% Inputs:
%   cba = DCM from the B frame to the A frame (unitless)
%
% Outputs
%   ypr = ZYX euler angle sequenc (radians)
%
% Example Usage
% [ ypr ] = dcm2ypr( cba )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 10:57:51
% Reference: Strapdown Navigation Second Edition, Paul Savage, Equations
% 3.2.3.2-1 to 3.2.3.2-4.
% Copyright 2018 Utah State University

%Extract components of 
cba(3,1) = cba(3,1);
cba(3,2) = cba(3,2);
cba(3,3) = cba(3,3);
cba(2,1) = cba(2,1);
cba(1,1) = cba(1,1);

theta = atan2(-cba(3,1), sqrt(cba(3,2)^2 + cba(3,3)^2));
phi = atan2(cba(3,2),cba(3,3));
psi = atan2(cba(2,1),cba(1,1));
if abs(cba(3,2)) >= 0.999
    phi = NaN;
    psi = NaN;
end
ypr = [psi, theta, phi]';
end
