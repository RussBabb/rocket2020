function [rpy] = dcm2xyz(cba)
%dcm2xyz computes a XYZ euler angle sequence beginning at the A frame and
%undergoing a roll rotation about the X axis, followed by a pitch rotation
%about the once-rotated Y axis, followed by a yaw rotation about the
%twice-rotated Z axis.
%
% Inputs:
%   cba = DCM from the B frame to the A frame (unitless)
%
% Outputs
%   rpy = XYZ euler angle sequenc (radians)
%
% Example Usage
% [rpy] = dcm2xyz(cba)
%

% Author: Randy Christensen
% Date: 06-Feb-2019 10:57:51
% Reference: none
% Copyright 2019 Utah State University



%Extract components of 
cab = cba';
theta = asin(-cab(1,3));
phi = atan2(cab(2,3),cab(3,3));
psi = atan2(cab(1,2),cab(1,1));
rpy = [phi, theta, psi]';
end

