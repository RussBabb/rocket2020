function [ v ] = vx_inv( vcross )
%vx_inv returns the vector from a cross-product matrix
%
% Inputs:
%   vcross = cross product matrix
%
% Outputs
%   v = vector
%
% Example Usage
% [ v ] = vx_inv( vcross )

% Author: Randy Christensen
% Date: 03-Jun-2019 16:54:19
% Reference:
% Copyright 2019 Utah State University

v = [vcross(3,2),vcross(1,3),vcross(2,1)]';

end
