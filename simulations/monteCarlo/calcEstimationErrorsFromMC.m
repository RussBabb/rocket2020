function [ delx ] = calcEstimationErrorsFromMC( xhat_buff, x_buff )
%calcEstimationErrors calculates estimation error
%
% Inputs:
%   xhat = N x m estimated state vector
%   x = N x m true state vector
%
% Outputs
%   delx = estimation error
%
% Example Usage
% [ delx ] = calcEstimationErrors( xhat, x )

% Author: Randy Christensen
% Date: 02-Sep-2018 00:56:41
% Reference:
% Copyright 2018 Utah State University
[n, m] = size(xhat_buff);
delx = zeros(n-1,m);
for i=1:m
    x = x_buff(:,i);
    xhat = xhat_buff(:,i);
    delx(:,i) = calcErrors( xhat, x );
end
end