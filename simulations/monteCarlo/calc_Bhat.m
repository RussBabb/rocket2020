function [ Bhat ] = calc_Bhat( xhat )
%calc_Bhat Calculates the process noise dynamic coupling matrix
%
% Inputs:
%   xhat = state vector (mixed)s)
%
% Outputs
%   Bhat = process noise dynamic coupling matrix (units)
%
% Example Usage
% [ Bhat ] = calc_Bhat( xhat )

% Author: Randy Christensen
% Date: 31-Aug-2018 10:26:13
% Reference: None
% Copyright 2018 Utah State University

%Unpack the inputs
q_b2i = xhat(7:10);
T_b2i = q2dcm(q_b2i);

I3 = eye(3);

%Compute Bhat
Bhat = zeros(length(xhat)-1, 20);
Bhat(4:6,1:3) = I3;     %Accel
Bhat(7:9,4:6) = T_b2i;  %Attitude
Bhat(13:15,7:9) = I3;   %Gyro bias
Bhat(16,10) = 1;        %Range bias
Bhat(17,11) = 1;        %Doppler bias
Bhat(18:20,12:14) = I3; %Gravity bias
Bhat(21:23,15:17) = I3; %Star camera bias
Bhat(24:26,18:20) = I3; %Terrain camera bias
end
