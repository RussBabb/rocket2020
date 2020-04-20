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
q_b2f = xhat(7:10);
R_b2f = q2dcm(q_b2f);

I3 = eye(3);

%Compute Bhat
Bhat = zeros(length(xhat)-1, 12);
Bhat(4:6,1:3) = I3;     %Accel
Bhat(7:9,4:6) = R_b2f;  %Attitude
Bhat(10:12,7:9) = I3;   %Accel bias
Bhat(13:15,10:12) = I3; %Gyro bias
% Bhat(16,13) = 1;        %Altimeter bias
% Bhat(17,14) = 1;        %Airspeed bias
end
