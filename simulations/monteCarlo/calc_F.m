function [ Fhat ] = calc_F( xhat, ytilde, simpar )
%get_F computes the dynamics coupling matrix
%
% Inputs:
%   xhat = state vector (mixed)
%   ytilde = continuous measurements (mixed)
%   simpar = simulation parameters
%
% Outputs
%   Fhat = state dynamics matrix
%
% Example Usage
% [ Fhat ] = calc_F( xhat, ytilde, simpar )

% Author: Randy Christensen
% Date: 30-Aug-2018 16:27:40
% Reference: None
% Copyright 2018 Utah State University

% Unpack states
rhat_f = xhat(1:3);
vhat_b = xhat(4:6);
qhat_b2f = xhat(7:10);
bhat_accel = xhat(11:13);
bhat_gyro = xhat(14:16);
bhat_alt = xhat(17);
bhat_air = xhat(18);

%Unpack the inputs
nutilde_b = ytilde(1:3);
omegatilde_b = ytilde(4:6);

tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_alt = simpar.general.tau_alt;
tau_air = simpar.general.tau_air;

R_b2f = q2dcm(qhat_b2f);

I3 = eye(3);
dg = calcdGrav(-rhat_f(3), simpar.init.lat);

%Compute Fhat
[m, ~] = size(xhat);
Fhat = zeros(m-1,m-1);
Fhat(1:3,4:6)       = R_b2f;
Fhat(4:6,1:3)       = R_b2f'*[0, 0, 0; 0, 0, 0; 0, 0, dg];
Fhat(4:6,4:6)       = -vx(omegatilde_b - bhat_accel);
Fhat(4:6,13:15)     = -vx(vhat_b);
Fhat(7:9,13:15)     = R_b2f;
Fhat(10:12,10:12)   = -1/tau_accel*I3;
Fhat(13:15,13:15)   = -1/tau_gyro*I3;
% Fhat(16,16)         = -1/tau_alt;
% Fhat(17,17)         = -1/tau_air;

end
