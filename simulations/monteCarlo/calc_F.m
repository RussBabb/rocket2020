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
rhat_i = xhat(1:3);
vhat_i = xhat(4:6);
qhat_b2i = xhat(7:10);
rhat_beacon = xhat(11:13);
% b_accel = x(11:13);
bhat_gyro = xhat(14:16);
bhat_range = xhat(17);
bhat_doppler = xhat(18);
bhat_grav = xhat(19:21);
that_sc = xhat(22:24);
that_tc = xhat(25:27);
rhat_features = xhat(28:end);

%Unpack the inputs
wtilde_b = ytilde(1:3);
% Rhat_b2i = q2dcm(q_b2i);

% tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_range = simpar.general.tau_range;
tau_doppler = simpar.general.tau_doppler;
tau_grav = simpar.general.tau_grav;
tau_sc = simpar.general.tau_sc;
tau_tc = simpar.general.tau_tc;

MU = simpar.general.MU;
J2 = simpar.general.J2;
R_EQ = simpar.general.R_EQ;
% T_I2LF = calc_I2LCF(t, simpar);

I3 = eye(3);
% ir = rhat_i/norm(rhat_i);

%Compute Fhat
[m, ~] = size(xhat);
Fhat = zeros(m-1,m-1);
Fhat(1:3,4:6) = I3;     %drdot/dxhat
Fhat(4:6,1:3) = calc_dg(rhat_i, [0, 0, 1]', R_EQ, J2, MU); %-MU/norm(rhat_i)^3*(I3 - 3*(ir*ir')); %
Fhat(4:6,18:20) = -I3;
Fhat(7:9,13:15) = q2dcm(qhat_b2i);
Fhat(13:15,13:15) = -1/tau_gyro*I3;
Fhat(16,16) = -1/tau_range;
Fhat(17,17) = -1/tau_doppler;
Fhat(18:20,18:20) = -1/tau_grav*I3;
Fhat(21:23,21:23) = -1/tau_sc*I3;
Fhat(24:26,24:26) = -1/tau_tc*I3;

end
