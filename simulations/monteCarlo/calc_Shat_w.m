function [ Shat_w ] = calc_Shat_w( simpar )
%calc_Shat_w calculates the noise power spectral density matrix
%
% Inputs:
%   simpar = simulation parameters
%
% Outputs
%   Shat_w =  noise power spectral density matrix
%
% Example Usage
% [ Shat_w ] = calc_Shat_w( simpar )

% Author: Randy Christensen
% Date: 31-Aug-2018 10:33:25
% Reference: 
% Copyright 2018 Utah State University

%Unpack the inputs
S_a = simpar.nav.params.Q_a;
% S_w = simpar.nav.params.Q_w;
S_w = simpar.nav.params.arw^2;

sig_accel_ss = simpar.nav.params.sig_accel_ss;
sig_gyro_ss = simpar.nav.params.sig_gyro_ss;
sig_alt_ss = simpar.nav.params.sig_alt_ss;
sig_air_ss = simpar.nav.params.sig_air_ss;

tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_alt = simpar.general.tau_alt;
tau_air = simpar.general.tau_air;

S_accel = 2*sig_accel_ss^2/tau_accel;
S_gyro = 2*sig_gyro_ss^2/tau_gyro;
S_alt = 2*sig_alt_ss^2/tau_alt;
S_air = 2*sig_air_ss^2/tau_air;


%Compute Shat_w
Shat_w = diag([S_a, S_a, S_a,...
    S_w, S_w, S_w, ...
    S_accel, S_accel, S_accel, ...
    S_gyro, S_gyro, S_gyro]);
end
