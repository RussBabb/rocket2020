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

% sig_accel_ss = simpar.nav.params.sig_accel_ss;
sig_gyro_ss = simpar.nav.params.sig_gyro_ss;
sig_range_ss = simpar.nav.params.sig_range_ss;
sig_doppler_ss = simpar.nav.params.sig_doppler_ss;
sig_grav_ss = simpar.nav.params.sig_grav_ss;
sig_sc_ss = simpar.nav.params.sig_sc_ss;
sig_tc_ss = simpar.nav.params.sig_tc_ss;

% tau_accel = simpar.general.tau_accel;
tau_gyro = simpar.general.tau_gyro;
tau_range = simpar.general.tau_range;
tau_doppler = simpar.general.tau_doppler;
tau_grav = simpar.general.tau_grav;
tau_sc = simpar.general.tau_sc;
tau_tc = simpar.general.tau_tc;

% S_accel = 2*sig_accel_ss^2/tau_accel;
S_gyro = 2*sig_gyro_ss^2/tau_gyro;
S_range = 2*sig_range_ss^2/tau_range;
S_doppler = 2*sig_doppler_ss^2/tau_doppler;
S_grav = 2*sig_grav_ss^2/tau_grav;
S_sc = 2*sig_sc_ss^2/tau_sc;
S_tc = 2*sig_tc_ss^2/tau_tc;

%Compute Shat_w
Shat_w = diag([S_a, S_a, S_a,...
    S_w, S_w, S_w, ...
    S_gyro, S_gyro, S_gyro, ...
    S_range, ...
    S_doppler, ...
    S_grav, S_grav, S_grav, ...
    S_sc, S_sc, S_sc, ...
    S_tc, S_tc, S_tc]);
end
