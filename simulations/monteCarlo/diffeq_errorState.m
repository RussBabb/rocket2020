function [ delx_dot ] = errorState_de( delx, t, ytilde, xhat, simpar )
%NAVCOV_DE specifies the differential equation that governs the propagation
%of the covariance of the navigation state estimates

%Compute state dynamics matrix
Fhat = calc_F( xhat, ytilde, simpar );

%Compute Phat_dot
delx_dot = Fhat*delx;
end
