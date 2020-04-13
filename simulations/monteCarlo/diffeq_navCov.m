function [ Phat_dot ] = navCov_de( Phat, t, ytilde, xhat, simpar )
%NAVCOV_DE specifies the differential equation that governs the propagation
%of the covariance of the navigation state estimates

%Compute state dynamics matrix
Fhat = calc_F( xhat, ytilde, simpar );

%Compute Bhat
Bhat = calc_Bhat( xhat );

%Compute Shat_w
Shat_w = calc_Shat_w( simpar );

%Compute Phat_dot
Phat_dot = Fhat*Phat + Phat*Fhat' + Bhat*Shat_w*Bhat';
end
