function [phi] = calc_PHI(xhat, ytilde, simpar)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
dt = simpar.general.dt;
F = calc_F(xhat, ytilde, simpar);
phi = eye(length(xhat)-1) + F*dt + F*F*dt*dt/2;
end

