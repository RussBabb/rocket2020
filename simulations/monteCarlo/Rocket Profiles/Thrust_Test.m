
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This program is only meant to act as a test of the Interpolate_Thrust
% function to verify it is calculating thrust values that match the
% recorded values provided by the rocket motor manufacturer.
%
% It will call the Interpolate_Thrust subfunction and run it over a
% SELECTED range of time for a SELECTED time step and then plot the
% resulting thrust values.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all

Motor_Thrust = 2150; % (N) SELECT MOTOR THRUST
                     % "1160" - Cesaroni M1160
                     % "1170" - AeroTech L1170
                     % "1401" - Cesaroni M1401
                     % "1420" - AeroTech L1420
                     % "1600" - AeroTech M1600
                     % "2150" - Cesaroni M2150

Burn_Time=6; % (s) SELECT BURN TIME
dt=0.002;    % (s) SELECT TIME STEP

i=1;
t(i)=0;
Thrust(i)=0;

while t(i)<Burn_Time
    t(i+1)=i*dt;
    Thrust(i+1) = interpProfile(t(i+1));
    i=i+1;
end

plot(t,Thrust)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%