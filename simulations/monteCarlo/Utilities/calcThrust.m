%% Define the Thrust function
%  Input: time (sec), stage properties, ambient pressure (kPa)
%  Output: Thrust (N)
function F_thrust = calcThrust(t, stage, P_amb)
A_e = stage.A_e;
P_sea = 101.300;

F_thrust = interpProfile(t);

% Apply altitude adjustment
if F_thrust > 0
    F_thrust = F_thrust + (P_sea - P_amb)*1000*A_e;
end
% Calculate exit velocity
% V_e = M_e*sqrt(gamma*R_g*T_e);  %m/s
% 
% Calculate the thrust
% F_thrust_momentum = mdot_e*V_e;
% F_thrust_pressure = A_e*(P_e - P_inf)*1000;
% lambda = (1 + cos(theta))/2;    %Momentum loss correction for conical nozzle
% F_thrust = F_thrust_momentum*lambda + F_thrust_pressure;
end