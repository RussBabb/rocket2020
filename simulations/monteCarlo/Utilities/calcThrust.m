%% Define the Thrust function
%  Input: Massflow in, Massflow out, Exit pressure, Ambient pressure, Exit
%  area, Exit mach number, Incoming velocity, gamma, Rg, Exit temp
%  Output: Thrust (N), Specific impulse (seconds)
function F_thrust = calcThrust(t, stage, P_amb)
motor = stage.motor;
A_e = stage.A_e;
P_sea = 101.300;

if isa(motor, 'double')
    F_thrust = Interpolate_Thrust(t, motor);
    
elseif motor == "hybrid"
    F_thrust = stage.F_thrust;
end

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