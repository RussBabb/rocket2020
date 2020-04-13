function [T_amb, P_amb, rho, c, mu] = getExpAtmProps(h, F107, Ap)
%getExpAtmProps Returns atmospheric properties based on an exponential
%atmosphere model.
%   Input: Altitude (km), F107, Planetary index
%   Output: Ambient Temperature (K), Ambient Pressure (kPa), Atmospheric
%   Density (kg/m^3), Speed of sound (m/s), Atmospheric Viscosity

%Kinetic temperature
T_amb = 900 + 2.5*(F107-70)+1.5*Ap;

%Molecular mass of atmosphere
m = 27 -0.012*(h-200);

%Scale height
Hscale = T_amb/m;

rho = 6.0e-10*exp( -(h-175)/Hscale);

P_amb = rho*Rg*T;

c = sqrt(gamma*Rg*T);
end

