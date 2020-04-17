function g = calcGrav(h, lambda)
%calcGrav Calculates the graviatational acceleration magnitude  for a given
%altitude
%   h = altutude above mean sea level (m)
%   lambda = launch latitude
MU = 3.98060044e5;      %km^3/s^2
R_EQ = 6378.13649;      %km
omega = 7.2929115e-5;
e = 0.08181980;

R_EARTH = R_EQ/sqrt(1 + e^2/(1 - e^2)*sin(lambda)^2);
g = (MU/(R_EARTH + h/1000)^2 - omega^2*(R_EARTH + h/1000))*1000;
end
