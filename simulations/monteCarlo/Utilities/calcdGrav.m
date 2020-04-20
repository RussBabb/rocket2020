function dg = calcdGrav(h, lat)
%calcGrav Calculates the derivative of the graviatational acceleration for 
%a given altitude and latitude
%   h = altutude above mean sea level (m)
%   lat = launch latitude
MU = 3.98060044e5;      %km^3/s^2
R_EQ = 6378.13649;      %km
omega = 7.2929115e-5;
e = 0.08181980;

R_EARTH = R_EQ/sqrt(1 + e^2/(1 - e^2)*sin(lat)^2);
dg = (1/(R_EARTH + h/1000)^2 - 2*MU/(R_EARTH + h/1000)^3 - omega^2)*1000;
end
