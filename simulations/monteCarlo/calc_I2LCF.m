function [T] = calc_I2LCF(t, simpar)
%calc_I2LCF Calculates the rotation matrix for LCI to LCF transformations
%   t = simulation time (seconds)

WDOT_MOON = simpar.general.WDOT_MOON;

% J2000 = 2451545.0; %Epoch of J2000

% get the Julian Date
% JD = getJD(year,month,day,hour, minute,second);

%Days past J2000 epoch
% D = JD - J2000;

% Wmars = 176.630 + WDOT_MARS*D;

w = 0 + WDOT_MOON*t; % Convert to radians

T = [cos(w) sin(w) 0;
    -sin(w) cos(w) 0;
        0     0    1];
end

