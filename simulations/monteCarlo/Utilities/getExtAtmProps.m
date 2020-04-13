function [T_amb, P_amb, rho, c, mu] = getExtAtmProps(Alt, choice)
%getExtAtmProps Returns atmospheric properties from the 1985 US Standard
%Atmosphere extended with properties up to 2000k m.
%   Input: Altitude (km)
%   Output: Ambient Temp (K), Ambient Pressure (Pa), Density (kg/m^3),
%   Local speed of sound (m/s), Local viscosity
%   ====================================================================
%   Properties taken from:
%   US Standard Atmosphere [Alt <= 86 km],
%   "Atmospheric Structure and its Variation in the region from 25 t0
%   120 km" by Groves (1971) using Feb at 30°N [90 km <= Alt <= 110 km],
%   and the Jacchia Atmosphere Model from Jacchia (1971) assuming an
%   Exospheric temperature of 1000K [120 <= Alt <= 2000 km].
%   ====================================================================
% load("ExtendedAtmosphere")
global extStdAtmAvg extStdAtmMax extStdAtmMin

if choice == "avg"
    extStdAtm = extStdAtmAvg;
elseif choice == "min"
    extStdAtm = extStdAtmMin;
elseif choice =="max"
    extStdAtm = extStdAtmMax;
end

N = length(extStdAtm);
minAlt = extStdAtm(1,1);
maxAlt = extStdAtm(N,1);

if Alt < minAlt
    T_amb = extStdAtm(1, 5);
    P_amb = extStdAtm(1, 6)/1000;
    rho = extStdAtm(1, 7);
    c = extStdAtm(1, 8);
    mu = extStdAtm(1, 9)*10^-6;
elseif Alt < maxAlt
    i = 1;
    refAlt = extStdAtm(1,1);
    while Alt >= refAlt
        if Alt == refAlt
            T_amb = extStdAtm(i, 5);
            P_amb = extStdAtm(i, 6)/1000;
            rho = extStdAtm(i, 7);
            c = extStdAtm(i, 8);
            mu = extStdAtm(i, 9)*10^-6;
        elseif Alt < extStdAtm(i+1,1)
            T_amb = (Alt - extStdAtm(i, 1))*(extStdAtm(i+1, 5) - extStdAtm(i, 5))/(extStdAtm(i+1, 1) - extStdAtm(i, 1)) + extStdAtm(i, 5);
            P_amb = ((Alt - extStdAtm(i, 1))*(extStdAtm(i+1, 6) - extStdAtm(i, 6))/(extStdAtm(i+1, 1) - extStdAtm(i, 1)) + extStdAtm(i, 6))/1000;
            rho = (Alt - extStdAtm(i, 1))*(extStdAtm(i+1, 7) - extStdAtm(i, 7))/(extStdAtm(i+1, 1) - extStdAtm(i, 1)) + extStdAtm(i, 7);
            c = (Alt - extStdAtm(i, 1))*(extStdAtm(i+1, 8) - extStdAtm(i, 8))/(extStdAtm(i+1, 1) - extStdAtm(i, 1)) + extStdAtm(i, 8);
            mu = ((Alt - extStdAtm(i, 1))*(extStdAtm(i+1, 9) - extStdAtm(i, 9))/(extStdAtm(i+1, 1) - extStdAtm(i, 1)) + extStdAtm(i, 9))*10^-6;
        end %end if
        
        refAlt = extStdAtm(i+1,1);
        i = i+1;
    end %end while
    
elseif Alt == maxAlt
    T_amb = extStdAtm(N, 5);
    P_amb = extStdAtm(N, 6)/1000;
    rho = extStdAtm(N, 7);
    c = extStdAtm(N, 8);
    mu = extStdAtm(N, 9)*10^-6;
else
    T_amb = 0;
    P_amb = 0;
    rho = 0;
    c = 0;
    mu = 0;
end %end if

end %end function