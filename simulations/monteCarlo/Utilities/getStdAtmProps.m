function [T_amb, P_amb, rho, c, mu] = getStdAtmProps(Alt)
%getStdAtmProps Returns atmospheric properties from the 1985 US Standard
%Atmosphere.
%   Input: Altitude (km)
%   Output: Ambient Temperature (K), Ambient Pressure (kPa), Atmospheric
%   Density (kg/m^3), Speed of sound (m/s), Atmospheric Viscosity
% load("standardAtmosphere")
global stdAtm

N = length(stdAtm);
minAlt = stdAtm(1,1);
maxAlt = stdAtm(N,1);

if Alt < minAlt
    T_amb = stdAtm(N, 5);
    P_amb = stdAtm(N, 6)/1000;
    rho = stdAtm(N, 7);
    c = stdAtm(N, 8);
    mu = stdAtm(N, 9)*10^-6;
elseif Alt < maxAlt
    i = 1;
    refAlt = stdAtm(1,1);
    while Alt >= refAlt
        if Alt == refAlt
            T_amb = stdAtm(i, 5);
            P_amb = stdAtm(i, 6)/1000;
            rho = stdAtm(i, 7);
            c = stdAtm(i, 8);
            mu = stdAtm(i, 9)*10^-6;
        elseif Alt < stdAtm(i+1,1)
            T_amb = (Alt - stdAtm(i, 1))*(stdAtm(i+1, 5) - stdAtm(i, 5))/(stdAtm(i+1, 1) - stdAtm(i, 1)) + stdAtm(i, 5);
            P_amb = ((Alt - stdAtm(i, 1))*(stdAtm(i+1, 6) - stdAtm(i, 6))/(stdAtm(i+1, 1) - stdAtm(i, 1)) + stdAtm(i, 6))/1000;
            rho = (Alt - stdAtm(i, 1))*(stdAtm(i+1, 7) - stdAtm(i, 7))/(stdAtm(i+1, 1) - stdAtm(i, 1)) + stdAtm(i, 7);
            c = (Alt - stdAtm(i, 1))*(stdAtm(i+1, 8) - stdAtm(i, 8))/(stdAtm(i+1, 1) - stdAtm(i, 1)) + stdAtm(i, 8);
            mu = ((Alt - stdAtm(i, 1))*(stdAtm(i+1, 9) - stdAtm(i, 9))/(stdAtm(i+1, 1) - stdAtm(i, 1)) + stdAtm(i, 9))*10^-6;
        end %end if
        
        refAlt = stdAtm(i+1,1);
        i = i+1;
    end %end while
    
elseif Alt == maxAlt
    T_amb = stdAtm(N, 5);
    P_amb = stdAtm(N, 6)/1000;
    rho = stdAtm(N, 7);
    c = stdAtm(N, 8);
    mu = stdAtm(N, 9)*10^-6;
else
    T_amb = 0;
    P_amb = 0;
    rho = 0;
    c = 0;
    mu = 0;
end %end if

end %end function