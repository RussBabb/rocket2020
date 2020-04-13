function I_sp = calcIsp(F_thrust, mdot)
g_0 = 9.806;    %m/s^2
I_sp = 1/g_0*F_thrust/mdot;
end