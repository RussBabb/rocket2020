function Fdrag = calcFdragChute(qbar, t, chute)
%calcFdragChute Calculates the drag force caused by a parachute
%   This function calculates the drag force produced by a parachute as a
%   function of time. During inflation, the inflation curve method is used
%   to estimate drag force. The inflation method uses an exponential
%   relationship to approximate the drag force produced by an inflating
%   parachute. After full inflation, drag is calculated using the standard
%   drag equation.
%
%   t_start <= t < t_fullyInflated -> Fdrag = inflation method
%   t >= t_fullyInflated -> Fdrag = q*Aref*C_D
%
%   qbar - dynamic pressure
%   t - simulation time
%   chute - struct with parachute properties

C_D_chute = chute.C_D; %parachute drag coefficient
A_chute = chute.A_ref; %parachute reference area
C_x = chute.C_x;       %maximum load factor
n = chute.n;           %inflation exponent
t_inflate = chute.t_inflate;
t_deploy = chute.t_deploy;

if ~chute.deployed || t < 0 || t < t_deploy || t_deploy < 0
    Fdrag = 0;
elseif t - t_deploy <= t_inflate
    Fdrag = qbar*C_D_chute*A_chute*C_x*((t - t_deploy)/t_inflate)^n;
else
    Fdrag = qbar*C_D_chute*A_chute;
end

end
