function xNew = integTrap(F,xOld,xPass1, xPass2,t,dt)
%integTrap Integrates the given differential equations F using the
%trapezoidal rule
%   F = function containing the differential equations to integrate
%   xOld = the current state vector
%   xPass1 = values to pass through for the current state
%   xPass2 = values to pass through for the next state
%   t = the current time
%   dt = the integration timestep being used
xNew = xOld + (feval(F,xOld,xPass1,t) + feval(F,xOld,xPass2,t))*dt/2;
end

