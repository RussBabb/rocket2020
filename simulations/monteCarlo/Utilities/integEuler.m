function [xnew,dydx] = integEuler(diffeq,xold,t,ytilde,u,simparams)
%integEuler Integrates the given set of differential equations in
%state-space form using first-order euler integration
%   
dt = simparams.general.dt;
dydx = feval(diffeq,xold,t,ytilde,u,simparams);
xnew = xold + dydx*dt;
end

