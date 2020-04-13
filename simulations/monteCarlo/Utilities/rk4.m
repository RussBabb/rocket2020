%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xnew, dydx] = rk4(diffeq,xold,t,ytilde,u,simparams)

h=simparams.general.dt;
hh = h/2;
h6 = h/6;

y = xold;
dydx = feval(diffeq,y,t,ytilde,u,simparams);
yt = y + hh*dydx;

dyt = feval(diffeq,yt,t+hh,ytilde,u,simparams);
yt = y +hh*dyt;

dym = feval(diffeq,yt,t+hh,ytilde,u,simparams);
yt = y +h*dym;
dym = dyt + dym;

dyt = feval(diffeq,yt,t+h,ytilde,u,simparams);
yout = y + h6*(dydx+dyt+2*dym);
xnew = yout;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end