function dg = calc_dg(r, n,  R_EQ, J2, MU)
%calc_g Calculate the partial derivative of the gravity vector, including
%the J2 effect, with respect to the position vector
%   r - position vector
%   n - normal vector
%   R_EQ - body's radius at its equator
%   J2 - the J2 coefficient
%   MU - the body's gravitational parameter
I3 = eye(3);

ir = r/norm(r);
in = n/norm(n);

k = MU*J2*R_EQ^2/2;
p = 6*dot(r, n)*n + 3*r - 15*dot(r/norm(r), n)^2*r;

dg = -MU/norm(r)^3*(I3 - 3*(ir*ir')) + 5*k*p*ir'/norm(r)^6 - k/norm(r)^5*(6*(n*n') + 3*I3 - -15*dot(ir, n)^2*I3 - 30*r*dot(ir, in)*n'*(I3 - ir*ir')/norm(r));
end

