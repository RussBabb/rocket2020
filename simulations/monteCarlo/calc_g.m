function g = calc_g(r, n, R_EQ, J2, MU)
%calc_g Calculate the gravity vector including the J2 effect
%   r - position vector
%   n - normal vector
%   R_EQ - body's radius at its equator
%   J2 - the J2 coefficient
%   MU - the body's gravitational parameter

g = -MU*r/norm(r)^3 - MU*J2*R_EQ^2/(2*norm(r)^5)*(6*dot(r, n)*n + 3*r - 15*dot(r/norm(r), n)^2*r);
end

