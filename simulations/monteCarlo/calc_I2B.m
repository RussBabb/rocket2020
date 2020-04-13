function [T] = calc_I2B(r,v)
%calc_I2B Calculates the Inertial to Body Frame rotation matrix
%   r = satellite inertial position
%   v = satellite inertial velocity
iz = -r/norm(r);
h = cross(r, v);
iy = -h/norm(h);
ix = cross(iy, iz)/norm(cross(iy, iz));

T = [ix, iy, iz]';
end

