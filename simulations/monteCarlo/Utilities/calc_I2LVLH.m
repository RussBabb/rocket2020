function [T] = calc_I2LVLH(r,v)
%calc_I2LVLH Calculates the Inertial to LVLH rotation matrix
%   r = satellite inertial position
%   v = satellite inertial velocity
iz = r/norm(r);
h = cross(r, v);
iy = h/norm(h);
ix = cross(iy, iz)/norm(cross(iy, iz));

T = [ix, iy, iz]';
end

