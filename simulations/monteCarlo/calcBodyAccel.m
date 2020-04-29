function A_b = calcBodyAccel(x, dx, simpar)
%calcBodyAccel Calculates the body accelerations as measured by a
%body-aligned 3-axis accelerometer.
%   Detailed explanation goes here

% Unpack states
r_f = x(1:3);
v_b = x(4:6);
q_b2f = x(7:10);
w_b = x(11:13);

vdot_b = dx(4:6);

% Calculate the body to earth fixed rotation matrix and EF gravity vector
R_b2f = q2dcm(q_b2f);
h = -r_f(3);
g = [0; 0; calcGrav(h, simpar.init.lat)];

% Calculate body accelerations
A_b = vdot_b - R_b2f'*g + cross(w_b, v_b);
end

