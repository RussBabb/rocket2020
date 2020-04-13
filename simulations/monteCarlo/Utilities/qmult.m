function [ w ] = qmult( u, v )
%QMULT performs quaternion multiplications per equation 3.2.4-28 of Savage
	a = u(1);
	r = u(2:4);
	e = v(1);
	s = v(2:4);
	w_vec = cross(r,s) + a*s + e*r;
	w_scalar = -dot(r,s) + a*e;
	w = [w_scalar; w_vec];
end

