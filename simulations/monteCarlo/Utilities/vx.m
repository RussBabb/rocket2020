function vcross = vx(vin)
%vx Vector cross product matrix
%	A = vx(B) returns the vector cross product matrix associated with the
%	vector B such that C = vx(A)*B = A x B
%	Reference Savage Eqn 3.1.1-14
%	Written by Randy Christensen
%	May 20 2007
%#codegen
v = zeros(3,1);
v = vin;
vcross = [ 0   -v(3)  v(2);
	   v(3)   0  -v(1);
	  -v(2) v(1)    0];