function [ R ] = q2dcm( q )
%Q2R_SAVAGE Converts a quaternion to a rotation matrix, per section 3.2.4.2
%of Savage

a = q(1);
b = q(2);
c = q(3);
d = q(4);

R11 = a^2 + b^2 - c^2 - d^2;
R12 = 2*(b*c - a*d);
R13 = 2*(b*d + a*c);
R21 = 2*(b*c + a*d);
R22 = a^2 - b^2 + c^2 - d^2;
R23 = 2*(c*d - a*b);
R31 = 2*(b*d - a*c);
R32 = 2*(c*d + a*b);
R33 = a^2 - b^2 - c^2 + d^2;

R = [R11 R12 R13;
    R21 R22 R23;
    R31 R32 R33];
end

