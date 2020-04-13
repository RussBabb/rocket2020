function [ qba ] = dcm2q( cba )
%dcm2q converts an DCM from the B frame to the A frame to an attitude
%quaternion from the B frame to the A frame
%
% Inputs:
%   cba = DCM from the B frame to the A frame (unitless)
%
% Outputs
%   qba = attitude quaternion from the B frame to the A frame (unitless)
%
% Example Usage
% [ qba ] = dcm2q( cba )
%

% Author: Randy Christensen
% Date: 06-Feb-2019 12:01:27
% Reference: Strapdown Navigation Second Edition, Paul Savage, section
% 3.2.4.3
% Copyright 2018 Utah State University

tr = trace(cba);
Pa = 1 + tr;
Pb = 1 + 2*cba(1,1) - tr;
Pc = 1 + 2*cba(2,2) - tr;
Pd = 1 + 2*cba(3,3) - tr;

if max([Pa,Pb,Pc,Pd]) == Pa
    a = 0.5*sqrt(Pa);
    b = (cba(3,2) - cba(2,3))/(4*a);
    c = (cba(1,3) - cba(3,1))/(4*a);
    d = (cba(2,1) - cba(1,2))/(4*a);
elseif max([Pa,Pb,Pc,Pd]) == Pb
    b = 0.5*sqrt(Pb);
    c = (cba(2,1) + cba(1,2))/(4*b);
    d = (cba(1,3) + cba(3,1))/(4*b);
    a = (cba(3,2) - cba(2,3))/(4*b);
elseif max([Pa,Pb,Pc,Pd]) == Pc
    c = 0.5*sqrt(Pc);
    d = (cba(3,2) + cba(2,3))/(4*c);
    a = (cba(1,3) - cba(3,1))/(4*c);
    b = (cba(2,1) + cba(1,2))/(4*c);
else
    d = 0.5*sqrt(Pd);
    a = (cba(2,1) - cba(1,2))/(4*d);
    b = (cba(1,3) + cba(3,1))/(4*d);
    c = (cba(3,2) + cba(2,3))/(4*d);
end

if a<=0
    a = -a;
    b = -b;
    c = -c;
    d = -d;
end

qba = [a, b, c, d]';
qba = qba/norm(qba);
end
