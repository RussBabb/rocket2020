function [ delx ] = calcErrors( x_hat, x )
%calcStateErrors Summary of the function goes here
%   Detailed explanation goes here
%
% Inputs:
%   x_hat = estimated state vector(mixed units)
%   x = state vector (mixed units)
%
% Outputs
%   delx = error state vector (mixed units)
%
% Example Usage
% [ delx ] = calcErrors( x_hat, x )

%Get size of input and verify that it is a single vector
[n_x,m_x] = size(x);
[n_xhat, m_xhat] = size(x_hat);
assert(m_x == m_xhat);
assert(n_x-3 == n_xhat);

%Calculate errors
delx = zeros(n_x-4,m_x);
for i=1:m_x
    delx(1:3,i) = x(1:3,i) - x_hat(1:3,i);          %Rocket position
    delx(4:6,i) = x(4:6,i) - x_hat(4:6,i);          %Rocket velocity
    delq = qmult(x(7:10,i), qConjugate(x_hat(7:10,i)));
    delx(7:9,i) = -2*delq(2:4);                     %Rocket attitude
    delx(10,i) = x(14,i) - x_hat(11,i);             %Rocket mass
    delx(11:13,i) = x(15:17,i) - x_hat(12:14,i);    %Accelerometer bias
    delx(14:16,i) = x(18:20,i) - x_hat(15:17,i);    %Gyroscope bias
    delx(17,i) = x(21,i) - x_hat(18,i);             %Altimeter bias
    delx(18,i) = x(22,i) - x_hat(19,i);             %Airspeed bias
%     delx(19:21,i) = x(23:25,i) - x_hat(20:22,i); %Wind: 23:25
end
end
