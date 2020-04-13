function x_hat = injectErrors(x, delx)
%injectErrors injects errors into the state estimates
%
% Inputs:
%   x = state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   x_hat = estimated state vector(units)
%
% Example Usage
% [ x_hat ] = injectErrors( x, delx )

%Get size of inputs
[n_x,m_x] = size(x);
[~, m_delx] = size(delx);
assert(m_x == m_delx);

%Inject errors
x_hat = zeros(n_x-3,m_x);
for i=1:m_x
    x_hat(1:3,i) = x(1:3,i) - delx(1:3,i);          %Rocket position
    x_hat(4:6,i) = x(4:6,i) - delx(4:6,i);          %Rocket velocity
    delq = [1; -delx(7:9)/2];
    x_hat(7:10,i) = qConjugate(qmult(qConjugate(x(7:10,i)), delq)); 
    x_hat(7:10,i) = normalizeQuat(x_hat(7:10,i));   %Rocket attitude
    x_hat(11,i) = x(14,i) - delx(10,i);             %Rocket mass
    x_hat(12:14,i) = x(15:17,i) - delx(11:13,i);    %Accelerometer bias
    x_hat(15:17,i) = x(18:20,i) - delx(14:16,i);    %Gyroscope bias
    x_hat(18,i) = x(21,i) - delx(17,i);             %Altimeter bias
    x_hat(19,i) = x(22,i) - delx(18,i);             %Airspeed bias
%     x_hat(20:22,i) = x(23:25,i) - delx(19:21,i);    %Wind: 23:25
end
end
