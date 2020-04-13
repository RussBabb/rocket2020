function [x, x_hatcor] = correctErrors(x_hat, delx)
%correctState corrects the state vector given an esimated error state
%vector
%
% Inputs:
%   x_hat = estimated state vector (mixed units)
%   delx = error state vector (mixed units)
%
% Outputs
%   x = description (units)
%
% Example Usage
% [ x ] = correctState( x_hat, delx )

%Get size of input
[n_x,m_x] = size(x_hat);
n_x = n_x+3;
[~, m_delx] = size(delx);
assert(m_x == m_delx);
% assert(m_x==1 && m_delx == 1);

%Correct errors
x = zeros(n_x,m_x);
x_hatcor = zeros(n_x-3,m_x);
for i=1:m_x
    x(1:3,i) = x_hat(1:3,i) + delx(1:3,i); %Rocket position
    x(4:6,i) = x_hat(4:6,i) + delx(4:6,i); %Rocket airspeed
    delq = [1; -delx(7:9,i)/2];
    x(7:10,i) = qmult(delq, x_hat(7:10,i));
    x(7:10,i) = normalizeQuat(x(7:10,i)); %Rocket attitude
    %Rocket attitude rate: 11:13
    x(14,i) = x_hat(11,i) + delx(10,i); %Rocket mass
    x(15:17,i) = x_hat(12:14,i) + delx(11:13,i); %Accelerometer bias
    x(18:20,i) = x_hat(15:17,i) + delx(14:16,i); %Gyroscope bias
    x(21,i) = x_hat(18,i) + delx(17,i); %Altimeter bias
    x(22,i) = x_hat(19,i) + delx(18,i); %Airspeed bias
%     x(23:25,i) = x_hat(20:22,i) + delx(19:21,i); %Wind: 23:25
    
    x_hatcor(1:3,i) = x_hat(1:3,i) + delx(1:3,i);       %Rocket position
    x_hatcor(4:6,i) = x_hat(4:6,i) + delx(4:6,i);       %Rocket velocity
    delq = [1; -delx(7:9,i)/2];
    x_hatcor(7:10,i) = qmult(delq, x_hat(7:10,i));
    x_hatcor(7:10,i) = normalizeQuat(x(7:10,i));        %Rocket attitude
    x_hatcor(11,i) = x_hat(11,i) + delx(10,i);          %Rocket mass
    x_hatcor(12:14,i) = x_hat(12:14,i) + delx(11:13,i); %Accelerometer bias
    x_hatcor(15:17,i) = x_hat(15:17,i) + delx(14:16,i); %Gyroscope bias
    x_hatcor(18,i) = x_hat(18,i) + delx(17,i);          %Altimeter bias
    x_hatcor(19,i) = x_hat(19,i) + delx(18,i);          %Airspeed bias
%     x_hatcor(23:25,i) = x_hat(20:22,i) + delx(19:21,i); %Wind: 23:25
end

end
