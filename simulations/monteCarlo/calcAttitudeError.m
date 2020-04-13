function [ rotVector, totalAngle] = calcAttitudeError(q_a_buff, q_b_buff)
%calcAttitudeError calculates error in attitude
%
% Inputs:
%   q_a_buff = 4xN buffer of attitude quaternions
%   q_b_buff = 4xN buffer of attitude quaternions
%
% Outputs
%   rotVector = vector angular error between attitude quaternions (rad)
%   totalAngle = angular error between attitude quaternions (rad)
%
% Example Usage
% [ rotVector, totalAngle] = calcAttitudeError(q_a_buff, q_b_buff)
%

% Author: Randy Christensen
% Date: 08-Aug-2018 00:16:37
% Reference: 
% Copyright 2018 Utah State University
[m_a,N_a] = size(q_a_buff);
[m_b,N_b] = size(q_b_buff);
assert((m_a==4)&(m_b==4))
assert(N_a == N_b);
delta_q = zeros(4,N_a);
for i=1:N_a
    delta_q(:,i) = qmult(q_a_buff(:,i),...
        qConjugate(q_b_buff(:,i)));
end
rotVector = -delta_q(2:4,:)*2;
totalAngle = acos(delta_q(1,:)*2);
end
