function [ qconj ] = qConjugate( q )
%QCONJUGATE_SAVAGE calculates the conjugate of the quaternion per equation
%3.3.4-14 of Savage
	qconj = zeros(4,1);
	qconj(1) = q(1);
	qconj(2:4) = -q(2:4);
end

